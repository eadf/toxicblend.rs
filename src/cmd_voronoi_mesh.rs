use crate::{
    type_utils, GrowingVob, PB_Command, PB_Face, PB_KeyValuePair, PB_Model, PB_Reply, PB_Vertex,
    TBError,
};

use boostvoronoi as BV;
use cgmath::{EuclideanSpace, SquareMatrix, Transform, UlpsEq};
use itertools::Itertools;
use linestring::linestring_2d::{Aabb2, VoronoiParabolicArc};
use std::collections::HashMap;

/// converts from a private, comparable and hash-able format
/// only use this for bit perfect copies of floats that are f64::is_finite()
#[allow(dead_code)]
#[inline(always)]
fn transmute_to_f64(a: &(u64, u64)) -> cgmath::Point2<f64> {
    cgmath::Point2 {
        x: f64::from_bits(a.0),
        y: f64::from_bits(a.1),
    }
}

/// converts to a private, comparable and hash-able format
/// only use this for bit perfect copies of floats that are f64::is_finite()
/// todo: -0.0 and +0.0 are not identical
#[inline(always)]
#[allow(dead_code)]
fn transmute_to_u64(a: &cgmath::Point2<f64>) -> (u64, u64) {
    (a.x.to_bits(), a.y.to_bits())
}

struct DiagramHelperRw {
    /// a map between hash:able 2d coordinates and the known vertex index of pb_vertices
    vertex_map: ahash::AHashMap<(u64, u64), usize>,
    pb_vertices: Vec<PB_Vertex>,
}

impl Default for DiagramHelperRw {
    fn default() -> Self {
        Self {
            vertex_map: ahash::AHashMap::new(),
            pb_vertices: Vec::new(),
        }
    }
}

impl DiagramHelperRw {
    /// converts to a private, comparable and hash-able format
    /// only use this for matching bit perfect float, f64::is_finite(), copies
    /// todo: -0.0 and +0.0 are not identical
    #[inline(always)]
    fn transmute_xy_to_u64(x: f64, y: f64) -> (u64, u64) {
        (x.to_bits(), y.to_bits())
    }

    #[inline(always)]
    #[allow(dead_code)]
    fn get_dup_checked_vertex(&self, vertex: &[f64; 2]) -> Result<u64, TBError> {
        let key = Self::transmute_xy_to_u64(vertex[0], vertex[1]);
        if let Some(n) = self.vertex_map.get(&key) {
            Ok(*n as u64)
        } else {
            Err(TBError::InternalError(format!(
                "Could not find mapped vertex index of: ({:.12?},{:.12?})",
                vertex[0], vertex[1]
            )))
        }
    }

    /// transform the voronoi Point into a PB point. Perform duplication checks
    #[inline(always)]
    fn place_new_pb_vertex_dup_check(
        &mut self,
        vertex: &[f64; 3],
        inverted_transform: &cgmath::Matrix4<f64>,
    ) -> usize {
        let mut pb_vertices = Vec::<PB_Vertex>::with_capacity(0);
        std::mem::swap(&mut self.pb_vertices, &mut pb_vertices);

        let key = Self::transmute_xy_to_u64(vertex[0], vertex[1]);
        let rv = *self.vertex_map.entry(key).or_insert_with(|| {
            let n = pb_vertices.len();
            let transformed_v = inverted_transform.transform_point(cgmath::Point3 {
                x: vertex[0],
                y: vertex[1],
                z: vertex[2],
            });
            pb_vertices.push(PB_Vertex {
                x: transformed_v.x,
                y: transformed_v.y,
                z: transformed_v.z,
            });
            n
        });

        std::mem::swap(&mut self.pb_vertices, &mut pb_vertices);
        rv
    }

    /// transform the voronoi Point into a PB point. Does not perform any de-duplication checks
    #[inline(always)]
    #[allow(dead_code)]
    fn place_new_pb_vertex_unchecked(
        &mut self,
        vertex: &[f64; 2],
        z_coord: f64,
        inverted_transform: &cgmath::Matrix4<f64>,
    ) -> usize {
        let v = inverted_transform.transform_point(cgmath::Point3 {
            x: vertex[0],
            y: vertex[1],
            z: z_coord,
        });
        let n = self.pb_vertices.len();
        self.pb_vertices.push(PB_Vertex {
            x: v.x,
            y: v.y,
            z: v.z,
        });
        n
    }
}

/// Helper structs that build PB buffer from Diagram
/// This construct contains the read-only items
struct DiagramHelperRo {
    diagram: BV::Diagram<f64>,
    vertices: Vec<BV::Point<i64>>,
    segments: Vec<BV::Line<i64>>,
    //aabb: Aabb2<f64>,
    // this list uses the diagram::Edge id as index
    rejected_edges: vob::Vob<u32>,
    // this list uses the diagram::Vertex id as index
    internal_vertices: vob::Vob<u32>,
    //discrete_distance: f64,
    inverted_transform: cgmath::Matrix4<f64>,
}

impl DiagramHelperRo {
    /// Retrieves a point from the voronoi input in the order it was presented to
    /// the voronoi builder
    #[inline(always)]
    fn retrieve_point(&self, cell_id: BV::CellIndex) -> Result<BV::Point<i64>, TBError> {
        let (index, cat) = self.diagram.get_cell(cell_id)?.get().source_index_2();
        Ok(match cat {
            BV::SourceCategory::SinglePoint => self.vertices[index],
            BV::SourceCategory::SegmentStart => self.segments[index - self.vertices.len()].start,
            BV::SourceCategory::Segment | BV::SourceCategory::SegmentEnd => {
                self.segments[index - self.vertices.len()].end
            }
        })
    }

    /// Retrieves a segment from the voronoi input in the order it was presented to
    /// the voronoi builder
    #[inline(always)]
    fn retrieve_segment(&self, cell_id: BV::CellIndex) -> Result<&BV::Line<i64>, TBError> {
        let cell = self.diagram.get_cell(cell_id)?.get();
        let index = cell.source_index() - self.vertices.len();
        Ok(&self.segments[index])
    }

    /// Convert a secondary edge into a set of vertices.
    /// [maybe start, one mid, maybe end point]
    /// Secondary edges are a bit tricky because they lack the mid-point vertex where they
    /// intersect with the segment that created the edge. So we need to re-create it.
    /// Secondary edges can also be half internal and half external i.e. the two vertices may
    /// be on opposite sides of the inside/outside boundary.
    fn convert_secondary_edge(&self, edge: &BV::Edge) -> Result<Vec<[f64; 3]>, TBError> {
        let edge_id = edge.id();
        let edge_twin_id = self.diagram.edge_get_twin(edge_id)?;
        let cell_id = self.diagram.edge_get_cell(edge_id)?;
        let cell = self.diagram.get_cell(cell_id)?.get();
        let twin_cell_id = self.diagram.get_edge(edge_twin_id)?.get().cell().unwrap();

        let segment = if cell.contains_point() {
            self.retrieve_segment(twin_cell_id)?
        } else {
            self.retrieve_segment(cell_id)?
        };
        let vertex0 = edge.vertex0();
        let vertex1 = self.diagram.edge_get_vertex1(edge_id)?;

        let start_point = if let Some(vertex0) = vertex0 {
            let vertex0 = self.diagram.vertex_get(vertex0)?.get();
            if !self.internal_vertices.get_f(vertex0.get_id().0) {
                None
            } else if vertex0.is_site_point() {
                Some(cgmath::Point3 {
                    x: vertex0.x(),
                    y: vertex0.y(),
                    z: 0.0,
                })
            } else {
                Some(cgmath::Point3 {
                    x: vertex0.x(),
                    y: vertex0.y(),
                    z: f64::NAN,
                })
            }
        } else {
            None
        };

        let end_point = if let Some(vertex1) = vertex1 {
            let vertex1 = self.diagram.vertex_get(vertex1)?.get();
            if !self.internal_vertices.get_f(vertex1.get_id().0) {
                None
            } else if vertex1.is_site_point() {
                Some(cgmath::Point3 {
                    x: vertex1.x(),
                    y: vertex1.y(),
                    z: 0.0,
                })
            } else {
                Some(cgmath::Point3 {
                    x: vertex1.x(),
                    y: vertex1.y(),
                    z: f64::NAN,
                })
            }
        } else {
            None
        };

        let cell_point = {
            let cell_point = if cell.contains_point() {
                self.retrieve_point(cell_id)?
            } else {
                self.retrieve_point(twin_cell_id)?
            };
            cgmath::Point2 {
                x: cell_point.x as f64,
                y: cell_point.y as f64,
            }
        };

        let segment = linestring::linestring_2d::Line2::from([
            segment.start.x as f64,
            segment.start.y as f64,
            segment.end.x as f64,
            segment.end.y as f64,
        ]);

        let mut samples = Vec::<[f64; 3]>::new();

        if let Some(start_point) = start_point {
            if start_point.z.is_finite() {
                samples.push([start_point.x, start_point.y, start_point.z]);
            } else {
                let z_comp = if cell.contains_point() {
                    -linestring::linestring_2d::distance_to_point_squared(
                        cell_point,
                        cgmath::Point2 {
                            x: start_point.x,
                            y: start_point.y,
                        },
                    )
                    .sqrt()
                } else {
                    -linestring::linestring_2d::distance_to_line_squared_safe(
                        segment.start,
                        segment.end,
                        cgmath::Point2 {
                            x: start_point.x,
                            y: start_point.y,
                        },
                    )
                    .sqrt()
                };
                samples.push([start_point.x, start_point.y, z_comp]);
            }
        }

        samples.push([cell_point.x, cell_point.y, 0.0]);

        if let Some(end_point) = end_point {
            if end_point.z.is_finite() {
                samples.push([end_point.x, end_point.y, end_point.z]);
            } else {
                let z_comp = if cell.contains_point() {
                    -linestring::linestring_2d::distance_to_point_squared(
                        cell_point,
                        cgmath::Point2 {
                            x: end_point.x,
                            y: end_point.y,
                        },
                    )
                    .sqrt()
                } else {
                    -linestring::linestring_2d::distance_to_line_squared_safe(
                        segment.start,
                        segment.end,
                        cgmath::Point2 {
                            x: end_point.x,
                            y: end_point.y,
                        },
                    )
                    .sqrt()
                };
                samples.push([end_point.x, end_point.y, z_comp]);
            }
        }
        Ok(samples)
    }

    /// Convert an edge into a set of vertices
    /// primary edges: [start, end point]
    /// curved edges, [start, multiple mid, end point]
    /// todo: try to consolidate code with convert_secondary_edge()
    fn convert_edge(
        &self,
        edge: &BV::Edge,
        discretization_dist: f64,
    ) -> Result<Vec<[f64; 3]>, TBError> {
        let edge_id = edge.id();
        let edge_twin_id = self.diagram.edge_get_twin(edge_id)?;
        let cell_id = self.diagram.edge_get_cell(edge_id)?;
        let cell = self.diagram.get_cell(cell_id)?.get();
        let twin_cell_id = self.diagram.get_edge(edge_twin_id)?.get().cell()?;
        let segment = if cell.contains_point() {
            self.retrieve_segment(twin_cell_id)?
        } else {
            self.retrieve_segment(cell_id)?
        };
        let vertex0 = edge.vertex0();
        let vertex1 = self.diagram.edge_get_vertex1(edge_id)?;

        let (start_point, startpoint_is_internal) = if let Some(vertex0) = vertex0 {
            let vertex0 = self.diagram.vertex_get(vertex0)?.get();

            let start_point = if vertex0.is_site_point() {
                cgmath::Point3 {
                    x: vertex0.x(),
                    y: vertex0.y(),
                    z: 0.0,
                }
            } else {
                cgmath::Point3 {
                    x: vertex0.x(),
                    y: vertex0.y(),
                    z: f64::NAN,
                }
            };
            (
                start_point,
                self.internal_vertices.get_f(vertex0.get_id().0),
            )
        } else {
            return Err(TBError::InternalError(format!(
                "Edge vertex0 could not be found. {}:{}",
                file!(),
                line!()
            )));
        };

        let (end_point, end_point_is_internal) = if let Some(vertex1) = vertex1 {
            let vertex1 = self.diagram.vertex_get(vertex1)?.get();

            let end_point = if vertex1.is_site_point() {
                cgmath::Point3 {
                    x: vertex1.x(),
                    y: vertex1.y(),
                    z: 0.0,
                }
            } else {
                cgmath::Point3 {
                    x: vertex1.x(),
                    y: vertex1.y(),
                    z: f64::NAN,
                }
            };
            (end_point, self.internal_vertices.get_f(vertex1.get_id().0))
        } else {
            return Err(TBError::InternalError(format!(
                "Edge vertex1 could not be found. {}:{}",
                file!(),
                line!()
            )));
        };

        let cell_point = if cell.contains_point() {
            self.retrieve_point(cell_id)?
        } else {
            self.retrieve_point(twin_cell_id)?
        };
        let cell_point = cgmath::Point2 {
            x: cell_point.x as f64,
            y: cell_point.y as f64,
        };

        let segment = linestring::linestring_2d::Line2::from([
            segment.start.x as f64,
            segment.start.y as f64,
            segment.end.x as f64,
            segment.end.y as f64,
        ]);

        let mut samples = Vec::<[f64; 3]>::new();

        if edge.is_curved() {
            let arc = VoronoiParabolicArc::new(
                segment,
                cell_point,
                cgmath::Point2 {
                    x: start_point.x,
                    y: start_point.y,
                },
                cgmath::Point2 {
                    x: end_point.x,
                    y: end_point.y,
                },
            );

            for p in arc.discretise_3d(discretization_dist).points().iter() {
                samples.push([p.x, p.y, p.z]);
            }
        } else {
            if startpoint_is_internal {
                if start_point.z.is_finite() {
                    samples.push([start_point.x, start_point.y, start_point.z]);
                } else {
                    let z_comp = if cell.contains_point() {
                        -linestring::linestring_2d::distance_to_point_squared(
                            cell_point,
                            cgmath::Point2 {
                                x: start_point.x,
                                y: start_point.y,
                            },
                        )
                        .sqrt()
                    } else {
                        -linestring::linestring_2d::distance_to_line_squared_safe(
                            segment.start,
                            segment.end,
                            cgmath::Point2 {
                                x: start_point.x,
                                y: start_point.y,
                            },
                        )
                        .sqrt()
                    };
                    samples.push([start_point.x, start_point.y, z_comp]);
                }
            }

            if end_point_is_internal {
                if end_point.z.is_finite() {
                    samples.push([end_point.x, end_point.y, end_point.z]);
                } else {
                    let z_comp = if cell.contains_point() {
                        -linestring::linestring_2d::distance_to_point_squared(
                            cell_point,
                            cgmath::Point2 {
                                x: end_point.x,
                                y: end_point.y,
                            },
                        )
                        .sqrt()
                    } else {
                        -linestring::linestring_2d::distance_to_line_squared_safe(
                            segment.start,
                            segment.end,
                            cgmath::Point2 {
                                x: end_point.x,
                                y: end_point.y,
                            },
                        )
                        .sqrt()
                    };
                    samples.push([end_point.x, end_point.y, z_comp]);
                }
            }
        }

        Ok(samples)
    }

    /// convert the edges of the diagram into a list of vertices
    fn convert_edges(
        &self,
        discretization_dist: f64,
    ) -> Result<(DiagramHelperRw, ahash::AHashMap<usize, Vec<usize>>), TBError> {
        let mut hrw = DiagramHelperRw::default();
        let mut rv = ahash::AHashMap::<usize, Vec<usize>>::new();

        // this block is not really needed, just makes it convenient to debug
        /*{
            for v in self.vertices.iter() {
                let _ = hrw.place_new_pb_vertex_dup_check(
                    &[v.x as f64, v.y as f64, 0.0],
                    &self.inverted_transform,
                );
            }
            for l in self.segments.iter() {
                let _ = hrw.place_new_pb_vertex_dup_check(
                    &[l.start.x as f64, l.start.y as f64, 0.0],
                    &self.inverted_transform,
                );
                let _ = hrw.place_new_pb_vertex_dup_check(
                    &[l.end.x as f64, l.end.y as f64, 0.0],
                    &self.inverted_transform,
                );
            }
        }*/

        for edge in self.diagram.edges() {
            let edge = edge.get();
            let edge_id = edge.id();
            // secondary edges may be in the rejected list while still contain needed data
            if !edge.is_secondary() && self.rejected_edges.get_f(edge_id.0) {
                // ignore rejected edges, but only non-secondary ones.
                continue;
            }

            let twin_id = edge.twin()?;

            //println!("edge:{:?}", edge_id.0);
            if !rv.contains_key(&twin_id.0) {
                let samples = if edge.is_secondary() {
                    self.convert_secondary_edge(&edge)?
                } else {
                    self.convert_edge(&edge, discretization_dist)?
                };
                let mut pb_edge: Vec<usize> = Vec::with_capacity(samples.len());
                for v in samples.into_iter().map(|coord| {
                    hrw.place_new_pb_vertex_dup_check(&coord, &self.inverted_transform)
                }) {
                    if !pb_edge.contains(&v) {
                        pb_edge.push(v);
                    }
                }

                let _ = rv.insert(edge_id.0, pb_edge);
            } else {
                // ignore edge because the twin is already processed
            }
        }
        Ok((hrw, rv))
    }

    /// if a cell contains a segment the pb_face should be split into two faces, one
    /// on each side of the segment.
    fn split_pb_face_by_segment(
        &self,
        v0n: u64,
        v1n: u64,
        pb_face: &PB_Face,
    ) -> Result<Option<(PB_Face, PB_Face)>, TBError> {
        if let Some(v0i) = pb_face.vertices.iter().position(|x| x == &v0n) {
            if let Some(v1i) = pb_face.vertices.iter().position(|x| x == &v1n) {
                let mut a = Vec::<u64>::new();
                let mut b = Vec::<u64>::new();
                if v0i < v1i {
                    // todo, could this be made into a direct .collect() ?
                    for i in (0..=v0i).chain(v1i..pb_face.vertices.len()) {
                        a.push(pb_face.vertices[i]);
                    }
                    for i in v0i..=v1i {
                        b.push(pb_face.vertices[i]);
                    }
                } else {
                    // todo, could this be made into a direct .collect() ?
                    for i in (0..=v1i).chain(v0i..pb_face.vertices.len()) {
                        a.push(pb_face.vertices[i]);
                    }
                    for i in v1i..=v0i {
                        b.push(pb_face.vertices[i]);
                    }
                };
                return Ok(Some((PB_Face { vertices: a }, PB_Face { vertices: b })));
            }
        }
        Ok(None)
    }

    /// Iterate over each cell, generate mesh
    fn iterate_cells(
        &self,
        mut dhrw: DiagramHelperRw,
        edge_map: ahash::AHashMap<usize, Vec<usize>>,
    ) -> Result<(Vec<PB_Face>, Vec<PB_Vertex>), TBError> {
        let mut pb_faces = Vec::<PB_Face>::new();

        for cell in self.diagram.cells().iter() {
            let cell = cell.get();
            let cell_id = cell.id();

            if cell.contains_point() {
                let cell_point = {
                    let cp = self.retrieve_point(cell_id)?;
                    dhrw.place_new_pb_vertex_dup_check(
                        &[cp.x as f64, cp.y as f64, 0.0],
                        &self.inverted_transform,
                    ) as u64
                };

                for edge_id in self.diagram.cell_edge_iterator(cell_id) {
                    let edge = self.diagram.get_edge(edge_id)?.get();
                    let twin_id = edge.twin()?;

                    if self.rejected_edges.get_f(edge_id.0) && !edge.is_secondary() {
                        continue;
                    }
                    let mod_edge: Box<dyn ExactSizeIterator<Item = &usize>> = {
                        if let Some(e) = edge_map.get(&edge_id.0) {
                            Box::new(e.iter())
                        } else {
                            Box::new(
                                edge_map
                                    .get(&twin_id.0)
                                    .ok_or_else(|| {
                                        TBError::InternalError(format!(
                                            "could not get twin edge, {}, {}",
                                            file!(),
                                            line!()
                                        ))
                                    })?
                                    .iter()
                                    .rev(),
                            )
                        }
                    };

                    for (a, b) in mod_edge.tuple_windows::<(_, _)>() {
                        let a = *a as u64;
                        let b = *b as u64;

                        if a != cell_point && b != cell_point {
                            let mut pb_face = PB_Face {
                                vertices: Vec::new(),
                            };
                            let mut face = vec![a, b, cell_point];
                            pb_face.vertices.append(&mut face);
                            //print!(" pb:{:?},", pb_face.vertices);
                            if pb_face.vertices.len() > 2 {
                                pb_faces.push(pb_face);
                            } else {
                                //print!("ignored ");
                            }
                        }
                    }
                }
                //println!();
            }
            if cell.contains_segment() {
                let segment = self.retrieve_segment(cell_id)?;
                let v0n = dhrw.place_new_pb_vertex_dup_check(
                    &[segment.start.x as f64, segment.start.y as f64, 0.0],
                    &self.inverted_transform,
                );
                let v1n = dhrw.place_new_pb_vertex_dup_check(
                    &[segment.end.x as f64, segment.end.y as f64, 0.0],
                    &self.inverted_transform,
                );
                //print!("SCell:{} v0:{} v1:{} ", cell_id.0, v0n, v1n);
                let mut new_face = PB_Face {
                    vertices: Vec::new(),
                };
                for edge_id in self.diagram.cell_edge_iterator(cell_id) {
                    let edge = self.diagram.get_edge(edge_id)?.get();
                    let twin_id = edge.twin()?;

                    let mod_edge: Box<dyn ExactSizeIterator<Item = &usize>> = {
                        if let Some(e) = edge_map.get(&edge_id.0) {
                            Box::new(e.iter())
                        } else if let Some(e) = edge_map.get(&twin_id.0) {
                            Box::new(e.iter().rev())
                        } else {
                            //let e:Option<usize> = None;
                            Box::new(None.iter())
                        }
                    };

                    for v in mod_edge.map(|x| *x as u64) {
                        //print! {"{:?},", v};
                        if !new_face.vertices.contains(&v) {
                            new_face.vertices.push(v);
                        }
                    }
                }
                if let Some((split_a, split_b)) =
                    self.split_pb_face_by_segment(v0n as u64, v1n as u64, &new_face)?
                {
                    if split_a.vertices.len() > 2 {
                        pb_faces.push(split_a);
                    }
                    if split_b.vertices.len() > 2 {
                        pb_faces.push(split_b);
                    }
                } else if new_face.vertices.len() > 2 {
                    pb_faces.push(new_face);
                }
            }
        }
        Ok((pb_faces, dhrw.pb_vertices))
    }
}

#[allow(clippy::type_complexity)]
fn parse_input(
    input_pb_model: &PB_Model,
    cmd_arg_max_voronoi_dimension: f64,
) -> Result<
    (
        Vec<BV::Point<i64>>,
        Vec<BV::Line<i64>>,
        Aabb2<f64>,
        cgmath::Matrix4<f64>,
    ),
    TBError,
> {
    let mut aabb = linestring::linestring_3d::Aabb3::<f64>::default();
    for v in input_pb_model.vertices.iter() {
        aabb.update_point(cgmath::Point3::new(v.x as f64, v.y as f64, v.z as f64))
    }

    let (plane, transform, vor_aabb)= centerline::get_transform_relaxed(
        aabb,
        cmd_arg_max_voronoi_dimension,
        super::EPSILON,
        f64::default_max_ulps(),
    ).map_err(|_|{
        let aabb_d = aabb.get_high().unwrap() - aabb.get_low().unwrap();
        let aabb_c = (aabb.get_high().unwrap().to_vec() + aabb.get_low().unwrap().to_vec())/2.0;
        TBError::InputNotPLane(format!(
            "Input data not in one plane and/or plane not intersecting origin: Δ({},{},{}) C({},{},{})",
            aabb_d.x, aabb_d.y, aabb_d.z, aabb_c.x, aabb_c.y, aabb_c.z))
    })?;

    let invers_transform = transform
        .invert()
        .ok_or(TBError::CouldNotCalculateInvertMatrix)?;

    println!("voronoi: data was in plane:{:?} aabb:{:?}", plane, aabb);
    //println!("input Lines:{:?}", input_pb_model.vertices);

    let mut vor_lines = Vec::<BV::Line<i64>>::with_capacity(input_pb_model.faces.len());
    let vor_vertices: Vec<BV::Point<i64>> = input_pb_model
        .vertices
        .iter()
        .map(|vertex| {
            let p = type_utils::xy_to_2d(&transform.transform_point(cgmath::Point3 {
                x: vertex.x,
                y: vertex.y,
                z: vertex.z,
            }));
            BV::Point {
                x: p.x as i64,
                y: p.y as i64,
            }
        })
        .collect();
    let mut used_vertices = vob::Vob::<u32>::fill(vor_vertices.len());

    for face in input_pb_model.faces.iter() {
        match face.vertices.len() {
            3..=usize::MAX => return Err(TBError::ModelContainsFaces("Model can't contain any faces, only edges and points. Use the 2d_outline tool to remove faces".to_string())),
            2 => {
                let v0 = face.vertices[0] as usize;
                let v1 = face.vertices[1] as usize;

                vor_lines.push(BV::Line {
                    start: vor_vertices[v0],
                    end: vor_vertices[v1],
                });
                let _ = used_vertices.set(v0, true);
                let _ = used_vertices.set(v1, true);
            },
            // This does not work, face.len() is never 1
            //1 => points.push(vertices_2d[face.vertices[0] as usize]),
            _ => (),
        }
    }
    // save the unused vertices as points
    let vor_vertices: Vec<BV::Point<i64>> = vor_vertices
        .into_iter()
        .enumerate()
        .filter(|x| !used_vertices.get_f(x.0))
        .map(|x| x.1)
        .collect();
    Ok((vor_vertices, vor_lines, vor_aabb, invers_transform))
}

/// from the list of rejected edges, find a list of internal (non-rejected) vertices.
fn find_internal_vertices(
    diagram: &BV::Diagram<f64>,
    rejected_edges: &vob::Vob<u32>,
) -> Result<vob::Vob<u32>, TBError> {
    let mut internal_vertices = vob::Vob::<u32>::fill(diagram.vertices().len());
    for (_, e) in diagram
        .edges()
        .iter()
        .enumerate()
        .filter(|(eid, _)| !rejected_edges.get_f(*eid))
    {
        let e = e.get();
        if e.is_primary() {
            if let Some(v0) = e.vertex0() {
                let _ = internal_vertices.set(v0.0, true);
            }
            if let Some(v1) = diagram.edge_get_vertex1(e.id())? {
                let _ = internal_vertices.set(v1.0, true);
            }
        }
    }
    for (vid, _) in diagram
        .vertices()
        .iter()
        .enumerate()
        .filter(|(_, v)| v.get().is_site_point())
    {
        let _ = internal_vertices.set(vid, true);
    }
    Ok(internal_vertices)
}

/// Runs boost voronoi over the input and generates to output model.
/// Removes the external edges as we can't handle infinite length edges in blender.
fn voronoi_mesh(
    input_pb_model: &PB_Model,
    cmd_arg_max_voronoi_dimension: f64,
    cmd_discretization_distance: f64,
) -> Result<PB_Model, TBError> {
    let (vor_vertices, vor_lines, vor_aabb2, inverted_transform) =
        parse_input(input_pb_model, cmd_arg_max_voronoi_dimension)?;
    let vor_diagram = {
        BV::Builder::default()
            .with_vertices(vor_vertices.iter())?
            .with_segments(vor_lines.iter())?
            .build()?
    };
    let discretization_dist = {
        let max_dist = vor_aabb2.get_high().unwrap() - vor_aabb2.get_low().unwrap();
        let max_dist = max_dist.x.max(max_dist.y);
        cmd_discretization_distance * max_dist / 100.0
    };

    let reject_edges = super::voronoi_utils::reject_external_edges(&vor_diagram)?;
    let internal_vertices = find_internal_vertices(&vor_diagram, &reject_edges)?;
    let diagram_helper = DiagramHelperRo {
        vertices: vor_vertices,
        segments: vor_lines,
        diagram: vor_diagram,
        rejected_edges: reject_edges,
        internal_vertices,
        inverted_transform,
    };

    let (dhrw, mod_edges) = diagram_helper.convert_edges(discretization_dist)?;
    let (pb_faces, pb_vertices) = diagram_helper.iterate_cells(dhrw, mod_edges)?;

    Ok(PB_Model {
        name: input_pb_model.name.clone(),
        world_orientation: input_pb_model.world_orientation.clone(),
        faces: pb_faces,
        vertices: pb_vertices,
    })
}

/// Run the voronoi_mesh command
pub(crate) fn command(
    a_command: PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!(
        r#"                                              __                         /\
    ___  __ ____ _______  ____   ____   ____ |__|   _____   ____   ______  |__
    \  \/ // __ \\_  __ \/ __ \ /    \ / __ \|  |  /     \_/ __ \ /  ___/  |  \
     \   /(  \_\ )|  | \/  \_\ )   |  \  \_\ )  | |  | |  \  ___/_\___ \|      \
      \_/  \____/ |__|   \____/|___|  /\____/|__| |__|_|  /\___  /____  \___|  /
                                    \/                  \/     \/     \/     \/"#
    );
    if a_command.models.is_empty() {
        return Err(TBError::InvalidInputData(
            "This operation requires ome input model".to_string(),
        ));
    }

    if a_command.models.len() > 1 {
        return Err(TBError::InvalidInputData(
            "This operation only supports one model as input".to_string(),
        ));
    }

    let cmd_arg_max_voronoi_dimension = {
        let tmp_value = super::DEFAULT_MAX_VORONOI_DIMENSION.to_string();
        let value = options.get("MAX_VORONOI_DIMENSION").unwrap_or(&tmp_value);
        value.parse::<f64>().map_err(|_| {
            TBError::InvalidInputData(format!(
                "Could not parse the MAX_VORONOI_DIMENSION parameter {:?}",
                value
            ))
        })?
    };
    if !(super::DEFAULT_MAX_VORONOI_DIMENSION..100_000_000.0)
        .contains(&cmd_arg_max_voronoi_dimension)
    {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of MAX_VORONOI_DIMENSION is [{}..100_000_000[% :({})",
            super::DEFAULT_MAX_VORONOI_DIMENSION,
            cmd_arg_max_voronoi_dimension
        )));
    }
    let cmd_arg_discretization_distance = {
        let tmp_value = super::DEFAULT_VORONOI_DISCRETE_DISTANCE.to_string();
        let value = options.get("DISTANCE").unwrap_or(&tmp_value);
        value.parse::<f64>().map_err(|_| {
            TBError::InvalidInputData(format!(
                "Could not parse the DISTANCE parameter {:?}",
                value
            ))
        })?
    };
    if !(super::DEFAULT_VORONOI_DISCRETE_DISTANCE..5.0).contains(&cmd_arg_discretization_distance) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of DISTANCE is [{}..5.0[% :({})",
            super::DEFAULT_VORONOI_DISCRETE_DISTANCE,
            cmd_arg_discretization_distance
        )));
    }
    // used for simplification and discretization distance
    let max_distance = cmd_arg_max_voronoi_dimension * cmd_arg_discretization_distance / 100.0;

    // we already tested that there is only one model
    for model in a_command.models.iter().take(1) {
        println!("model.name:{:?}, ", model.name);
        println!("model.vertices:{:?}, ", model.vertices.len());
        println!("model.faces:{:?}, ", model.faces.len());
        println!(
            "model.world_orientation:{:?}, ",
            model.world_orientation.as_ref().map_or(0, |_| 16)
        );
        println!("MAX_VORONOI_DIMENSION:{:?}", cmd_arg_max_voronoi_dimension);
        println!(
            "VORONOI_DISCRETE_DISTANCE:{:?}%",
            cmd_arg_discretization_distance
        );
        println!("max_distance:{:?}", max_distance);
        println!();
    }

    // we already tested a_command.models.len()
    let input_model = &a_command.models[0];
    let output_model = voronoi_mesh(
        input_model,
        cmd_arg_max_voronoi_dimension,
        cmd_arg_discretization_distance,
    )?;
    let mut reply = PB_Reply {
        options: vec![PB_KeyValuePair {
            key: "ONLY_EDGES".to_string(),
            value: "False".to_string(),
        }],
        models: Vec::with_capacity(1),
        models32: Vec::with_capacity(0),
    };
    println!("Total number of vertices: {}", output_model.vertices.len());
    println!("Total number of faces: {}", output_model.faces.len());
    reply.models.push(output_model);
    Ok(reply)
}
