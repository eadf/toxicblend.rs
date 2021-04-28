use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex as PB_Vertex;
use boostvoronoi::builder as VB;
use boostvoronoi::diagram as VD;
use cgmath::{EuclideanSpace, SquareMatrix, Transform, UlpsEq};
//use itertools::Itertools;
use linestring::cgmath_2d::Aabb2;
use linestring::cgmath_2d::VoronoiParabolicArc;
use std::collections::{HashMap, VecDeque};
//use boostvoronoi::{InputType, OutputType};

/// converts from a private, comparable and hash-able format
/// only use this for floats that are f64::is_finite()
#[allow(dead_code)]
#[inline(always)]
fn transmute_to_f64(a: &(u64, u64)) -> cgmath::Point2<f64> {
    cgmath::Point2 {
        x: f64::from_bits(a.0),
        y: f64::from_bits(a.1),
    }
}

/// converts to a private, comparable and hash-able format
/// only use this for floats that are f64::is_finite()
#[inline(always)]
fn transmute_to_u64(a: &cgmath::Point2<f64>) -> (u64, u64) {
    (a.x.to_bits(), a.y.to_bits())
}

/// Helper structs that build PB buffer from VoronoiDiagram
struct DiagramHelper {
    diagram: VD::VoronoiDiagram<i64, f64>,
    vertices: Vec<boostvoronoi::Point<i64>>,
    segments: Vec<boostvoronoi::Line<i64>>,
    aabb: Aabb2<f64>,
    rejected_edges: Option<yabf::Yabf>,
    discrete_distance: f64,
    remove_externals: bool,
}

impl DiagramHelper {
    /// Mark infinite edges and their adjacent edges as EXTERNAL.
    /// Also mark secondary edges if self.remove_secondary_edges is set
    fn reject_edges(&mut self) -> Result<(), TBError> {
        let mut rejected_edges = yabf::Yabf::default();
        // ensure capacity of bit field by setting last bit +1 to true
        rejected_edges.set_bit(self.diagram.edges().len(), true);

        for edge in self.diagram.edges().iter() {
            let edge = edge.get();
            let edge_id = edge.get_id();

            if !self
                .diagram
                .edge_is_finite(Some(edge_id))
                .ok_or_else(|| TBError::InternalError("Could not get edge status".to_string()))?
            {
                self.mark_connected_edges(edge_id, &mut rejected_edges, true)?;
                rejected_edges.set_bit(edge_id.0, true);
            }
        }
        self.rejected_edges = Some(rejected_edges);
        Ok(())
    }

    /// Marks this edge and all other edges connecting to it via vertex1.
    /// Repeat stops when connecting to input geometry.
    /// if 'initial' is set to true it will search both ways, edge and the twin edge.
    /// 'initial' will be set to false when going past the first edge
    fn mark_connected_edges(
        &self,
        edge_id: VD::VoronoiEdgeIndex,
        marked_edges: &mut yabf::Yabf,
        initial: bool,
    ) -> Result<(), TBError> {
        if marked_edges.bit(edge_id.0) {
            return Ok(());
        }

        let mut initial = initial;
        let mut queue = VecDeque::<VD::VoronoiEdgeIndex>::new();
        queue.push_front(edge_id);

        'outer: while !queue.is_empty() {
            // unwrap is safe since we just checked !queue.is_empty()
            let edge_id = queue.pop_back().unwrap();

            if marked_edges.bit(edge_id.0) {
                initial = false;
                continue 'outer;
            }

            let v1 = self.diagram.edge_get_vertex1(Some(edge_id));
            if self.diagram.edge_get_vertex0(Some(edge_id)).is_some() && v1.is_none() {
                // this edge leads to nowhere
                marked_edges.set_bit(edge_id.0, true);
                initial = false;
                continue 'outer;
            }
            marked_edges.set_bit(edge_id.0, true);

            #[allow(unused_assignments)]
            if initial {
                initial = false;
                queue.push_back(self.diagram.edge_get_twin(Some(edge_id)).ok_or_else(|| {
                    TBError::InternalError("Could not get edge twin".to_string())
                })?);
            } else {
                marked_edges.set_bit(
                    self.diagram
                        .edge_get_twin(Some(edge_id))
                        .ok_or_else(|| {
                            TBError::InternalError("Could not get edge twin".to_string())
                        })?
                        .0,
                    true,
                );
            }

            if v1.is_none()
                || !self.diagram.edges()[(Some(edge_id))
                    .ok_or_else(|| TBError::InternalError("Could not get edge twin".to_string()))?
                    .0]
                    .get()
                    .is_primary()
            {
                // stop traversing this line if vertex1 is not found or if the edge is not primary
                initial = false;
                continue 'outer;
            }
            // v1 is always Some from this point on
            if let Some(v1) = v1 {
                let v1 = self
                    .diagram
                    .vertex_get(Some(v1))
                    .ok_or_else(|| {
                        TBError::InternalError("Could not get expected vertex".to_string())
                    })?
                    .get();
                if v1.is_site_point() {
                    // stop iterating line when site points detected
                    initial = false;
                    continue 'outer;
                }
                //self.reject_vertex(v1, color);
                let mut e = v1.get_incident_edge();
                let v_incident_edge = e;
                while let Some(this_edge) = e {
                    if !marked_edges.bit(this_edge.0) {
                        queue.push_back(this_edge);
                    }
                    e = self.diagram.edge_rot_next(Some(this_edge));
                    if e == v_incident_edge {
                        break;
                    }
                }
            }
            initial = false;
        }
        Ok(())
    }

    /// converts to a private, comparable and hash-able format
    /// only use this for floats that are f64::is_finite()
    #[inline(always)]
    fn transmute_to_u64(x: f64, y: f64) -> (u64, u64) {
        (x.to_bits(), y.to_bits())
    }

    #[inline(always)]
    fn get_dup_checked_vertex(
        vertex: &[f64; 2],
        new_vertex_map: &mut ahash::AHashMap<(u64, u64), usize>,
    ) -> Result<u64, TBError> {
        let key = Self::transmute_to_u64(vertex[0], vertex[1]);
        if let Some(n) = new_vertex_map.get(&key) {
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
        vertex: &[f64; 2],
        new_vertex_map: &mut ahash::AHashMap<(u64, u64), usize>,
        pb_vertices: &mut Vec<PB_Vertex>,
        inverted_transform: &cgmath::Matrix4<f64>,
        z_coord: f64,
    ) -> usize {
        let key = Self::transmute_to_u64(vertex[0], vertex[1]);
        *new_vertex_map.entry(key).or_insert_with(|| {
            let n = pb_vertices.len();
            let transformed_v = inverted_transform.transform_point(cgmath::Point3 {
                x: vertex[0],
                y: vertex[1],
                z: z_coord,
            });
            pb_vertices.push(PB_Vertex {
                x: transformed_v.x,
                y: transformed_v.y,
                z: transformed_v.z,
            });
            /*println!(
                "Found a new vertex:{:.12}, {:.12} named it {}",
                vertex[0], vertex[1], n
            );
            */
            n
        })
    }

    /// transform the voronoi Point into a PB point. Does not perform any duplication checks
    #[inline(always)]
    #[allow(dead_code)]
    fn place_new_pb_vertex_unchecked(
        vertex: &[f64; 2],
        pb_vertices: &mut Vec<PB_Vertex>,
        inverted_transform: &cgmath::Matrix4<f64>,
        z_coord: f64,
    ) -> usize {
        let v = inverted_transform.transform_point(cgmath::Point3 {
            x: vertex[0],
            y: vertex[1],
            z: z_coord,
        });
        let n = pb_vertices.len();
        pb_vertices.push(PB_Vertex {
            x: v.x,
            y: v.y,
            z: v.z,
        });
        n
    }

    /// Convert voronoi cells into PB_Model data
    #[allow(unused_labels)]
    fn convert_cells(
        &mut self,
        new_vertex_map: &mut ahash::AHashMap<(u64, u64), usize>,
        pb_vertices: &mut Vec<PB_Vertex>,
        pb_faces: &mut Vec<PB_Face>,
        inverted_transform: &cgmath::Matrix4<f64>,
    ) -> Result<(), TBError> {
        let max_dist = self.aabb.get_high().unwrap() - self.aabb.get_low().unwrap();
        let max_dist = max_dist.x.max(max_dist.y);
        let max_dist = self.discrete_distance * max_dist / 100.0;

        // an affine that does nothing
        // Todo: replace with an option
        //let dummy_affine = boostvoronoi::visual_utils::SimpleAffine::<i64, f64>::default();
        //println!(
        //    "convert_cells, max_dist:{}, self.cmd_discrete_distance:{}%",
        //    max_dist, self.discrete_distance
        //);

        let rejected_edges = self
            .rejected_edges
            .take()
            .unwrap_or_else(|| yabf::Yabf::with_capacity(0));

        //println!("There are {} faces", self.diagram.cells().len());

        'cell_loop: for (cell_id, cell_c) in self.diagram.cell_iter().enumerate()
        /*.skip(1).take(1)*/
        {
            assert_eq!(cell_c.get().get_id(), cell_id);
            let mut kill_pill = 100;

            let mut edge_id = cell_c
                .get()
                .get_incident_edge()
                .ok_or_else(|| TBError::InternalError("Could not unwrap edge".to_string()))?;
            let start_edge: VD::VoronoiEdgeIndex = edge_id;

            let cell1_id = self.diagram.edge_get_cell(Some(edge_id)).ok_or_else(|| {
                TBError::InternalError(format!(
                    "Could not get cell id for edge:{}. {}",
                    edge_id.0,
                    line!()
                ))
            })?;

            let cell1 = self.diagram.get_cell(cell1_id).get();

            let mut pb_face = PB_Face { vertices: vec![] };

            'edge_loop: loop {
                let edge = self.diagram.get_edge(edge_id).get();
                //println!("@{:?}", edge);

                let edge_twin_id = self.diagram.edge_get_twin(Some(edge_id)).ok_or_else(|| {
                    TBError::InternalError(format!("Could not get twin. {}", line!()))
                })?;
                let edge_is_finite =
                    self.diagram.edge_is_finite(Some(edge_id)).ok_or_else(|| {
                        TBError::InternalError(format!("Could not get edge. {}", line!()))
                    })?;

                let (start_point, end_point) = if !edge_is_finite {
                    self.replace_infinite_with_origin(edge_id)?
                } else {
                    // Edge is finite so we know that vertex0 and vertex1 is_some()

                    let vertex0 = self
                        .diagram
                        .vertex_get(edge.vertex0())
                        .ok_or_else(|| {
                            TBError::InternalError(format!("Could not find vertex 0. {}", line!()))
                        })?
                        .get();

                    let vertex1 =
                        self.diagram
                            .edge_get_vertex1(Some(edge_id))
                            .ok_or_else(|| {
                                TBError::InternalError(format!(
                                    "Could not find vertex 1. {}",
                                    line!()
                                ))
                            })?;

                    let vertex1 = self
                        .diagram
                        .vertex_get(Some(vertex1))
                        .ok_or_else(|| {
                            TBError::InternalError(format!("Could not find vertex 1. {}", line!()))
                        })?
                        .get();

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
                    (start_point, end_point)
                };

                let twin_cell_id = self.diagram.get_edge(edge_twin_id).get().cell().unwrap();

                let cell_point = if cell1.contains_point() {
                    self.retrieve_point(cell1_id)
                } else {
                    self.retrieve_point(twin_cell_id)
                };
                let cell_point = cgmath::Point2 {
                    x: cell_point.x as f64,
                    y: cell_point.y as f64,
                };

                let segment = if cell1.contains_point() {
                    self.retrieve_segment(twin_cell_id)
                } else {
                    self.retrieve_segment(cell1_id)
                };
                let segment = linestring::cgmath_2d::Line2::from([
                    segment.start.x as f64,
                    segment.start.y as f64,
                    segment.end.x as f64,
                    segment.end.y as f64,
                ]);

                let mut samples = Vec::<[f64; 3]>::new();

                if edge.is_curved() {
                    /*println!(
                        "curved !! start_point={:?}, end_point={:?}",
                        start_point, end_point
                    );*/

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
                    let arc = arc.discretise_3d(max_dist);
                    for p in arc.points().iter() {
                        samples.push([p.x, p.y, p.z]);
                    }
                } else {
                    if start_point.z.is_finite() {
                        samples.push([start_point.x, start_point.y, start_point.z]);
                    } else {
                        let z_comp = if cell1.contains_point() {
                            -linestring::cgmath_2d::distance_to_point_squared(
                                &cell_point,
                                &cgmath::Point2 {
                                    x: start_point.x,
                                    y: start_point.y,
                                },
                            )
                            .sqrt()
                        } else {
                            -linestring::cgmath_2d::distance_to_line_squared_safe(
                                &segment.start,
                                &segment.end,
                                &cgmath::Point2 {
                                    x: start_point.x,
                                    y: start_point.y,
                                },
                            )
                            .sqrt()
                        };
                        samples.push([start_point.x, start_point.y, z_comp]);
                    }
                    if end_point.z.is_finite() {
                        samples.push([end_point.x, end_point.y, end_point.z]);
                    } else {
                        let z_comp = if cell1.contains_point() {
                            -linestring::cgmath_2d::distance_to_point_squared(
                                &cell_point,
                                &cgmath::Point2 {
                                    x: end_point.x,
                                    y: end_point.y,
                                },
                            )
                            .sqrt()
                        } else {
                            -linestring::cgmath_2d::distance_to_line_squared_safe(
                                &segment.start,
                                &segment.end,
                                &cgmath::Point2 {
                                    x: end_point.x,
                                    y: end_point.y,
                                },
                            )
                            .sqrt()
                        };
                        samples.push([end_point.x, end_point.y, z_comp]);
                    }
                }

                //println!("samples {:?}", samples);
                if !samples.is_empty() {
                    let processed_samples: Vec<usize> = samples
                        .iter()
                        .map(|v| {
                            Self::place_new_pb_vertex_dup_check(
                                &[v[0], v[1]],
                                new_vertex_map,
                                pb_vertices,
                                &inverted_transform,
                                v[2],
                            )
                        })
                        .collect();
                    if (!edge.is_secondary()) && rejected_edges.bit(edge_id.0) {
                        // don't collect any of the edges/vertices of this cell
                        //println!(
                        //    "***************   Edge {} was rejected, skipping edge",
                        //    edge_id.0
                        //);
                    } else {
                        for v in processed_samples.into_iter() {
                            let v = v as u64;
                            if pb_face.vertices.is_empty() || *(pb_face.vertices.last().unwrap()) != v {
                                pb_face.vertices.push(v);
                            } else {
                                //println!("would have placed {} again", v);
                            }
                        }
                    }
                }
                kill_pill -= 1;
                if kill_pill == 0 {
                    return Err(TBError::InternalError(
                        "convert_cells() 2: Detected infinite loop".to_string(),
                    ));
                }
                edge_id = self.diagram.edge_get_next(Some(edge_id)).ok_or_else(|| {
                    TBError::InternalError("Could not unwrap next edge".to_string())
                })?;
                if edge_id == start_edge {
                    //println!("edge_id == start_edge {}=={}", edge_id.0, start_edge.0);
                    break 'edge_loop;
                }
            }
            if !pb_face.vertices.is_empty() {
                let first = *pb_face.vertices.first().unwrap();
                if *pb_face.vertices.last().unwrap() == first {
                    let _ = pb_face.vertices.pop();
                    //println!(
                    //    "edge geometry face had double end point {:?} {}",
                    //    &pb_face.vertices, first
                    //);
                }
            }
            //println!("Cell:{} produced:{:?}", cell_id, pb_face);
            if cell_c.get().contains_segment() {
                if let Some(split) = self.split_pb_faces(
                    &pb_face,
                    VD::VoronoiCellIndex(cell_id),
                    edge_id,
                    new_vertex_map,
                )? {
                    if !split.0.vertices.is_empty() {
                        pb_faces.push(split.0);
                    }
                    if !split.1.vertices.is_empty() {
                        pb_faces.push(split.1);
                    }
                } else if !pb_face.vertices.is_empty() {
                    pb_faces.push(pb_face);
                }
            } else if !pb_face.vertices.is_empty() {
                pb_faces.push(pb_face);
            }

            //break 'cell_loop
        }
        self.rejected_edges = Some(rejected_edges);
        /*println!("pb vertices:{:?}", pb_vertices.len());
        for f in pb_faces.iter() {
            println!(" pb face:{:2?}", f.vertices);
        }
        */
        Ok(())
    }

    fn split_pb_faces(
        &self,
        pb_face: &PB_Face,
        cell_id: VD::VoronoiCellIndex,
        edge_id: VD::VoronoiEdgeIndex,
        new_vertex_map: &mut ahash::AHashMap<(u64, u64), usize>,
    ) -> Result<Option<(PB_Face, PB_Face)>, TBError> {
        let cell = self.diagram.get_cell(cell_id).get();
        let twin_id = self.diagram.edge_get_twin(Some(edge_id)).unwrap();
        let twin_cell_id = self.diagram.edge_get_cell(Some(twin_id)).unwrap();

        let segment = if cell.contains_point() {
            self.retrieve_segment(twin_cell_id)
        } else {
            self.retrieve_segment(cell_id)
        };
        let v0n = Self::get_dup_checked_vertex(
            &[segment.start.x as f64, segment.start.y as f64],
            new_vertex_map,
        )?;
        let v1n = Self::get_dup_checked_vertex(
            &[segment.end.x as f64, segment.end.y as f64],
            new_vertex_map,
        )?;
        if let Some(v0i) = pb_face.vertices.iter().position(|x| x == &v0n) {
            if let Some(v1i) = pb_face.vertices.iter().position(|x| x == &v1n) {
                /*if pb_face.vertices.contains(&v0n) && pb_face.vertices.contains(&v1n) {
                    println!(
                        "COULD SPLIT v0n:{} v0i:{} v1n:{} v1i:{}",
                        v0n, v0i, v1n, v1i
                    );
                }*/
                let mut a = Vec::<u64>::new();
                let mut b = Vec::<u64>::new();
                if v0i < v1i {

                    for i in (0..=v0i).chain(v1i..pb_face.vertices.len()) {
                        a.push(pb_face.vertices[i]);
                    }
                    for i in v0i..=v1i {
                        b.push(pb_face.vertices[i]);
                    }
                    //println!("v:{:?}", pb_face.vertices);
                    //println!("a:{:?}", a);
                    //println!("b:{:?}", b);

                } else {

                    for i in (0..=v1i).chain(v0i..pb_face.vertices.len()) {
                        a.push(pb_face.vertices[i]);
                    }
                    for i in v1i..=v0i {
                        b.push(pb_face.vertices[i]);
                    }
                    //println!("v:{:?}", pb_face.vertices);
                    //println!("a:{:?}", a);
                    //println!("b:{:?}", b);
                };
                return Ok(Some((PB_Face { vertices: a }, PB_Face { vertices: b })))
            }
        }
        Ok(None)
    }

    /// Retrieves a point from the voronoi input in the order it was presented to
    /// the voronoi builder
    #[inline(always)]
    fn retrieve_point(&self, cell_id: VD::VoronoiCellIndex) -> boostvoronoi::Point<i64> {
        let (index, cat) = self.diagram.get_cell(cell_id).get().source_index_2();
        match cat {
            VD::SourceCategory::SinglePoint => self.vertices[index],
            VD::SourceCategory::SegmentStart => self.segments[index - self.vertices.len()].start,
            VD::SourceCategory::Segment | VD::SourceCategory::SegmentEnd => {
                self.segments[index - self.vertices.len()].end
            }
        }
    }

    /// Retrieves a segment from the voronoi input in the order it was presented to
    /// the voronoi builder
    #[inline(always)]
    fn retrieve_segment(&self, cell_id: VD::VoronoiCellIndex) -> &boostvoronoi::Line<i64> {
        let cell = self.diagram.get_cell(cell_id).get();
        let index = cell.source_index() - self.vertices.len();
        &self.segments[index]
    }

    fn replace_infinite_with_origin(
        &self,
        edge_id: VD::VoronoiEdgeIndex,
    ) -> Result<(cgmath::Point3<f64>, cgmath::Point3<f64>), TBError> {
        let edge = self.diagram.get_edge(edge_id);
        //const cell_type& cell1 = *edge.cell();
        let cell1_id = self.diagram.edge_get_cell(Some(edge_id)).unwrap();
        let cell1 = self.diagram.get_cell(cell1_id).get();
        //const cell_type& cell2 = *edge.twin()->cell();
        let cell2_id = self
            .diagram
            .edge_get_twin(Some(edge_id))
            .and_then(|e| self.diagram.edge_get_cell(Some(e)))
            .unwrap();
        let cell2 = self.diagram.get_cell(cell2_id).get();

        let mut origin = [0_f64, 0.0];
        // Infinite edges could not be created by two segment sites.
        if cell1.contains_point() && cell2.contains_point() {
            let p1 = self.retrieve_point(cell1_id);
            let p2 = self.retrieve_point(cell2_id);
            origin[0] = ((p1.x as f64) + (p2.x as f64)) * 0.5;
            origin[1] = ((p1.y as f64) + (p2.y as f64)) * 0.5;
        } else {
            origin = if cell1.contains_segment() {
                let p = self.retrieve_point(cell2_id);
                [p.x as f64, p.y as f64]
            } else {
                let p = self.retrieve_point(cell1_id);
                [p.x as f64, p.y as f64]
            };
        }

        let vertex0 = edge.get().vertex0();
        let start_point = if vertex0.is_none() {
            cgmath::Point3 {
                x: origin[0],
                y: origin[1],
                z: 0.0,
            }
        } else {
            let vertex0 = self.diagram.vertex_get(vertex0).unwrap().get();
            cgmath::Point3 {
                x: vertex0.x(),
                y: vertex0.y(),
                z: f64::NAN,
            }
        };

        let vertex1 = self.diagram.edge_get_vertex1(Some(edge_id));
        let end_point = if vertex1.is_none() {
            cgmath::Point3 {
                x: origin[0],
                y: origin[1],
                z: 0.0,
            }
        } else {
            let vertex1 = self.diagram.vertex_get(vertex1).unwrap().get();
            cgmath::Point3 {
                x: vertex1.x(),
                y: vertex1.y(),
                z: f64::NAN,
            }
        };

        Ok((start_point, end_point))
    }
}

#[allow(clippy::type_complexity)]
fn parse_input(
    input_pb_model: &PB_Model,
    cmd_arg_max_voronoi_dimension: f64,
) -> Result<
    (
        Vec<boostvoronoi::Point<i64>>,
        Vec<boostvoronoi::Line<i64>>,
        Aabb2<f64>,
        cgmath::Matrix4<f64>,
    ),
    TBError,
> {
    let mut aabb = linestring::cgmath_3d::Aabb3::<f64>::default();
    for v in input_pb_model.vertices.iter() {
        aabb.update_point(&cgmath::Point3::new(v.x as f64, v.y as f64, v.z as f64))
    }

    let (plane, transform, vor_aabb)= centerline::get_transform_relaxed(
        &aabb,
        cmd_arg_max_voronoi_dimension,
        super::EPSILON,
        f64::default_max_ulps(),
    ).map_err(|_|{
        let aabbe_d = aabb.get_high().unwrap() - aabb.get_low().unwrap();
        let aabbe_c = (aabb.get_high().unwrap().to_vec() + aabb.get_low().unwrap().to_vec())/2.0;
        TBError::InputNotPLane(format!(
            "Input data not in one plane and/or plane not intersecting origin: Δ({},{},{}) C({},{},{})",
            aabbe_d.x, aabbe_d.y, aabbe_d.z,aabbe_c.x, aabbe_c.y, aabbe_c.z))
    })?;

    let invers_transform = transform
        .invert()
        .ok_or(TBError::CouldNotCalculateInvertMatrix)?;

    println!("voronoi: data was in plane:{:?} aabb:{:?}", plane, aabb);
    //println!("input Lines:{:?}", input_pb_model.vertices);

    let mut vor_lines = Vec::<boostvoronoi::Line<i64>>::with_capacity(input_pb_model.faces.len());
    let vor_vertices: Vec<boostvoronoi::Point<i64>> = input_pb_model
        .vertices
        .iter()
        .map(|vertex| {
            let p = super::xy_to_2d(&transform.transform_point(cgmath::Point3 {
                x: vertex.x,
                y: vertex.y,
                z: vertex.z,
            }));
            boostvoronoi::Point {
                x: p.x as i64,
                y: p.y as i64,
            }
        })
        .collect();
    let mut used_vertices = yabf::Yabf::with_capacity(vor_vertices.len());

    for face in input_pb_model.faces.iter() {
        match face.vertices.len() {
            3..=usize::MAX => return Err(TBError::ModelContainsFaces("Model can't contain any faces, only edges and points. Use the 2d_outline tool to remove faces".to_string())),
            2 => {
                let v0 = face.vertices[0] as usize;
                let v1 = face.vertices[1] as usize;

                vor_lines.push(boostvoronoi::Line {
                    start: vor_vertices[v0],
                    end: vor_vertices[v1],
                });
                used_vertices.set_bit(v0, true);
                used_vertices.set_bit(v1, true);
            },
            // This does not work, face.len() is never 1
            //1 => points.push(vertices_2d[face.vertices[0] as usize]),
            _ => (),
        }
    }
    // save the unused vertices as points
    let vor_vertices: Vec<boostvoronoi::Point<i64>> = vor_vertices
        .into_iter()
        .enumerate()
        .filter(|x| !used_vertices.bit(x.0))
        .map(|x| x.1)
        .collect();
    drop(used_vertices);

    //println!("lines_2d.len():{:?}", vor_lines.len());
    //println!("vertices_2d.len():{:?}", vor_vertices.len());
    Ok((vor_vertices, vor_lines, vor_aabb, invers_transform))
}

/// Runs boost voronoi over the input,
/// Removes the external edges as we can't handle infinite length edges in blender.
fn voronoi_mesh(
    input_pb_model: &PB_Model,
    cmd_arg_max_voronoi_dimension: f64,
    cmd_discrete_distance: f64,
    cmd_remove_externals: bool,
) -> Result<PB_Model, TBError> {
    let (vor_vertices, vor_lines, vor_aabb2, inverted_transform) =
        parse_input(input_pb_model, cmd_arg_max_voronoi_dimension)?;
    let mut vb = VB::Builder::default();
    vb.with_vertices(vor_vertices.iter())?;
    vb.with_segments(vor_lines.iter())?;
    let vor_diagram = vb.construct()?;
    //println!("diagram:{:?}", vor_diagram);
    //println!("aabb2:{:?}", vor_aabb2);
    let mut diagram_helper = DiagramHelper {
        vertices: vor_vertices,
        segments: vor_lines,
        diagram: vor_diagram,
        aabb: vor_aabb2,
        rejected_edges: None,
        discrete_distance: cmd_discrete_distance,
        remove_externals: cmd_remove_externals,
    };

    if diagram_helper.remove_externals {
        diagram_helper.reject_edges()?;
    }
    build_output(false, input_pb_model, diagram_helper, inverted_transform)
}

fn build_output(
    include_input_geometry: bool,
    input_pb_model: &PB_Model,
    diagram: DiagramHelper,
    inverted_transform: cgmath::Matrix4<f64>,
) -> Result<PB_Model, TBError> {
    // a map of hashtable point to vertex number
    let mut new_vertex_map = ahash::AHashMap::<(u64, u64), usize>::default();

    let mut vertices_2d = {
        // convert this back to network form again
        // todo: use the duplicate checker of DiagramHelper
        let vertices_2d: Vec<PB_Vertex> = diagram
            .vertices
            .iter()
            .enumerate()
            .map(|p| {
                let v0 = cgmath::Point2 {
                    x: p.1.x as f64,
                    y: p.1.y as f64,
                };
                let key = transmute_to_u64(&v0);
                let _ = new_vertex_map.entry(key).or_insert_with(|| p.0);
                let v = inverted_transform.transform_point(super::xy_to_3d(&cgmath::Point2 {
                    x: p.1.x as f64,
                    y: p.1.y as f64,
                }));
                PB_Vertex {
                    x: v.x,
                    y: v.y,
                    z: v.z,
                }
            })
            .collect();
        assert_eq!(vertices_2d.len(), diagram.vertices.len());
        vertices_2d
    }; //else {
       //  Vec::<PB_Vertex>::new()
       //};

    // push the line vertices to the same vec
    let mut faces_2d = if include_input_geometry {
        // todo: use the duplicate checker of DiagramHelper
        let mut faces_2d = Vec::<PB_Face>::with_capacity(diagram.segments.len());
        for l in diagram.segments.iter() {
            let mut face = PB_Face {
                vertices: Vec::<u64>::with_capacity(2),
            };
            let v0 = l.start;
            let v0 = cgmath::Point2 {
                x: v0.x as f64,
                y: v0.y as f64,
            };
            let key = transmute_to_u64(&v0);
            let v0 = inverted_transform.transform_point(super::xy_to_3d(&v0));
            let n = new_vertex_map.entry(key).or_insert_with(|| {
                let rv = vertices_2d.len();
                vertices_2d.push(PB_Vertex {
                    x: v0.x,
                    y: v0.y,
                    z: v0.z,
                });
                rv
            });
            face.vertices.push(*n as u64);

            let v1 = l.end;
            let v1 = cgmath::Point2 {
                x: v1.x as f64,
                y: v1.y as f64,
            };
            let key = transmute_to_u64(&v1);
            let v1 = inverted_transform.transform_point(super::xy_to_3d(&v1));
            let n = new_vertex_map.entry(key).or_insert_with(|| {
                let rv = vertices_2d.len();
                vertices_2d.push(PB_Vertex {
                    x: v1.x,
                    y: v1.y,
                    z: v1.z,
                });
                rv
            });
            face.vertices.push(*n as u64);
            if !face.vertices.is_empty() {
                let first = *face.vertices.first().unwrap();
                if *face.vertices.last().unwrap() != first {
                    face.vertices.push(first);
                    println!(
                        "input geometry face was missing end point {:?}",
                        face.vertices
                    );
                }
            }
            faces_2d.push(face);
        }
        faces_2d
    } else {
        Vec::<PB_Face>::new()
    };

    let mut diagram = diagram;
    if true {
        diagram.convert_cells(
            &mut new_vertex_map,
            &mut vertices_2d,
            &mut faces_2d,
            &inverted_transform,
        )?;
    }
    /*
        println!("input model vertices:{:?}", vertices_2d.len());
        println!("2d vertices");
        for v in diagram
            .diagram
            .vertices()
            .iter()
            .sorted_unstable_by(|v0, v1| v0.get().x().partial_cmp(&v1.get().x()).unwrap())
        {
            let v = v.get();
            println!("#{}, {:.12},{:.12}", v.get_id().0, v.x(), v.y())
        }
        println!("output vertices");
        for v in vertices_2d
            .iter()
            .enumerate()
            .sorted_unstable_by(|v0, v1| v0.1.x.partial_cmp(&v1.1.x).unwrap())
        {
            println!("#{}, {:.12},{:.12},{:.12}", v.0, v.1.x, v.1.y, v.1.z)
        }
        println!("input model faces:{:?}", faces_2d.len());
    */
    let model = PB_Model {
        name: input_pb_model.name.clone(),
        world_orientation: input_pb_model.world_orientation.clone(),
        vertices: vertices_2d, //Vec::<PB_Vertex>::with_capacity(0),
        faces: faces_2d,
    };
    Ok(model)
}

pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!("Centerline mesh got command: \"{}\"", a_command.command);
    if a_command.models.len() > 1 {
        return Err(TBError::InvalidInputData(
            "This operation only supports one model as input".to_string(),
        ));
    }

    let cmd_arg_max_voronoi_dimension = {
        let tmp_value = super::MAX_VORONOI_DIMENSION.to_string();
        let value = options.get("MAX_VORONOI_DIMENSION").unwrap_or(&tmp_value);
        value.parse::<f64>().map_err(|_| {
            TBError::InvalidInputData(format!(
                "Could not parse the MAX_VORONOI_DIMENSION parameter {:?}",
                value
            ))
        })?
    };
    if !(super::MAX_VORONOI_DIMENSION..100_000_000.0).contains(&cmd_arg_max_voronoi_dimension) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of MAX_VORONOI_DIMENSION is [{}..100_000_000[% :({})",
            super::MAX_VORONOI_DIMENSION,
            cmd_arg_max_voronoi_dimension
        )));
    }
    let cmd_arg_discrete_distance = {
        let tmp_value = super::VORONOI_DISCRETE_DISTANCE.to_string();
        let value = options.get("DISTANCE").unwrap_or(&tmp_value);
        value.parse::<f64>().map_err(|_| {
            TBError::InvalidInputData(format!(
                "Could not parse the DISTANCE parameter {:?}",
                value
            ))
        })?
    };
    if !(super::VORONOI_DISCRETE_DISTANCE..5.0).contains(&cmd_arg_discrete_distance) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of DISTANCE is [{}..5.0[% :({})",
            super::VORONOI_DISCRETE_DISTANCE,
            cmd_arg_discrete_distance
        )));
    }
    // used for simplification and discretization distance
    let max_distance = cmd_arg_max_voronoi_dimension * cmd_arg_discrete_distance / 100.0;

    let cmd_arg_remove_externals = {
        let default_value = "false".to_string();
        let value = options.get("REMOVE_EXTERNALS").unwrap_or(&default_value);
        value.parse::<bool>().map_err(|_| {
            TBError::InvalidInputData(format!(
                "Could not parse the REMOVE_EXTERNALS parameter {:?}",
                value
            ))
        })?
    };

    for model in a_command.models.iter() {
        println!("model.name:{:?}, ", model.name);
        println!("model.vertices:{:?}, ", model.vertices.len());
        println!("model.faces:{:?}, ", model.faces.len());
        println!(
            "model.world_orientation:{:?}, ",
            model.world_orientation.as_ref().map_or(0, |_| 16)
        );
        println!("MAX_VORONOI_DIMENSION:{:?}", cmd_arg_max_voronoi_dimension);
        println!("REMOVE_EXTERNALS:{:?}", cmd_arg_remove_externals);
        println!("VORONOI_DISCRETE_DISTANCE:{:?}%", cmd_arg_discrete_distance);
        println!("max_distance:{:?}", max_distance);
        println!();
    }

    if !a_command.models.is_empty() {
        let input_model = &a_command.models[0];
        let output_model = voronoi_mesh(
            &input_model,
            cmd_arg_max_voronoi_dimension,
            cmd_arg_discrete_distance,
            cmd_arg_remove_externals,
        )?;
        let mut reply = PB_Reply {
            options: vec![PB_KeyValuePair {
                key: "ONLY_EDGES".to_string(),
                value: "False".to_string(),
            }],
            models: Vec::<PB_Model>::new(),
        };

        reply.models.push(output_model);
        Ok(reply)
    } else {
        Err(TBError::InvalidInputData(
            "Model did not contain any data".to_string(),
        ))
    }
}
