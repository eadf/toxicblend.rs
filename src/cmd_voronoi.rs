use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex as PB_Vertex;
use crate::{GrowingVob, TBError};
use boostvoronoi::builder as VB;
use boostvoronoi::diagram as VD;
use boostvoronoi::geometry;
use boostvoronoi::visual_utils as VU;
use cgmath::{ulps_eq, EuclideanSpace, SquareMatrix, Transform, UlpsEq};
use itertools::Itertools;
use linestring::linestring_2d::Aabb2;
use std::collections::HashMap;

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

/// Helper structs that build PB buffer from Diagram
struct DiagramHelper {
    diagram: VD::Diagram<i64, f64>,
    vertices: Vec<geometry::Point<i64>>,
    segments: Vec<geometry::Line<i64>>,
    aabb: Aabb2<f64>,
    rejected_edges: Option<vob::Vob<u32>>,
    discrete_distance: f64,
    remove_secondary_edges: bool,
}

impl DiagramHelper {
    /// converts to a private, comparable and hash-able format
    /// only use this for floats that are f64::is_finite()
    #[inline(always)]
    fn transmute_to_u64(x: f64, y: f64) -> (u64, u64) {
        (x.to_bits(), y.to_bits())
    }

    /// transform the voronoi Point into a PB point. Perform duplication checks
    #[inline(always)]
    fn place_new_pb_vertex_dup_check(
        vertex: &[f64; 2],
        new_vertex_map: &mut ahash::AHashMap<(u64, u64), usize>,
        pb_vertices: &mut Vec<PB_Vertex>,
        inverted_transform: &cgmath::Matrix4<f64>,
    ) -> usize {
        let key = Self::transmute_to_u64(vertex[0], vertex[1]);
        let v1 = inverted_transform.transform_point(cgmath::Point3 {
            x: vertex[0],
            y: vertex[1],
            z: 0.0,
        });
        *new_vertex_map.entry(key).or_insert_with(|| {
            let n = pb_vertices.len();
            pb_vertices.push(PB_Vertex {
                x: v1.x,
                y: v1.y,
                z: v1.z,
            });
            n
        })
    }

    /// transform the voronoi Point into a PB point. Perform *no* duplication checks
    #[inline(always)]
    fn place_new_pb_vertex_unchecked(
        vertex: &[f64; 2],
        pb_vertices: &mut Vec<PB_Vertex>,
        inverted_transform: &cgmath::Matrix4<f64>,
    ) -> usize {
        let v = inverted_transform.transform_point(cgmath::Point3 {
            x: vertex[0],
            y: vertex[1],
            z: 0.0,
        });
        let n = pb_vertices.len();
        pb_vertices.push(PB_Vertex {
            x: v.x,
            y: v.y,
            z: v.z,
        });
        n
    }

    /// Convert voronoi edges into PB_Model data
    fn convert_edges(
        &mut self,
        new_vertex_map: &mut ahash::AHashMap<(u64, u64), usize>,
        pb_vertices: &mut Vec<PB_Vertex>,
        pb_faces: &mut Vec<PB_Face>,
        inverted_transform: cgmath::Matrix4<f64>,
    ) -> Result<(), TBError> {
        let max_dist = self.aabb.get_high().unwrap() - self.aabb.get_low().unwrap();
        let max_dist = max_dist.x.max(max_dist.y);
        let max_dist = self.discrete_distance * max_dist;
        //println!("max distance = {}",max_dist);
        let dummy_affine = boostvoronoi::visual_utils::SimpleAffine::default();

        let mut already_drawn: vob::Vob<u32> = vob::Vob::<u32>::fill(self.diagram.edges().len());

        let rejected_edges = self
            .rejected_edges
            .take()
            .unwrap_or_else(|| vob::Vob::<u32>::fill(self.diagram.edges().len()));

        for it in self.diagram.edges().iter().enumerate() {
            let edge_id = VD::EdgeIndex(it.0);
            let edge = it.1.get();
            if (self.remove_secondary_edges && edge.is_secondary())
                || already_drawn.get_f(edge_id.0)
                || rejected_edges.get_f(edge_id.0)
            {
                // already done this, or rather - it's twin
                continue;
            }

            // no point in setting current edge as drawn, the edge id will not repeat
            // set twin as drawn
            let _ = already_drawn.set(self.diagram.edge_get_twin(edge_id)?.0, true);

            // the coordinates in samples must be 'screen' coordinates, i.e. affine transformed
            let mut samples = Vec::<[f64; 2]>::new();
            if !self.diagram.edge_is_finite(edge_id)? {
                self.clip_infinite_edge(edge_id, &mut samples)?;
            } else {
                // edge is finite -> vertexX.unwrap() is safe
                let vertex0 = self.diagram.edge_get_vertex0(edge_id)?.unwrap();
                let vertex0 = self.diagram.vertex_get(vertex0)?.get();

                samples.push([vertex0.x(), vertex0.y()]);
                let vertex1 = self.diagram.edge_get_vertex1(edge_id)?.unwrap();
                let vertex1 = self.diagram.vertex_get(vertex1)?.get();

                samples.push([vertex1.x(), vertex1.y()]);
                if edge.is_curved() {
                    self.sample_curved_edge(
                        max_dist,
                        &dummy_affine,
                        VD::EdgeIndex(it.0),
                        &mut samples,
                    )?;
                }
            }

            if samples.len() > 1 {
                let len = samples.len();
                let mut processed_samples = Vec::<usize>::with_capacity(samples.len());
                processed_samples.push(
                    samples
                        .first()
                        .map(|v| {
                            Self::place_new_pb_vertex_dup_check(
                                v,
                                new_vertex_map,
                                pb_vertices,
                                &inverted_transform,
                            )
                        })
                        .unwrap(),
                );

                for v in samples.iter().skip(1).take(len - 2).map(|v| {
                    Self::place_new_pb_vertex_unchecked(v, pb_vertices, &inverted_transform)
                }) {
                    processed_samples.push(v);
                }
                processed_samples.push(
                    samples
                        .last()
                        .map(|v| {
                            Self::place_new_pb_vertex_dup_check(
                                v,
                                new_vertex_map,
                                pb_vertices,
                                &inverted_transform,
                            )
                        })
                        .unwrap(),
                );

                for (v0, v1) in processed_samples.into_iter().tuple_windows::<(_, _)>() {
                    pb_faces.push(PB_Face {
                        vertices: vec![v0 as u64, v1 as u64],
                    });
                }
            }
        }
        self.rejected_edges = Some(rejected_edges);
        Ok(())
    }

    /// Important: sampled_edge should contain both edge endpoints initially.
    /// sampled_edge should be 'screen' coordinates, i.e. affine transformed from voronoi output
    fn sample_curved_edge(
        &self,
        max_dist: f64,
        dummy_affine: &VU::SimpleAffine<i64, f64>,
        edge_id: VD::EdgeIndex,
        sampled_edge: &mut Vec<[f64; 2]>,
    ) -> Result<(), TBError> {
        let cell_id = self.diagram.edge_get_cell(edge_id)?;
        let cell = self.diagram.get_cell(cell_id)?.get();
        let twin_id = self.diagram.edge_get_twin(edge_id)?;
        let twin_cell_id = self.diagram.edge_get_cell(twin_id)?;

        let point = if cell.contains_point() {
            self.retrieve_point(cell_id)?
        } else {
            self.retrieve_point(twin_cell_id)?
        };
        let segment = if cell.contains_point() {
            self.retrieve_segment(twin_cell_id)?
        } else {
            self.retrieve_segment(cell_id)?
        };
        VU::VoronoiVisualUtils::<i64, f64>::discretize(
            &point,
            segment,
            max_dist,
            dummy_affine,
            sampled_edge,
        );
        Ok(())
    }

    /// Retrieves a point from the voronoi input in the order it was presented to
    /// the voronoi builder
    #[inline(always)]
    fn retrieve_point(&self, cell_id: VD::CellIndex) -> Result<geometry::Point<i64>, TBError> {
        let (index, cat) = self.diagram.get_cell(cell_id)?.get().source_index_2();
        Ok(match cat {
            VD::SourceCategory::SinglePoint => self.vertices[index],
            VD::SourceCategory::SegmentStart => self.segments[index - self.vertices.len()].start,
            VD::SourceCategory::Segment | VD::SourceCategory::SegmentEnd => {
                self.segments[index - self.vertices.len()].end
            }
        })
    }

    /// Retrieves a segment from the voronoi input in the order it was presented to
    /// the voronoi builder
    #[inline(always)]
    fn retrieve_segment(&self, cell_id: VD::CellIndex) -> Result<&geometry::Line<i64>, TBError> {
        let cell = self.diagram.get_cell(cell_id)?.get();
        let index = cell.source_index() - self.vertices.len();
        Ok(&self.segments[index])
    }

    fn clip_infinite_edge(
        &self,
        edge_id: VD::EdgeIndex,
        clipped_edge: &mut Vec<[f64; 2]>,
    ) -> Result<(), TBError> {
        let edge = self.diagram.get_edge(edge_id)?;
        //const cell_type& cell1 = *edge.cell();
        let cell1_id = self.diagram.edge_get_cell(edge_id)?;
        let cell1 = self.diagram.get_cell(cell1_id)?.get();
        //const cell_type& cell2 = *edge.twin()->cell();
        let cell2_id = self
            .diagram
            .edge_get_cell(self.diagram.edge_get_twin(edge_id)?)?;
        let cell2 = self.diagram.get_cell(cell2_id)?.get();

        let mut origin = [0_f64, 0.0];
        let mut direction = [0_f64, 0.0];
        // Infinite edges could not be created by two segment sites.
        if cell1.contains_point() && cell2.contains_point() {
            let p1 = self.retrieve_point(cell1_id)?;
            let p2 = self.retrieve_point(cell2_id)?;
            origin[0] = ((p1.x as f64) + (p2.x as f64)) * 0.5;
            origin[1] = ((p1.y as f64) + (p2.y as f64)) * 0.5;
            direction[0] = (p1.y as f64) - (p2.y as f64);
            direction[1] = (p2.x as f64) - (p1.x as f64);
        } else {
            origin = if cell1.contains_segment() {
                let p = self.retrieve_point(cell2_id)?;
                [p.x as f64, p.y as f64]
            } else {
                let p = self.retrieve_point(cell1_id)?;
                [p.x as f64, p.y as f64]
            };
            let segment = if cell1.contains_segment() {
                self.retrieve_segment(cell1_id)?
            } else {
                self.retrieve_segment(cell2_id)?
            };
            let dx = segment.end.x - segment.start.x;
            let dy = segment.end.y - segment.start.y;
            if (ulps_eq!((segment.start.x as f64), origin[0])
                && ulps_eq!((segment.start.y) as f64, origin[1]))
                ^ cell1.contains_point()
            {
                direction[0] = dy as f64;
                direction[1] = -dx as f64;
            } else {
                direction[0] = -dy as f64;
                direction[1] = dx as f64;
            }
        }

        let side = self.aabb.get_high().unwrap() - self.aabb.get_low().unwrap();
        let side = side.x.max(side.y);
        let koef = side / (direction[0].abs().max(direction[1].abs()));

        let vertex0 = edge.get().vertex0();
        if let Some(vertex0) = vertex0 {
            let vertex0 = self.diagram.vertex_get(vertex0)?.get();
            clipped_edge.push([vertex0.x(), vertex0.y()]);
        } else {
            clipped_edge.push([
                origin[0] - direction[0] * koef,
                origin[1] - direction[1] * koef,
            ]);
        }
        let vertex1 = self.diagram.edge_get_vertex1(edge_id)?;
        if let Some(vertex1) = vertex1 {
            let vertex1 = self.diagram.vertex_get(vertex1)?.get();
            clipped_edge.push([vertex1.x(), vertex1.y()]);
        } else {
            clipped_edge.push([
                origin[0] + direction[0] * koef,
                origin[1] + direction[1] * koef,
            ]);
        }
        Ok(())
    }
}

#[allow(clippy::type_complexity)]
fn parse_input(
    input_pb_model: &PB_Model,
    cmd_arg_max_voronoi_dimension: f64,
) -> Result<
    (
        Vec<geometry::Point<i64>>,
        Vec<geometry::Line<i64>>,
        Aabb2<f64>,
        cgmath::Matrix4<f64>,
    ),
    TBError,
> {
    let mut aabb = linestring::linestring_3d::Aabb3::<f64>::default();
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

    let mut vor_lines = Vec::<geometry::Line<i64>>::with_capacity(input_pb_model.faces.len());
    let vor_vertices: Vec<geometry::Point<i64>> = input_pb_model
        .vertices
        .iter()
        .map(|vertex| {
            let p = super::xy_to_2d(&transform.transform_point(cgmath::Point3 {
                x: vertex.x,
                y: vertex.y,
                z: vertex.z,
            }));
            geometry::Point {
                x: p.x as i64,
                y: p.y as i64,
            }
        })
        .collect();
    let mut used_vertices = vob::Vob::fill(vor_vertices.len());

    for face in input_pb_model.faces.iter() {
        match face.vertices.len() {
            3..=usize::MAX => return Err(TBError::ModelContainsFaces("Model can't contain any faces, only edges and points. Use the 2d_outline tool to remove faces".to_string())),
            2 => {
                let v0 = face.vertices[0] as usize;
                let v1 = face.vertices[1] as usize;

                vor_lines.push(geometry::Line {
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
    let vor_vertices: Vec<geometry::Point<i64>> = vor_vertices
        .into_iter()
        .enumerate()
        .filter(|x| !used_vertices.get_f(x.0))
        .map(|x| x.1)
        .collect();
    drop(used_vertices);

    println!("lines_2d.len():{:?}", vor_lines.len());
    println!("vertices_2d.len():{:?}", vor_vertices.len());
    Ok((vor_vertices, vor_lines, vor_aabb, invers_transform))
}

/// Runs boost voronoi over the input,
/// Removes the external edges as we can't handle infinite length edges in blender.
fn voronoi(
    input_pb_model: &PB_Model,
    cmd_arg_max_voronoi_dimension: f64,
    cmd_arg_discrete_distance: f64,
    cmd_remove_externals: bool,
    cmd_remove_secondary_edges: bool,
) -> Result<PB_Model, TBError> {
    let (vor_vertices, vor_lines, vor_aabb2, inverted_transform) =
        parse_input(input_pb_model, cmd_arg_max_voronoi_dimension)?;
    let mut vb = VB::Builder::default();
    vb.with_vertices(vor_vertices.iter())?;
    vb.with_segments(vor_lines.iter())?;
    let vor_diagram = vb.build()?;
    //println!("diagram:{:?}", vor_diagram);
    println!("aabb2:{:?}", vor_aabb2);
    let mut diagram_helper = DiagramHelper {
        vertices: vor_vertices,
        segments: vor_lines,
        diagram: vor_diagram,
        aabb: vor_aabb2,
        rejected_edges: None,
        discrete_distance: cmd_arg_discrete_distance,
        remove_secondary_edges: cmd_remove_secondary_edges,
    };
    if cmd_remove_externals {
        diagram_helper.rejected_edges = Some(super::voronoi_utils::reject_external_edges(
            &diagram_helper.diagram,
        )?);
    }
    build_output(input_pb_model, diagram_helper, inverted_transform)
}

fn build_output(
    input_pb_model: &PB_Model,
    diagram: DiagramHelper,
    inverted_transform: cgmath::Matrix4<f64>,
) -> Result<PB_Model, TBError> {
    // a map of hashtable point to vertex number
    let mut new_vertex_map = ahash::AHashMap::<(u64, u64), usize>::default();

    let mut vertices_2d = {
        // had to encase this in a block b/o the borrow checker.

        // convert this back to network form again
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
        vertices_2d
    };
    //let last_point_plus_one = vertices_2d.len();

    // push the line vertices to the same vec
    let mut faces_2d = {
        // had to encase this in a block b/o the borrow checker.
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
            faces_2d.push(face);
        }
        faces_2d
    };

    let mut diagram = diagram;
    diagram.convert_edges(
        &mut new_vertex_map,
        &mut vertices_2d,
        &mut faces_2d,
        inverted_transform,
    )?;

    println!("input model vertices:{:?}", vertices_2d.len());
    println!("input model faces:{:?}", faces_2d.len());

    let model = PB_Model {
        name: input_pb_model.name.clone(),
        world_orientation: input_pb_model.world_orientation.clone(),
        vertices: vertices_2d, //Vec::<PB_Vertex>::with_capacity(0),
        faces: faces_2d,
    };
    Ok(model)
}

pub fn command(
    a_command: PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!(
        r#"                                              __
    ___  __ ____ _______  ____   ____   ____ |__|
    \  \/ // __ \\_  __ \/ __ \ /    \ / __ \|  |
     \   /(  \_\ )|  | \/  \_\ )   |  \  \_\ )  |
      \_/  \____/ |__|   \____/|___|  /\____/|__|
                                    \/           "#
    );
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
    let cmd_arg_discrete_distance = {
        let tmp_value = super::DEFAULT_VORONOI_DISCRETE_DISTANCE.to_string();
        let value = options
            .get("VORONOI_DISCRETE_DISTANCE")
            .unwrap_or(&tmp_value);
        value.parse::<f64>().map_err(|_| {
            TBError::InvalidInputData(format!(
                "Could not parse the VORONOI_DISCRETE_DISTANCE parameter {:?}",
                value
            ))
        })?
    };
    if !(super::DEFAULT_VORONOI_DISCRETE_DISTANCE..1.0).contains(&cmd_arg_discrete_distance) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of VORONOI_DISCRETE_DISTANCE is [{}..1.0[% :({})",
            super::DEFAULT_VORONOI_DISCRETE_DISTANCE,
            cmd_arg_discrete_distance
        )));
    }

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

    let cmd_arg_remove_secondary_edges = {
        let default_value = "false".to_string();
        let value = options
            .get("REMOVE_SECONDARY_EDGES")
            .unwrap_or(&default_value);
        value.parse::<bool>().map_err(|_| {
            TBError::InvalidInputData(format!(
                "Could not parse the REMOVE_SECONDARY_EDGES parameter {:?}",
                value
            ))
        })?
    };

    // used for simplification and discretization distance
    let max_distance = cmd_arg_max_voronoi_dimension * cmd_arg_discrete_distance / 100.0;

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

    // percentage to multiplication conversion
    let cmd_arg_discrete_distance = cmd_arg_discrete_distance / 100.0;

    if !a_command.models.is_empty() {
        let input_model = &a_command.models[0];
        let output_model = voronoi(
            input_model,
            cmd_arg_max_voronoi_dimension,
            cmd_arg_discrete_distance,
            cmd_arg_remove_externals,
            cmd_arg_remove_secondary_edges,
        )?;
        let mut reply = PB_Reply {
            options: vec![PB_KeyValuePair {
                key: "ONLY_EDGES".to_string(),
                value: "True".to_string(),
            }],
            models: Vec::with_capacity(1),
            models32: Vec::with_capacity(0),
        };

        reply.models.push(output_model);
        Ok(reply)
    } else {
        Err(TBError::InvalidInputData(
            "Model did not contain any data".to_string(),
        ))
    }
}
