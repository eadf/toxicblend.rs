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
use linestring::cgmath_2d::Aabb2;
//use linestring::cgmath_2d::VoronoiParabolicArc;
//
use std::collections; // ::{HashMap, VecDeque};


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

enum Edge {
    /// point-to-point line
    LineEdge {edge_id:usize, line_edge_id:usize, is_reversed:bool},
    /// An, erhm, curved edge
    CurvedEdge{edge_id:usize, curved_edge_id:usize, is_reversed:bool},
}

struct Cell {
    real_cell_id:usize,
    edges:Vec<Edge>,
    is_point_cell:bool,
}

/// A different representation of boostvoronoi::diagram::Diagram
/// The cells are split if built from segments. The splitting segment is added an an extra edge.
struct SplitDiagram {
    diagram: VD::VoronoiDiagram<i64, f64>,
    vertices: Vec<PB_Vertex>,
    edges: Vec<Edge>,
    line_edges: Vec<(usize,usize)>,
    curved_edges: Vec<Vec<usize>>,
    cells: Vec<Cell>,
    rejected_edges : yabf::Yabf,
}

impl SplitDiagram {
    fn new(diagram: VD::VoronoiDiagram<i64, f64>) -> Result<Self,TBError> {
        let rejected_edges= super::voronoi_utils::reject_external_edges(&diagram)?;
        Ok(Self{
            diagram,
            vertices:Vec::<PB_Vertex>::new(),
            edges:Vec::<Edge>::new(),
            line_edges: Vec::<(usize,usize)>::new(),
            curved_edges:Vec::<Vec<usize>>::new(),
            cells:Vec::<Cell>::new(),
            rejected_edges,
         })
    }
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
    let mut vb = VB::Builder::<i64,f64>::default();
    vb.with_vertices(vor_vertices.iter())?;
    vb.with_segments(vor_lines.iter())?;
    let split = SplitDiagram::new(vb.construct()?)?;

    let rv = PB_Model {
        name: input_pb_model.name.clone(),
        world_orientation: input_pb_model.world_orientation.clone(),
        vertices: Vec::<PB_Vertex>::with_capacity(0),
        faces: Vec::<PB_Face>::with_capacity(0),
    };
    Ok(rv)
}

pub fn command(
    a_command: &PB_Command,
    options: collections::HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!("Voronoi mesh 2 got command: \"{}\"", a_command.command);
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
        let default_value = "true".to_string();
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