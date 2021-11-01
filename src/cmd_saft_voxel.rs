use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face32 as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Matrix4x432 as PB_Matrix4x432;
use crate::toxicblend_pb::Model32 as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex32 as PB_Vertex;
use crate::TBError;
use glam_saft::Vec3;
use saft::BoundingBox;
use std::collections::HashMap;
use std::time;

/// unpack the input PB_Model
#[allow(clippy::type_complexity)]
pub fn parse_input_pb_model(obj: &PB_Model) -> Result<(Vec<(u32, u32)>, BoundingBox), TBError> {
    if obj.vertices.len() >= u32::MAX as usize {
        return Err(TBError::Overflow(format!(
            "Input data contains too many vertices. {}",
            obj.vertices.len()
        )));
    }
    let mut aabb = BoundingBox::default();

    for v in obj.vertices.iter() {
        aabb.extend(Vec3::new(v.x, v.y, v.z));
    }

    let mut edges = Vec::<(u32, u32)>::with_capacity(obj.vertices.len() + 100);

    for face in obj.faces.iter() {
        if face.vertices.len() > 2 {
            return Err(TBError::ModelContainsFaces("Model can't contain any faces, only edges. Use the 2d_outline tool to remove faces".to_string()));
        }
        if face.vertices.len() < 2 {
            return Err(TBError::InvalidInputData(
                "Edge containing none or only one vertex".to_string(),
            ));
        };
        edges.push((
            *face.vertices.first().unwrap(),
            *face.vertices.last().unwrap(),
        ));
    }
    Ok((edges, aabb))
}

/// initialize the sdf capsules and generate the mesh
fn build_voxel(
    radius_multiplier: f32,
    divisions: f32,
    vertices: &[PB_Vertex],
    edges: Vec<(u32, u32)>,
    aabb: BoundingBox,
) -> Result<
    (
        f32, // <- voxel_size
        saft::TriangleMesh,
    ),
    TBError,
> {
    let dimensions = aabb.max - aabb.min;
    let max_dimension = dimensions.x.max(dimensions.y).max(dimensions.z);

    let radius = max_dimension * radius_multiplier; // unscaled
    let thickness = radius * 2.0; // unscaled
    let scale = (divisions / max_dimension) as f32;

    println!(
        "Voxelizing using tube thickness. {} = {}*{}*{}",
        thickness, max_dimension, radius_multiplier, scale
    );

    println!(
        "Voxelizing using divisions = {}, max dimension = {}, scale factor={} (max_dimension*scale={})",
        divisions,
        max_dimension,
        scale,
        (max_dimension as f32) * scale
    );
    println!();

    println!("aabb.high:{:?}", aabb.max);
    println!("aabb.low:{:?}", aabb.min);
    println!("delta:{:?}", aabb.max - aabb.min);

    let mean_resolution = max_dimension * scale;
    println!("mean_resolution:{:?}", mean_resolution);

    let mesh_options = saft::MeshOptions {
        mean_resolution,
        max_resolution: mean_resolution,
        min_resolution: 8.0,
    };

    let vertices: Vec<_> = vertices
        .iter()
        .map(|v| Vec3::new(v.x, v.y, v.z) * scale)
        .collect();

    let now = time::Instant::now();
    let mut graph = saft::Graph::default();

    let radius = radius * scale; // now scaled
    let capsules: Vec<_> = edges
        .into_iter()
        .map(|(e0, e1)| graph.capsule([vertices[e0 as usize], vertices[e1 as usize]], radius))
        .collect();

    let root = graph.op_union_multi(capsules);
    let mesh = saft::mesh_from_sdf(&graph, root, mesh_options)?;

    println!("mesh_from_sdf() duration: {:?}", now.elapsed());

    Ok((1.0 / scale, mesh))
}

/// Build the return model, totally ignore colors
pub(crate) fn build_output_bp_model(
    pb_model_name: String,
    pb_world: Option<PB_Matrix4x432>,
    voxel_size: f32,
    mesh: saft::TriangleMesh,
) -> Result<PB_Model, TBError> {
    let pb_vertices: Vec<PB_Vertex> = mesh
        .positions
        .iter()
        .map(|v| PB_Vertex {
            x: (voxel_size * v[0]),
            y: (voxel_size * v[1]),
            z: (voxel_size * v[2]),
        })
        .collect();

    Ok(PB_Model {
        name: pb_model_name,
        world_orientation: pb_world,
        vertices: pb_vertices,
        faces: vec![PB_Face {
            vertices: mesh.indices,
        }],
    })
}

pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    let now = time::Instant::now();
    println!(
        r#"  _________       _____  __ ____   ____                 .__
 /   _____/____ _/ ____\/  |\   \ /   /______  ___ ____ |  |
 \_____  \\__  \\   __\\   __\   Y   /  _ \  \/  // __ \|  |
 /        \/ __ \|  |   |  |  \     (  <_> >    <\  ___/|  |__
/_______  (____  /__|   |__|   \___/ \____/__/\_ \\___  >____/
        \/     \/                               \/    \/"#
    );

    if a_command.models32.len() > 1 {
        return Err(TBError::InvalidInputData(format!(
            "This operation only supports one model as input:{}",
            a_command.models32.len()
        )));
    }
    if a_command.models32.is_empty() {
        return Err(TBError::InvalidInputData(
            "Model did not contain any data (using model32)".to_string(),
        ));
    }
    let cmd_arg_radius_multiplier = options
        .get("RADIUS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the RADIUS parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| {
            TBError::InvalidInputData("Could not parse the RADIUS parameter".to_string())
        })?
        / 100.0;

    let cmd_arg_divisions = options
        .get("DIVISIONS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the DIVISIONS parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| {
            TBError::InvalidInputData("Could not parse the DIVISIONS parameter".to_string())
        })?;
    if !(9.9..400.1).contains(&cmd_arg_divisions) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of DIVISIONS is [{}..{}[% :({})",
            10, 400, cmd_arg_divisions
        )));
    }

    for model in a_command.models32.iter() {
        println!("model.name:{:?}, ", model.name);
        println!("model.vertices:{:?}, ", model.vertices.len());
        println!("model.faces:{:?}, ", model.faces.len());
        println!(
            "model.world_orientation:{:?}, ",
            model.world_orientation.as_ref().map_or(0, |_| 16)
        );
        println!("Tube radius:{:?} multiplier ", cmd_arg_radius_multiplier);
        println!("Voxel divisions:{:?} ", cmd_arg_divisions);
        println!();
    }

    let (edges, aabb) = parse_input_pb_model(&a_command.models32[0])?;
    let (voxel_size, mesh) = build_voxel(
        cmd_arg_radius_multiplier,
        cmd_arg_divisions,
        &a_command.models32[0].vertices,
        edges,
        aabb,
    )?;
    let packed_faces_model = build_output_bp_model(
        a_command.command.clone(),
        a_command.models32[0].world_orientation.clone(),
        voxel_size,
        mesh,
    )?;
    println!(
        "Total number of vertices: {}",
        packed_faces_model.vertices.len()
    );

    println!(
        "Total number of triangles: {}",
        packed_faces_model
            .faces
            .iter()
            .map(|x| x.vertices.len())
            .sum::<usize>()
            / 3
    );

    let reply = PB_Reply {
        options: vec![
            PB_KeyValuePair {
                key: "ONLY_EDGES".to_string(),
                value: "False".to_string(),
            },
            PB_KeyValuePair {
                key: "PACKED_FACES".to_string(),
                value: "True".to_string(),
            },
        ],
        models: Vec::with_capacity(0),
        models32: vec![packed_faces_model],
    };
    println!("total duration: {:?}", now.elapsed());
    Ok(reply)
}
