use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face32 as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Matrix4x432 as PB_Matrix4x432;
use crate::toxicblend_pb::Model32 as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex32 as PB_Vertex;
use glam_saft::Vec3;
use saft::BoundingBox;
use std::collections::HashMap;
use std::time;

#[derive(Default)]
struct GyroidParameters {
    cmd_arg_divisions: f32,
    cmd_arg_s_param: f32,
    cmd_arg_t_param: f32,
    cmd_arg_b_param: f32,
    cmd_arg_x_param: f32,
    cmd_arg_y_param: f32,
    cmd_arg_z_param: f32,
}

/// initialize the example sdf gyroid and generate the mesh
fn build_gyroid_voxel(
    mut params: GyroidParameters,
) -> Result<
    (
        f32, // <- voxel_size
        saft::TriangleMesh,
    ),
    TBError,
> {
    // set scale so that extent.min*scale -> -pi, extent.max*scale -> pi
    // when cmd_arg_s_param is 1.0
    let scale = params.cmd_arg_s_param * f32::PI() / (params.cmd_arg_divisions.abs() / 2.0);

    println!("Voxelizing using resolution. {}", params.division);

    let mean_resolution = params.division * scale;
    println!("mean_resolution:{:?}", mean_resolution);

    let mesh_options = saft::MeshOptions {
        mean_resolution,
        max_resolution: mean_resolution,
        min_resolution: 8.0,
    };

    let now = time::Instant::now();
    let mut graph = saft::Graph::default();

    // How to create "custom" sdf nodes?
    unimplemented!();
    let root = graph.op_union_multi(????);
    let mesh = saft::mesh_from_sdf(&graph, root, mesh_options)?;

    println!("mesh_from_sdf() duration: {:?}", now.elapsed());

    // scale the voxel so that the result is 3 'units' wide or so.
    let voxel_size = 3.0 / params.cmd_arg_divisions;
    Ok((voxel_size, mesh))
}

#[allow(clippy::field_reassign_with_default)]
pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!(
        r#".▄▄ ·  ▄▄▄· ·▄▄▄▄▄▄▄▄  .▄▄ · ·▄▄▄▄  ·▄▄▄
▐█ ▀. ▐█ ▀█ ▐▄▄ •██    ▐█ ▀. ██· ██ ▐▄▄
▄▀▀▀█▄▄█▀▀█ █  ▪ ▐█.▪  ▄▀▀▀█▄▐█▪ ▐█▌█  ▪
▐█▄▪▐█▐█▪ ▐▌██ . ▐█▌·  ▐█▄▪▐███. ██ ██ .
 ▀▀▀▀  ▀  ▀ ▀▀▀  ▀▀▀    ▀▀▀▀ ▀▀▀▀▀• ▀▀▀ "#
    );

    if !a_command.models32.is_empty() {
        return Err(TBError::InvalidInputData(format!(
            "This operation does not need any models as input:{}",
            a_command.models32.len()
        )));
    }
    let mut params = GyroidParameters::default();
    params.cmd_arg_t_param = options
        .get("T")
        .ok_or_else(|| TBError::InvalidInputData("Missing the T parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the T parameter".to_string()))?;
    params.cmd_arg_b_param = options
        .get("B")
        .ok_or_else(|| TBError::InvalidInputData("Missing the B parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the B parameter".to_string()))?;
    params.cmd_arg_s_param = options
        .get("S")
        .ok_or_else(|| TBError::InvalidInputData("Missing the S parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the S parameter".to_string()))?;
    params.cmd_arg_x_param = options
        .get("X")
        .ok_or_else(|| TBError::InvalidInputData("Missing the X parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the Y parameter".to_string()))?;
    params.cmd_arg_y_param = options
        .get("Y")
        .ok_or_else(|| TBError::InvalidInputData("Missing the Y parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the Y parameter".to_string()))?;
    params.cmd_arg_z_param = options
        .get("Z")
        .ok_or_else(|| TBError::InvalidInputData("Missing the Z parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the Z parameter".to_string()))?;
    params.cmd_arg_divisions = options
        .get("DIVISIONS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the DIVISIONS parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| {
            TBError::InvalidInputData("Could not parse the DIVISIONS parameter".to_string())
        })?;
    let _cmd_arg_plug_ends = options
        .get("PLUG_ENDS")
        .cloned()
        .unwrap_or_else(|| "false".to_string())
        //.ok_or_else(|| TBError::InvalidInputData("Missing the PLUG_ENDS parameter".to_string()))?;
        .to_lowercase();
    let _cmd_arg_plug_ends = _cmd_arg_plug_ends.parse::<bool>().map_err(|_| {
        TBError::InvalidInputData(format!(
            "Could not parse the PLUG_ENDS parameter: '{}'",
            _cmd_arg_plug_ends
        ))
    })?;
    if !(9.9..400.1).contains(&params.cmd_arg_divisions) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of DIVISIONS is [{}..{}[% :({})",
            10, 400, params.cmd_arg_divisions
        )));
    }

    println!("Voxel divisions:{:?} ", params.cmd_arg_divisions);
    println!("s parameter:{:?} ", params.cmd_arg_s_param);
    println!("t parameter:{:?} ", params.cmd_arg_t_param);
    println!("b parameter:{:?} ", params.cmd_arg_b_param);
    println!("x parameter:{:?} ", params.cmd_arg_x_param);
    println!("y parameter:{:?} ", params.cmd_arg_y_param);
    println!("z parameter:{:?} ", params.cmd_arg_z_param);
    //println!("plug ends:{:?} ", params.cmd_arg_plug_ends);
    println!();

    let (voxel_size, mesh) = build_gyroid_voxel(
        params.cmd_arg_divisions,
    )?;
    let packed_faces_model = crate::cmd_saft_voxel::build_output_bp_model(
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
    Ok(reply)
}
