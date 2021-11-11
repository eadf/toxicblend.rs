use crate::{TBError, cmd_bb_voxel};
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Reply as PB_Reply;
use building_blocks::core::prelude::PointN;
use building_blocks::core::prelude::*;
use building_blocks::mesh::*;
use building_blocks::prelude::Sd16;
use building_blocks::storage::prelude::*;
use cgmath::num_traits::FloatConst;
use std::collections::HashMap;
use std::time;

type Point3f = PointN<[f32; 3]>;

const UNPADDED_CHUNK_SIDE: u32 = 16_u32;

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

#[allow(clippy::type_complexity)]
#[allow(clippy::too_many_arguments)]
/// Build the gyroid voxel data from the input
// todo: should i rebuild the return value to just Result<Vec<PosNormMesh, Extent3i>,TBError>?
// todo: the current return value does not live very long, a re-shuffle would just take time.
fn build_gyroid_voxel(
    params: GyroidParameters,
) -> Result<
    (
        f32, // <-voxel_size
        Vec<PosNormMesh>,
    ),
    TBError,
> {
    println!(
        "Voxelizing gyroid using divisions={}, s={}, t={}, b={}",
        params.cmd_arg_divisions,
        params.cmd_arg_s_param,
        params.cmd_arg_t_param,
        params.cmd_arg_b_param
    );
    println!();
    // set scale so that extent.min*scale -> -pi, extent.max*scale -> pi
    // when cmd_arg_s_param is 1.0
    let scale = params.cmd_arg_s_param * f32::PI() / (params.cmd_arg_divisions.abs() / 2.0);

    let map = {
        let now = time::Instant::now();

        let builder = ChunkTreeBuilder3x1::new(ChunkTreeConfig {
            chunk_shape: Point3i::fill(UNPADDED_CHUNK_SIDE as i32),
            ambient_value: Sd16::from(99999.0),
            root_lod: 0,
        });
        let mut map = builder.build_with_hash_map_storage();

        let extent = {
            let divisions_half = 1i32.max(params.cmd_arg_divisions as i32 / 2);
            let from_v = Point3::fill(-divisions_half);
            let to_v = Point3::fill(divisions_half);

            println!(
                "chunks_extent {:?} chunk_side:{}, scale:{}",
                Extent3i::from_min_and_lub(
                    from_v / (UNPADDED_CHUNK_SIDE as i32),
                    to_v / (UNPADDED_CHUNK_SIDE as i32)
                ),
                UNPADDED_CHUNK_SIDE,
                scale
            );
            Extent3i::from_min_and_lub(from_v, to_v)
        };

        let sdf = |p: Point3i| {
            let pa = Point3f::from(p) * scale;
            let sin_pa: Point3f = PointN([
                params.cmd_arg_x_param * pa.x().sin(),
                params.cmd_arg_y_param * pa.y().sin(),
                params.cmd_arg_z_param * pa.z().sin(),
            ]);
            let cos_pa_zxy: Point3f = PointN([
                params.cmd_arg_z_param * pa.z().cos(),
                params.cmd_arg_x_param * pa.x().cos(),
                params.cmd_arg_y_param * pa.y().cos(),
            ]);

            // sdf formula of a gyroid is: abs(dot(sin(pa), cos(pa.zxy)) - b) - t;
            Sd16::from(
                (sin_pa.dot(cos_pa_zxy) - params.cmd_arg_b_param).abs() - params.cmd_arg_t_param,
            )
        };
        // todo: copy to each chunk in parallel
        copy_extent(&extent, &Func(Box::new(sdf)), &mut map.lod_view_mut(0));
        println!("copy_extent() duration: {:?}", now.elapsed());
        map
    };

    // scale the voxel so that the result is 3 'units' wide or so.
    let voxel_size = 3.0 / params.cmd_arg_divisions;

    // Generate the chunk meshes.
    cmd_bb_voxel::generate_mesh(voxel_size, &map)
}

#[allow(clippy::field_reassign_with_default)]
pub(crate) fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    let now = time::Instant::now();
    println!(
        r#"____________________   _________    ___ _____
\______   \______   \ /   _____/ __| _// ____\
  |   |  _/ |   |  _/ \_____  \ / __ |\   __\
  |   |   \ |   |   \ /        \ /_/ | |  |
 /______  //______  //_______  /____ | |_ |
        \/        \/         \/     \/   \/"#
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

    let (voxel_size, mesh) = build_gyroid_voxel(params)?;
    let packed_faces_model =
        cmd_bb_voxel::build_output_bp_model("gyroid".to_string(), None, voxel_size, mesh)?;
    println!(
        "Total number of vertices: {}",
        packed_faces_model.vertices.len()
    );

    println!(
        "Total number of faces: {}",
        packed_faces_model
            .faces
            .iter()
            .map(|x| x.vertices.len())
            .sum::<usize>()
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
            // tell blender to remove doubles
            PB_KeyValuePair {
                key: "REMOVE_DOUBLES".to_string(),
                value: "True".to_string(),
            },
        ],
        models: Vec::with_capacity(0),
        models32: vec![packed_faces_model],
    };
    println!("total duration: {:?}", now.elapsed());
    Ok(reply)
}
