use super::TBError;
use crate::cmd_voxel;
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

#[allow(clippy::type_complexity)]
#[allow(clippy::too_many_arguments)]
/// Build the gyroid voxel data from the input
// todo: should i rebuild the return value to just Result<Vec<PosNormMesh>,TBError>?
// the current return value does not live very long, a re-shuffle would just take time.
fn build_gyroid_voxel(
    divisions: f32,
    cmd_arg_s_param: f32,
    cmd_arg_t_param: f32,
    cmd_arg_b_param: f32,
    cmd_arg_x_param: f32,
    cmd_arg_y_param: f32,
    cmd_arg_z_param: f32,
) -> Result<
    (
        f32, // <-voxel_size
        Vec<Result<Option<(PosNormMesh, Extent3i)>, tokio::task::JoinError>>,
    ),
    TBError,
> {
    println!(
        "Voxelizing gyroid using divisions={}, s={}, t={}, b={}",
        divisions, cmd_arg_s_param, cmd_arg_t_param, cmd_arg_b_param
    );
    println!();
    // set scale so that extent.min*scale -> -pi, extent.max*scale -> pi
    // when cmd_arg_s_param is 1.0
    let scale = cmd_arg_s_param * f32::PI() / (divisions.abs() / 2.0);

    let map = {
        let now = time::Instant::now();

        let builder = ChunkMapBuilder3x1::new(PointN([16; 3]), Sd16::ONE);
        let mut map = builder.build_with_hash_map_storage();

        let extent = {
            let divisions_half = divisions.abs() / 2.0;
            let from_v = Point3f::fill(-divisions_half);
            let to_v = Point3f::fill(divisions_half);
            let extent_min = from_v.meet(to_v).round().into_int();
            let extent_max = from_v.join(to_v).round().into_int();
            Extent3i::from_min_and_max(extent_min, extent_max) //.padded(thickness.ceil() as i32)
        };
        map.for_each_mut(&extent, |p: Point3i, prev_dist| {
            let pa = Point3f::from(p) * scale;
            let sin_pa: Point3f = PointN([
                cmd_arg_x_param * pa.x().sin(),
                cmd_arg_y_param * pa.y().sin(),
                cmd_arg_z_param * pa.z().sin(),
            ]);
            let cos_pa_zxy: Point3f = PointN([
                cmd_arg_z_param * pa.z().cos(),
                cmd_arg_x_param * pa.x().cos(),
                cmd_arg_y_param * pa.y().cos(),
            ]);

            // sdf formula of a gyroid is: abs(dot(sin(pa), cos(pa.zxy)) - b) - t;
            let mut dist =
                Sd16::from((sin_pa.dot(cos_pa_zxy) - cmd_arg_b_param).abs() - cmd_arg_t_param);
            *prev_dist = *prev_dist.min(&mut dist);
        });
        println!("for_each_mut() duration: {:?}", now.elapsed());
        map
    };

    // scale the voxel so that the result is 3 'units' wide or so.
    let voxel_size = 3.0 / divisions;

    // Generate the chunk meshes.
    cmd_voxel::generate_mesh(voxel_size, &map)
}

pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!(
        r#"  ________                     .__    .___
 /  _____/___.__._______  ____ |__| __| _/
/   \  __<   |  |\_  __ \/  _ \|  |/ __ |
\    \_\  \___  | |  | \(  <_> )  / /_/ |
 \______  / ____| |__|   \____/|__\____ |
        \/\/                           \/ "#
    );

    if !a_command.models32.is_empty() {
        return Err(TBError::InvalidInputData(format!(
            "This operation does not need any models as input:{}",
            a_command.models32.len()
        )));
    }
    let cmd_arg_t_param = options
        .get("T")
        .ok_or_else(|| TBError::InvalidInputData("Missing the T parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the T parameter".to_string()))?;
    let cmd_arg_b_param = options
        .get("B")
        .ok_or_else(|| TBError::InvalidInputData("Missing the B parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the B parameter".to_string()))?;
    let cmd_arg_s_param = options
        .get("S")
        .ok_or_else(|| TBError::InvalidInputData("Missing the S parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the S parameter".to_string()))?;
    let cmd_arg_x_param = options
        .get("X")
        .ok_or_else(|| TBError::InvalidInputData("Missing the X parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the Y parameter".to_string()))?;
    let cmd_arg_y_param = options
        .get("Y")
        .ok_or_else(|| TBError::InvalidInputData("Missing the Y parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the Y parameter".to_string()))?;
    let cmd_arg_z_param = options
        .get("Z")
        .ok_or_else(|| TBError::InvalidInputData("Missing the Z parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| TBError::InvalidInputData("Could not parse the Z parameter".to_string()))?;
    let cmd_arg_divisions = options
        .get("DIVISIONS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the DIVISIONS parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| {
            TBError::InvalidInputData("Could not parse the DIVISIONS parameter".to_string())
        })?;
    let cmd_arg_plug_ends = options
        .get("PLUG_ENDS")
        .cloned()
        .unwrap_or_else(|| "false".to_string())
        //.ok_or_else(|| TBError::InvalidInputData("Missing the PLUG_ENDS parameter".to_string()))?;
        .to_lowercase();
    let cmd_arg_plug_ends = cmd_arg_plug_ends.parse::<bool>().map_err(|_| {
        TBError::InvalidInputData(format!(
            "Could not parse the PLUG_ENDS parameter: '{}'",
            cmd_arg_plug_ends
        ))
    })?;
    if !(9.9..400.1).contains(&cmd_arg_divisions) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of DIVISIONS is [{}..{}[% :({})",
            10, 400, cmd_arg_divisions
        )));
    }

    println!("Voxel divisions:{:?} ", cmd_arg_divisions);
    println!("s parameter:{:?} ", cmd_arg_s_param);
    println!("t parameter:{:?} ", cmd_arg_t_param);
    println!("b parameter:{:?} ", cmd_arg_b_param);
    println!("x parameter:{:?} ", cmd_arg_x_param);
    println!("y parameter:{:?} ", cmd_arg_y_param);
    println!("z parameter:{:?} ", cmd_arg_z_param);
    //println!("plug ends:{:?} ", cmd_arg_plug_ends);
    println!();

    let (voxel_size, mesh) = build_gyroid_voxel(
        cmd_arg_divisions,
        cmd_arg_s_param,
        cmd_arg_t_param,
        cmd_arg_b_param,
        cmd_arg_x_param,
        cmd_arg_y_param,
        cmd_arg_z_param,
    )?;
    let packed_faces_model =
        cmd_voxel::build_output_bp_model("gyroid".to_string(), None, voxel_size, mesh)?;
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
        ],
        models: Vec::with_capacity(0),
        models32: vec![packed_faces_model],
    };
    Ok(reply)
}
