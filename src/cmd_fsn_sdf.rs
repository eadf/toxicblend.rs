use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::type_utils::*;
use cgmath::num_traits::FloatConst;
use fast_surface_nets::ndshape::ConstShape;
use fast_surface_nets::{surface_nets, SurfaceNetsBuffer};
use ilattice::glam::{IVec3, Vec3A};
use ilattice::prelude::*;

use rayon::prelude::*;
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

// The un-padded chunk side, it will become 16*16*16
const UNPADDED_CHUNK_SIDE: u32 = 14_u32;

type PaddedChunkShape = fast_surface_nets::ndshape::ConstShape3u32<
    { UNPADDED_CHUNK_SIDE + 2 },
    { UNPADDED_CHUNK_SIDE + 2 },
    { UNPADDED_CHUNK_SIDE + 2 },
>;

type Extent3i = Extent<IVec3>;

/// initialize the example sdf gyroid and generate the mesh buffers
fn build_gyroid_voxel(
    mut params: GyroidParameters,
) -> Result<
    (
        f32, // <- voxel_size
        Vec<(Vec3A, SurfaceNetsBuffer)>,
    ),
    TBError,
> {
    let chunks_per_side = params.cmd_arg_divisions as i32 / UNPADDED_CHUNK_SIDE as i32;
    params.cmd_arg_divisions = (UNPADDED_CHUNK_SIDE as i32 * chunks_per_side) as f32;

    println!(
        "Voxelizing gyroid using divisions={}, s={}, t={}, b={}",
        params.cmd_arg_divisions,
        params.cmd_arg_s_param,
        params.cmd_arg_t_param,
        params.cmd_arg_b_param
    );
    println!();

    let chunks_extent = {
        let half_side = 1.max(params.cmd_arg_divisions as i32 / (UNPADDED_CHUNK_SIDE as i32) / 2);
        params.cmd_arg_divisions = UNPADDED_CHUNK_SIDE as f32 * half_side as f32 * 2.0;

        let extent_min = IVec3::from([-half_side; 3]);
        let extent_max = IVec3::from([half_side; 3]);
        Extent3i::from_min_and_lub(extent_min, extent_max)
    };

    // set scale so that extent.min*scale -> -pi, extent.max*scale -> pi
    // when cmd_arg_s_param is 1.0
    let scale = params.cmd_arg_s_param * f32::PI() / (params.cmd_arg_divisions.abs() / 2.0);

    //let chunks_extent = Extent3i::from_min_and_lub(IVec3::new(-1,-1,-1), IVec3::new(0,0,0));
    println!("chunks_extent {:?} scale:{}", chunks_extent, scale);

    let now = time::Instant::now();

    let sdf_chunks = process_chunks(
        &params,
        scale,
        chunks_extent,
        IVec3::from([UNPADDED_CHUNK_SIDE as i32; 3]),
        999999f32,
        (chunks_extent * (UNPADDED_CHUNK_SIDE as i32)).padded(-1),
    );

    println!(
        "process_chunks() duration: {:?} generated {} chunks",
        now.elapsed(),
        sdf_chunks.len()
    );

    // scale the voxel so that the result is 3 'units' wide or so.
    let voxel_size = 3.0 / params.cmd_arg_divisions;
    Ok((voxel_size, sdf_chunks))
}

#[inline]
/// Spawn off threads creating and processing chunks.
/// Returns a Vec of processed chunks as (chunk_origin, SurfaceNetsBuffer).
fn process_chunks(
    params: &GyroidParameters,
    scale: f32,
    chunks_extent: Extent3i,
    unpadded_chunk_shape: IVec3,
    default_sdf_value: f32,
    world_extent: Extent3i,
) -> Vec<(Vec3A, SurfaceNetsBuffer)> {
    let min = chunks_extent.minimum;
    let max = chunks_extent.least_upper_bound();

    // Could also do:
    // (min.x..max.x).into_par_iter().flat_map(|x|
    //     (min.y..max.y).into_par_iter().flat_map(|y|
    //         (min.z..max.z).into_par_iter().map(|z| [x, y, z])))
    itertools::iproduct!(min.x..max.x, min.y..max.y, min.z..max.z)
        .par_bridge()
        .filter_map(move |p| {
            let chunk_min = IVec3::from(p) * unpadded_chunk_shape;
            let unpadded_chunk_extent =
                Extent3i::from_min_and_shape(chunk_min, unpadded_chunk_shape);

            generate_and_process_sdf_chunk(
                params,
                scale,
                unpadded_chunk_extent,
                default_sdf_value,
                &world_extent,
            )
        })
        .collect()
}

/// Generate the data of a single chunk
fn generate_and_process_sdf_chunk(
    params: &GyroidParameters,
    scale: f32,
    unpadded_chunk_extent: Extent3i,
    default_sdf_value: f32,
    world_extent: &Extent3i,
) -> Option<(Vec3A, SurfaceNetsBuffer)> {
    // the origin of this chunk, in voxel scale
    let p_offset_min = unpadded_chunk_extent.minimum;

    let mut array = { [default_sdf_value; PaddedChunkShape::SIZE as usize] };

    #[cfg(feature = "display_chunks")]
    // The corners of the un-padded chunk extent
    let corners: Vec<_> = unpadded_chunk_extent
        .corners()
        .iter()
        .map(|p| p.to_float())
        .collect();

    let mut some_neg_or_zero_found = false;
    let mut some_pos_found = false;

    let sdf = |pwo: &Vec3A| {
        let pwo = (*pwo) * scale;
        let sin_pa = Vec3A::from([
            params.cmd_arg_x_param * pwo.x().sin(),
            params.cmd_arg_y_param * pwo.y().sin(),
            params.cmd_arg_z_param * pwo.z().sin(),
        ]);
        let cos_pa_zxy = Vec3A::from([
            params.cmd_arg_z_param * pwo.z().cos(),
            params.cmd_arg_x_param * pwo.x().cos(),
            params.cmd_arg_y_param * pwo.y().cos(),
        ]);
        // sdf formula of a gyroid is: abs(dot(sin(pa), cos(pa.zxy)) - b) - t;
        (sin_pa.dot(cos_pa_zxy) - params.cmd_arg_b_param).abs() - params.cmd_arg_t_param
    };

    for (i, v) in array.iter_mut().enumerate() {
        // Point With Offset from the un-padded extent minimum
        let pwo = to_ivec(PaddedChunkShape::delinearize(i as u32)) + p_offset_min;
        let pwof = pwo.to_float();
        #[cfg(feature = "display_chunks")]
        {
            // todo: this could probably be optimized with PaddedChunkShape::linearize(corner_pos)
            let mut x = *v;
            for c in corners.iter() {
                x = x.min(c.distance(pwof) - 1.);
            }
            *v = (*v).min(x);
        }
        *v = (*v).min(sdf(&pwof));

        if *v <= 0.0 {
            some_neg_or_zero_found = true;
            if *v < 0.0 && !world_extent.contains(pwo) {
                // Cap off the sdf
                *v = 0.0;
                // and make sure this chunk is kept
                some_pos_found = true;
            }
        } else {
            some_pos_found = true;
        }
    }
    if some_pos_found && some_neg_or_zero_found {
        // A combination of positive and negative surfaces found - process this chunk
        let mut sn_buffer = SurfaceNetsBuffer::default();

        // do the voxel_size multiplication later, vertices pos. needs to match extent.
        surface_nets(
            &array,
            &PaddedChunkShape {},
            [0; 3],
            [UNPADDED_CHUNK_SIDE + 1; 3],
            &mut sn_buffer,
        );

        if sn_buffer.positions.is_empty() {
            // No vertices were generated by this chunk, ignore it
            None
        } else {
            Some((p_offset_min.to_float(), sn_buffer))
        }
    } else {
        None
    }
}

#[inline]
fn to_ivec([x, y, z]: [u32; 3]) -> IVec3 {
    IVec3::new(x as i32, y as i32, z as i32)
}

#[allow(clippy::field_reassign_with_default)]
pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    let now = time::Instant::now();
    println!(
        r#"___________             _________    .___ _____
\_   _____/_____ ____  /   _____/  __| _// ____\
 |    __)/  ___//    \ \_____  \  / __ |\   __\
 |     \ \___ \|   |  \/        \/ /_/ | |  |
 \___  //____  >___|  /_______  /\____ | |__|
     \/      \/     \/        \/      \/ "#
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
        crate::cmd_fsn_voxel::build_output_bp_model("gyroid".to_string(), None, voxel_size, mesh)?;
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
