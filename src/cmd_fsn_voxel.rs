use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face32 as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Matrix4x432 as PB_Matrix4x432;
use crate::toxicblend_pb::Model32 as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex32 as PB_Vertex;
use crate::{type_utils::*, TBError};
use fast_surface_nets::{ndshape::ConstShape, surface_nets, SurfaceNetsBuffer};
use ilattice::glam::{IVec3, Vec3A};
use ilattice::prelude::*;

use rayon::prelude::*;
use std::collections::HashMap;
use std::time;

// The un-padded chunk side, it will become 16*16*16
const UNPADDED_CHUNK_SIDE: u32 = 14_u32;
type PaddedChunkShape = fast_surface_nets::ndshape::ConstShape3u32<
    { UNPADDED_CHUNK_SIDE + 2 },
    { UNPADDED_CHUNK_SIDE + 2 },
    { UNPADDED_CHUNK_SIDE + 2 },
>;
const DEFAULT_SDF_VALUE: f32 = 999.0;
type Extent3i = Extent<IVec3>;

/// unpack the input PB_Model
#[allow(clippy::type_complexity)]
fn parse_input_pb_model(
    obj: &PB_Model,
) -> Result<(Vec<(u32, u32)>, linestring::linestring_3d::Aabb3<f32>), TBError> {
    let mut aabb = linestring::linestring_3d::Aabb3::<f32>::default();
    if obj.vertices.len() >= u32::MAX as usize {
        return Err(TBError::Overflow(format!(
            "Input data contains too many vertices. {}",
            obj.vertices.len()
        )));
    }
    for vertex in obj.vertices.iter() {
        aabb.update_point(&cgmath::Point3 {
            x: vertex.x,
            y: vertex.y,
            z: vertex.z,
        });
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

/// Build the chunk lattice and spawn off thread tasks for each chunk
fn build_voxel(
    radius_multiplier: f32,
    divisions: f32,
    vertices: &[PB_Vertex],
    edges: Vec<(u32, u32)>,
    aabb: linestring::linestring_3d::Aabb3<f32>,
) -> Result<
    (
        f32, // <- voxel_size
        Vec<(Vec3A, SurfaceNetsBuffer)>,
    ),
    TBError,
> {
    let dimensions = aabb.get_high().unwrap() - aabb.get_low().unwrap();
    let max_dimension = dimensions.x.max(dimensions.y).max(dimensions.z);

    let radius = max_dimension * radius_multiplier; // unscaled
    let scale = (divisions / max_dimension) as f32;

    println!(
        "Voxelizing using tube radius. {} = {}*{}*{}",
        radius, max_dimension, radius_multiplier, scale
    );

    println!(
        "Voxelizing using divisions = {}, max dimension = {}, scale factor={} (max_dimension*scale={})",
        divisions,
        max_dimension,
        scale,
        (max_dimension as f32) * scale
    );
    println!();

    let vertices: Vec<Vec3A> = vertices
        .iter()
        .map(|v| Vec3A::new(v.x, v.y, v.z) * scale)
        .collect();

    let chunks_extent = {
        let min_p = aabb.get_low().unwrap().to_float();
        let max_p = aabb.get_high().unwrap().to_float();
        // pad with the radius + one voxel
        (Extent::<Vec3A>::from_min_and_lub(min_p, max_p).padded(radius)
            * (scale / (UNPADDED_CHUNK_SIDE as f32)))
            .padded(1.0 / (UNPADDED_CHUNK_SIDE as f32))
            .containing_integer_extent()
    };

    let now = time::Instant::now();

    let sdf_chunks: Vec<_> = {
        let radius = radius * scale;
        let unpadded_chunk_shape = IVec3::from([UNPADDED_CHUNK_SIDE as i32; 3]);
        // Spawn off thread tasks creating and processing chunks.
        // Could also do:
        // (min.x..max.x).into_par_iter().flat_map(|x|
        //     (min.y..max.y).into_par_iter().flat_map(|y|
        //         (min.z..max.z).into_par_iter().map(|z| [x, y, z])))
        chunks_extent
            .iter3()
            .par_bridge()
            .filter_map(move |p| {
                let unpadded_chunk_extent =
                    Extent3i::from_min_and_shape(p * unpadded_chunk_shape, unpadded_chunk_shape);

                generate_and_process_sdf_chunk(unpadded_chunk_extent, &vertices, &edges, radius)
            })
            .collect()
    };

    println!(
        "process_chunks() duration: {:?} generated {} chunks",
        now.elapsed(),
        sdf_chunks.len()
    );

    Ok((1.0 / scale, sdf_chunks))
}

/// Generate the data of a single chunk
fn generate_and_process_sdf_chunk(
    unpadded_chunk_extent: Extent3i,
    vertices: &[Vec3A],
    edges: &[(u32, u32)],
    thickness: f32,
) -> Option<(Vec3A, SurfaceNetsBuffer)> {
    // the origin of this chunk, in voxel scale
    let padded_chunk_extent = unpadded_chunk_extent.padded(1);

    // filter out the edges that does not affect this chunk
    let filtered_edges: Vec<_> = edges
        .iter()
        .filter_map(|(e0, e1)| {
            let (e0, e1) = (*e0 as usize, *e1 as usize);
            let tube_extent = Extent::from_min_and_lub(
                vertices[e0].min(vertices[e1]) - Vec3A::from([thickness; 3]),
                vertices[e0].max(vertices[e1]) + Vec3A::from([thickness; 3]),
            )
            .containing_integer_extent();
            if !padded_chunk_extent.intersection(&tube_extent).is_empty() {
                // The AABB of the edge tube intersected this chunk - keep it
                Some((e0, e1))
            } else {
                None
            }
        })
        .collect();

    #[cfg(not(feature = "display_chunks"))]
    if filtered_edges.is_empty() {
        // no tubes intersected this chunk
        return None;
    }

    let mut array = { [DEFAULT_SDF_VALUE; PaddedChunkShape::SIZE as usize] };

    #[cfg(feature = "display_chunks")]
    // The corners of the un-padded chunk extent
    let corners: Vec<_> = unpadded_chunk_extent
        .corners3()
        .iter()
        .map(|p| p.to_float())
        .collect();

    let mut some_neg_or_zero_found = false;
    let mut some_pos_found = false;

    for pwo in padded_chunk_extent.iter3() {
        let v = {
            let p = pwo - unpadded_chunk_extent.minimum + 1;
            &mut array[PaddedChunkShape::linearize([p.x as u32, p.y as u32, p.z as u32]) as usize]
        };
        let pwo = pwo.to_float();
        // Point With Offset from the un-padded extent minimum
        //let pwo = to_float(PaddedChunkShape::delinearize(i as u32)) + p_offset_min;
        #[cfg(feature = "display_chunks")]
        {
            // todo: this could probably be optimized with PaddedChunkShape::linearize(corner_pos)
            let mut x = *v;
            for c in corners.iter() {
                x = x.min(c.distance(pwo) - 1.);
            }
            *v = (*v).min(x);
        }
        for (from_v, to_v) in filtered_edges
            .iter()
            .map(|(e0, e1)| (vertices[*e0], vertices[*e1]))
        {
            // This is the sdf formula of a capsule
            let pa = pwo - from_v;
            let ba = to_v - from_v;
            let t = pa.dot(ba) / ba.dot(ba);
            let h = t.clamp(0.0, 1.0);
            *v = (*v).min((pa - (ba * h)).length() - thickness);
        }
        if *v > 0.0 {
            some_pos_found = true;
        } else {
            some_neg_or_zero_found = true;
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
            Some((padded_chunk_extent.minimum.to_float(), sn_buffer))
        }
    } else {
        None
    }
}

/// Build the return model
pub(crate) fn build_output_bp_model(
    pb_model_name: String,
    pb_world: Option<PB_Matrix4x432>,
    voxel_size: f32,
    mesh_buffers: Vec<(Vec3A, SurfaceNetsBuffer)>,
) -> Result<PB_Model, TBError> {
    let now = time::Instant::now();

    let (mut pb_vertices, mut pb_faces) = {
        // calculate the maximum required vertices & facec capacity
        let (vertex_capacity, face_capacity) = mesh_buffers
            .iter()
            .fold((0_usize, 0_usize), |(v, f), chunk| {
                (v + chunk.1.positions.len(), f + chunk.1.indices.len())
            });
        if vertex_capacity >= u32::MAX as usize {
            return Err(TBError::Overflow(format!("Generated mesh contains too many vertices to be referenced by u32: {}. Reduce the resolution.", vertex_capacity)));
        }

        if face_capacity >= u32::MAX as usize {
            return Err(TBError::Overflow(format!("Generated mesh contains too many faces to be referenced by u32: {}. Reduce the resolution.", vertex_capacity)));
        }
        (
            Vec::with_capacity(vertex_capacity),
            Vec::with_capacity(face_capacity),
        )
    };

    for (vertex_offset, mesh_buffer) in mesh_buffers.iter() {
        // each chunk starts counting vertices from zero
        let indices_offset = pb_vertices.len() as u32;

        // vertices this far inside a chunk should (probably?) not be used outside this chunk.
        for pv in mesh_buffer.positions.iter() {
            pb_vertices.push(PB_Vertex {
                x: (voxel_size * (pv[0] + vertex_offset.x)),
                y: (voxel_size * (pv[1] + vertex_offset.y)),
                z: (voxel_size * (pv[2] + vertex_offset.z)),
            });
        }
        for vertex_id in mesh_buffer.indices.iter() {
            pb_faces.push(*vertex_id + indices_offset);
        }
    }

    println!(
        "Vertex return model packaging duration: {:?}",
        now.elapsed()
    );

    Ok(PB_Model {
        name: pb_model_name,
        world_orientation: pb_world,
        vertices: pb_vertices,
        faces: vec![PB_Face { vertices: pb_faces }],
    })
}

pub fn command(
    a_command: PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    let now = time::Instant::now();

    println!(
        r#"___________         ____   ____                 .__
\_   _____/_____ ___\   \ /   /______  ___ ____ |  |
 |    __)/  ___//    \   Y   /  _ \  \/  // __ \|  |
 |     \ \___ \|   |  \     (  <_> >    <\  ___/|  |__
 \___  //____  >___|  /\___/ \____/__/\_ \\___  >____/
     \/      \/     \/                  \/    \/"#
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
