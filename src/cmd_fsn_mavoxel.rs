use crate::{
    type_utils::*, PB_Command, PB_Face32, PB_KeyValuePair, PB_Matrix4x432, PB_Model32, PB_Reply,
    PB_Vertex32, TBError,
};
use fast_surface_nets::{ndshape::ConstShape, surface_nets, SurfaceNetsBuffer};
use ilattice::glam::{f32::Affine3A, IVec3, Mat3, Vec2, Vec3, Vec3A};
use ilattice::prelude::*;
use rayon::prelude::*;
use std::collections::HashMap;
use std::{borrow::Borrow, time};

// The un-padded chunk side, it will become 16*16*16
const UN_PADDED_CHUNK_SIDE: u32 = 14_u32;
type PaddedChunkShape = fast_surface_nets::ndshape::ConstShape3u32<
    { UN_PADDED_CHUNK_SIDE + 2 },
    { UN_PADDED_CHUNK_SIDE + 2 },
    { UN_PADDED_CHUNK_SIDE + 2 },
>;
const DEFAULT_SDF_VALUE: f32 = 999.0;
type Extent3i = Extent<IVec3>;

#[allow(clippy::upper_case_acronyms)]
#[derive(Debug, Copy, Clone)]
pub(crate) enum Plane {
    XY,
    XZ,
    YZ,
}

struct UnpackedModel {
    vertices: Vec<(Vec2, f32)>,
    edges: Vec<(u32, u32)>,
    aabb: Extent<Vec3A>,
    world_orientation: Option<PB_Matrix4x432>,
    model_name: String,
}

/// unpack the input PB_Model
#[allow(clippy::type_complexity)]
fn parse_input_pb_model(
    mut a_command: PB_Command,
    cmd_arg_radius_dimension: Plane,
) -> Result<UnpackedModel, TBError> {
    let obj = a_command.models32.pop().ok_or_else(|| {
        TBError::InvalidInputData("Model did not contain any data (using model32)".to_string())
    })?;

    if obj.vertices.len() >= u32::MAX as usize {
        return Err(TBError::Overflow(format!(
            "Input data contains too many vertices. {}",
            obj.vertices.len()
        )));
    }
    let mut aabb: Option<Extent<Vec3A>> = None;

    let vertices: Vec<_> = obj
        .vertices
        .into_iter()
        .map(|vertex| {
            let (point2, radius) = match cmd_arg_radius_dimension {
                Plane::YZ => (Vec2::new(vertex.y, vertex.z), vertex.x),
                Plane::XZ => (Vec2::new(vertex.x, vertex.z), vertex.y),
                Plane::XY => (Vec2::new(vertex.x, vertex.y), vertex.z),
            };
            let v_aabb =
                Extent::from_min_and_shape(Vec3A::new(point2.x, point2.y, 0.0), Vec3A::splat(0.0))
                    .padded(radius);

            aabb = if let Some(aabb) = aabb {
                Some(aabb.bound_union(&v_aabb))
            } else {
                Some(v_aabb)
            };

            (point2, radius)
        })
        .collect();

    let mut edges = Vec::<(u32, u32)>::with_capacity(vertices.len() + 100);

    for face in obj.faces.into_iter() {
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
    Ok(UnpackedModel {
        vertices,
        edges,
        aabb: aabb.unwrap(),
        world_orientation: obj.world_orientation,
        model_name: obj.name,
    })
}

#[allow(clippy::many_single_char_names)]
/// Build the chunk lattice and spawn off thread tasks for each chunk
fn build_voxel(
    divisions: f32,
    vertices: Vec<(Vec2, f32)>,
    edges: Vec<(u32, u32)>,
    aabb: Extent<Vec3A>,
    verbose: bool,
) -> Result<
    (
        f32, // <- voxel_size
        Vec<(Vec3A, SurfaceNetsBuffer)>,
    ),
    TBError,
> {
    let max_dimension = {
        let dimensions = aabb.shape;
        dimensions.x.max(dimensions.y).max(dimensions.z)
    };

    let scale = (divisions / max_dimension) as f32;

    if verbose {
        println!(
            "Voxelizing using divisions = {}, max dimension = {}, scale factor={} (max_dimension*scale={})",
            divisions,
            max_dimension,
            scale,
            (max_dimension as f32) * scale
        );
        println!();
    }

    let rounded_cones: Vec<(RoundedCone, Extent3i)> = edges
        .into_par_iter()
        .map(|(e0, e1)| {
            let (v0, r0) = vertices[e0 as usize];
            let (v0, r0) = (Vec2::new(v0.x, v0.y) * scale, r0 * scale);
            let (v1, r1) = vertices[e1 as usize];
            let (v1, r1) = (Vec2::new(v1.x, v1.y) * scale, r1 * scale);

            let ex0 =
                Extent::<Vec3A>::from_min_and_shape(Vec3A::new(v0.x, v0.y, 0.0), Vec3A::splat(0.0))
                    .padded(r0);
            let ex1 =
                Extent::<Vec3A>::from_min_and_shape(Vec3A::new(v1.x, v1.y, 0.0), Vec3A::splat(0.0))
                    .padded(r1);
            // The AABB of the rounded cone intersected this chunk - keep it
            let v = v1 - v0;
            let _c = v0 + v * 0.5; // center
            let h = v.length();
            let b = (r0 - r1) / h;
            let a = (1.0 - b * b).sqrt();
            // todo: this can't be correct and/or efficient
            let rotation = Mat3::from_rotation_z(v.angle_between(Vec2::new(0.0, 1.0)));
            let translation = rotation.transform_point2(v0);
            let translation = -Vec3::new(translation.x(), translation.y(), 0.0);
            let m = Affine3A::from_mat3_translation(rotation, translation);

            (
                RoundedCone { r0, r1, h, b, a, m },
                ex0.bound_union(&ex1).containing_integer_extent(),
            )
        })
        .collect();

    let chunks_extent = {
        // pad with the radius + one voxel
        (aabb * (scale / (UN_PADDED_CHUNK_SIDE as f32)))
            .padded(1.0 / (UN_PADDED_CHUNK_SIDE as f32))
            .containing_integer_extent()
    };

    let now = time::Instant::now();

    let sdf_chunks: Vec<_> = {
        let un_padded_chunk_shape = IVec3::from([UN_PADDED_CHUNK_SIDE as i32; 3]);
        // Spawn off thread tasks creating and processing chunks.
        // Could also do:
        // (min.x..max.x).into_par_iter().flat_map(|x|
        //     (min.y..max.y).into_par_iter().flat_map(|y|
        //         (min.z..max.z).into_par_iter().map(|z| [x, y, z])))
        chunks_extent
            .iter3()
            .par_bridge()
            .filter_map(move |p| {
                let un_padded_chunk_extent =
                    Extent3i::from_min_and_shape(p * un_padded_chunk_shape, un_padded_chunk_shape);

                generate_and_process_sdf_chunk(un_padded_chunk_extent, &rounded_cones)
            })
            .collect()
    };
    if verbose {
        println!(
            "process_chunks() duration: {:?} generated {} chunks",
            now.elapsed(),
            sdf_chunks.len()
        );
    }
    Ok((1.0 / scale, sdf_chunks))
}

/// This is the sdf formula of a rounded cone (at origin)
///   vec2 q = vec2( length(p.xz), p.y );
///   float b = (r1-r2)/h;
///   float a = sqrt(1.0-b*b);
///   float k = dot(q,vec2(-b,a));
///   if( k < 0.0 ) return length(q) - r1;
///   if( k > a*h ) return length(q-vec2(0.0,h)) - r2;
///   return dot(q, vec2(a,b) ) - r1;
struct RoundedCone {
    r0: f32,
    r1: f32,
    h: f32,
    /// (r0-r1)/h
    b: f32,
    /// sqrt(1.0-b*b);
    a: f32,
    m: Affine3A,
}

/// Generate the data of a single chunk.
fn generate_and_process_sdf_chunk(
    un_padded_chunk_extent: Extent3i,
    rounded_cones: &[(RoundedCone, Extent3i)],
) -> Option<(Vec3A, SurfaceNetsBuffer)> {
    // the origin of this chunk, in voxel scale
    let padded_chunk_extent = un_padded_chunk_extent.padded(1);

    // filter out the edges that does not affect this chunk
    let filtered_cones: Vec<_> = rounded_cones
        .iter()
        .enumerate()
        .filter_map(|(index, sdf)| {
            if !padded_chunk_extent.intersection(sdf.1.borrow()).is_empty() {
                Some(index as u32)
            } else {
                None
            }
        })
        .collect();

    #[cfg(not(feature = "display_chunks"))]
    if filtered_cones.is_empty() {
        // no tubes intersected this chunk
        return None;
    }

    let mut array = { [DEFAULT_SDF_VALUE; PaddedChunkShape::SIZE as usize] };

    #[cfg(feature = "display_chunks")]
    // The corners of the un-padded chunk extent
    let corners: Vec<_> = un_padded_chunk_extent
        .corners3()
        .iter()
        .map(|p| p.to_float())
        .collect();

    let mut some_neg_or_zero_found = false;
    let mut some_pos_found = false;

    for pwo in padded_chunk_extent.iter3() {
        let v = {
            let p = pwo - un_padded_chunk_extent.minimum + 1;
            &mut array[PaddedChunkShape::linearize([p.x as u32, p.y as u32, p.z as u32]) as usize]
        };
        // Point With Offset from the un-padded extent minimum
        let pwo = pwo.to_float();

        #[cfg(feature = "display_chunks")]
        {
            // todo: this could probably be optimized with PaddedChunkShape::linearize(corner_pos)
            let mut x = *v;
            for c in corners.iter() {
                x = x.min(c.distance(pwo) - 1.);
            }
            *v = (*v).min(x);
        }
        for index in filtered_cones.iter() {
            let cone = &rounded_cones[*index as usize].0;
            let pwo = cone.m.transform_point3a(pwo);

            let q = Vec2::new(Vec2::new(pwo.x(), pwo.z()).length(), pwo.y());
            let k = q.dot(Vec2::new(-cone.b, cone.a));
            let new_v = if k < 0.0 {
                q.length() - cone.r0
            } else if k > cone.a * cone.h {
                (q - Vec2::new(0.0, cone.h)).length() - cone.r1
            } else {
                q.dot(Vec2::new(cone.a, cone.b)) - cone.r0
            };

            *v = (*v).min(new_v);
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
            [UN_PADDED_CHUNK_SIDE + 1; 3],
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
    cmd_arg_radius_axis: Plane,
    verbose: bool,
) -> Result<PB_Model32, TBError> {
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
        match cmd_arg_radius_axis {
            Plane::XY =>
            // Z axis is the radius dimension, no swap
            {
                for pv in mesh_buffer.positions.iter() {
                    pb_vertices.push(PB_Vertex32 {
                        x: (voxel_size * (pv[0] + vertex_offset.x)),
                        y: (voxel_size * (pv[1] + vertex_offset.y)),
                        z: (voxel_size * (pv[2] + vertex_offset.z)),
                    });
                }
            }
            Plane::XZ =>
            // Y axis is the radius dimension, swap X,Y,Z to X,Z,Y
            {
                for pv in mesh_buffer.positions.iter() {
                    pb_vertices.push(PB_Vertex32 {
                        x: (voxel_size * (pv[0] + vertex_offset.x)),
                        y: (voxel_size * (pv[2] + vertex_offset.z)),
                        z: (voxel_size * (pv[1] + vertex_offset.y)),
                    });
                }
            }
            Plane::YZ =>
            // X axis is the radius dimension, swap X,Y,Z to Y,Z,X
            {
                for pv in mesh_buffer.positions.iter() {
                    pb_vertices.push(PB_Vertex32 {
                        x: (voxel_size * (pv[2] + vertex_offset.z)),
                        y: (voxel_size * (pv[0] + vertex_offset.x)),
                        z: (voxel_size * (pv[1] + vertex_offset.y)),
                    });
                }
            }
        }
        for vertex_id in mesh_buffer.indices.iter() {
            pb_faces.push(*vertex_id + indices_offset);
        }
    }

    if verbose {
        println!(
            "Vertex return model packaging duration: {:?}",
            now.elapsed()
        );
    }
    Ok(PB_Model32 {
        name: pb_model_name,
        world_orientation: pb_world,
        vertices: pb_vertices,
        faces: vec![PB_Face32 { vertices: pb_faces }],
    })
}

/// Run the fsn_mavoxel command
pub(crate) fn command(
    a_command: PB_Command,
    options: HashMap<String, String>,
    verbose: bool,
) -> Result<PB_Reply, TBError> {
    let now = time::Instant::now();
    if verbose {
        println!(
            r#"___________                _____     _____  ____   ___                     __
\_   _____/ ______ ____   /     \   /  _  \ \   \ /  / ____ ___  ___ ____ |  |
  |  ___)  /  ___//    \ /  \ /  \ /  /_\  \ \   \  / / __ \\  \/  // __ \|  |
  |  \__   \___ \|   |  \    \    \    |    \ \    / (  \_\ )\    /\  ___/_  |__
 /___  /  /____  \___|  /____/\_  /____|__  /  \  /   \____//__/\_ \\___  /____/
     \/        \/     \/        \/        \/    \/                \/    \/      "#
        );
        //crate::print_command(&a_command);
    }
    if a_command.models32.len() > 1 {
        return Err(TBError::InvalidInputData(format!(
            "This operation only supports one model as input:{}",
            a_command.models32.len()
        )));
    }

    let cmd_arg_radius_axis = match options
        .get("RADIUS_AXIS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the RADIUS_AXIS parameter".to_string()))?
        .as_str()
    {
        "XY" => Ok(Plane::XY),
        "XZ" => Ok(Plane::XZ),
        "YZ" => Ok(Plane::YZ),
        something_else => Err(TBError::InvalidInputData(format!(
            "Invalid RADIUS_AXIS parameter: {}",
            something_else
        ))),
    }?;

    let cmd_arg_divisions = options
        .get("DIVISIONS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the DIVISIONS parameter".to_string()))?
        .parse::<f32>()
        .map_err(|_| {
            TBError::InvalidInputData("Could not parse the DIVISIONS parameter".to_string())
        })?;
    if !(9.9..600.1).contains(&cmd_arg_divisions) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of DIVISIONS is [{}..{}[% :({})",
            10, 600, cmd_arg_divisions
        )));
    }

    if verbose {
        for model in a_command.models32.iter() {
            println!("model.name:{:?}, ", model.name);
            println!("model.vertices:{:?}, ", model.vertices.len());
            println!("model.faces:{:?}, ", model.faces.len());
            println!(
                "model.world_orientation:{:?}, ",
                model.world_orientation.as_ref().map_or(0, |_| 16)
            );
            println!("Voxel divisions:{:?} ", cmd_arg_divisions);
            println!(
                "2d point axis:{:?}. Radius is the third axis",
                cmd_arg_radius_axis
            );
            println!();
        }
    }

    let unpacked = parse_input_pb_model(a_command, cmd_arg_radius_axis)?;
    let (voxel_size, mesh) = build_voxel(
        cmd_arg_divisions,
        unpacked.vertices,
        unpacked.edges,
        unpacked.aabb,
        verbose,
    )?;
    let packed_faces_model = build_output_bp_model(
        unpacked.model_name,
        unpacked.world_orientation,
        voxel_size,
        mesh,
        cmd_arg_radius_axis,
        verbose,
    )?;
    if verbose {
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
    }
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
        models: Vec::default(),
        models32: vec![packed_faces_model],
    };
    if verbose {
        println!("total duration: {:?}", now.elapsed());
    }
    Ok(reply)
}
