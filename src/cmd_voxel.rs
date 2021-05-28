use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face32 as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model32 as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex32 as PB_Vertex;
use async_scoped::TokioScope;
use building_blocks::core::prelude::PointN;
use building_blocks::core::prelude::*;
use building_blocks::mesh::*;
use building_blocks::prelude::Sd16;
use building_blocks::storage::prelude::*;
use std::collections::HashMap;
use std::time;

type Point3f = PointN<[f32; 3]>;
type Extent3f = ExtentN<[f32; 3]>;

/// converts to a private, comparable and hash-able format
/// only use this for floats that are f32::is_finite().
/// This will only match on bit-perfect copies of f32
#[inline(always)]
fn transmute_to_u32(a: &[f32; 3]) -> (u32, u32, u32) {
    (a[0].to_bits(), a[1].to_bits(), a[2].to_bits())
}

/// unpack the input PB_Model
#[allow(clippy::type_complexity)]
pub fn parse_input_pb_model(
    obj: &PB_Model,
) -> Result<
    (
        Vec<PointN<[f32; 3]>>,
        Vec<(usize, usize)>,
        linestring::cgmath_3d::Aabb3<f32>,
    ),
    TBError,
> {
    let mut aabb = linestring::cgmath_3d::Aabb3::<f32>::default();
    let vertices: Vec<PointN<[f32; 3]>> = obj
        .vertices
        .iter()
        .map(|vertex| {
            aabb.update_point(&cgmath::Point3 {
                x: vertex.x,
                y: vertex.y,
                z: vertex.z,
            });
            PointN([vertex.x as f32, vertex.y as f32, vertex.z as f32])
        })
        .collect();
    let mut edges = Vec::<(usize, usize)>::with_capacity(vertices.len() + 100);

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
            *face.vertices.first().unwrap() as usize,
            *face.vertices.last().unwrap() as usize,
        ));
    }
    Ok((vertices, edges, aabb))
}

#[allow(clippy::type_complexity)]
/// Build the voxel data from the input
// todo: should i rebuild the return value to just Result<Vec<PosNormMesh>,TBError>?
// the current return value does not live very long, a re-shuffle would just take time.
fn build_voxel(
    radius_multiplier: f32,
    divisions: f32,
    vertices: Vec<PointN<[f32; 3]>>,
    edges: Vec<(usize, usize)>,
    aabb: linestring::cgmath_3d::Aabb3<f32>,
) -> Result<
    (
        f32, // <-voxel_size
        Vec<Result<Option<(PosNormMesh, Extent3i)>, tokio::task::JoinError>>,
    ),
    TBError,
> {
    let dimensions = aabb.get_high().unwrap() - aabb.get_low().unwrap();
    let max_dimension = dimensions.x.max(dimensions.y).max(dimensions.z);

    let radius = max_dimension * radius_multiplier;
    let scale = (divisions / max_dimension) as f32;
    let thickness = (radius * 2.0) as f32 * scale;
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

    println!("aabb.high:{:?}", aabb.get_high().unwrap());
    println!("aabb.low:{:?}", aabb.get_low().unwrap());
    println!(
        "delta:{:?}",
        aabb.get_high().unwrap() - aabb.get_low().unwrap()
    );
    let vertices: Vec<PointN<[f32; 3]>> = vertices.into_iter().map(|v| v * scale).collect();

    let map = {
        let now = time::Instant::now();

        let builder = ChunkMapBuilder3x1::new(PointN([16; 3]), Sd16::ONE);
        let mut map = builder.build_with_hash_map_storage();

        for (from_v, to_v) in edges
            .into_iter()
            .map(|(from, to)| (vertices[from], vertices[to]))
        {
            let sample_extent = {
                let extent_min = from_v.meet(to_v).round().into_int();
                let extent_max = from_v.join(to_v).round().into_int();
                Extent3i::from_min_and_max(extent_min, extent_max).padded(thickness.ceil() as i32)
            };
            map.for_each_mut(&sample_extent, |p: Point3i, prev_dist| {
                let pa = Point3f::from(p) - from_v;
                let ba = to_v - from_v;
                let t = pa.dot(ba) as f32 / ba.dot(ba) as f32;
                let h = t.clamp(0.0, 1.0);
                let dist = pa - (ba * h);
                let mut dist = Sd16::from(dist.norm() - thickness);
                *prev_dist = *prev_dist.min(&mut dist);
            });
        }
        println!("for_each_mut() duration: {:?}", now.elapsed());
        map
    };

    let voxel_size = 1.0 / scale;

    // Generate the chunk meshes.
    let map_ref = &map;

    let now = time::Instant::now();
    let chunk_meshes: Vec<Result<Option<(PosNormMesh, Extent3i)>, tokio::task::JoinError>> =
        TokioScope::scope_and_block(|s| {
            for chunk_key in map_ref.storage().keys() {
                s.spawn(async move {
                    let padded_chunk_extent = padded_surface_nets_chunk_extent(
                        &map_ref.indexer.extent_for_chunk_at_key(*chunk_key),
                    );
                    let mut padded_chunk = Array3x1::fill(padded_chunk_extent, Sd16(0));
                    copy_extent(&padded_chunk_extent, map_ref, &mut padded_chunk);

                    let mut surface_nets_buffer = SurfaceNetsBuffer::default();
                    // do the voxel_size multiplication later, vertices pos. needs to match extent.
                    surface_nets(
                        &padded_chunk,
                        &padded_chunk_extent,
                        1.0,
                        &mut surface_nets_buffer,
                    );

                    if surface_nets_buffer.mesh.indices.is_empty() {
                        None
                    } else {
                        Some((surface_nets_buffer.mesh, padded_chunk_extent))
                    }
                })
            }
        })
        .1;

    println!("surface_nets() duration: {:?}", now.elapsed());
    Ok((voxel_size, chunk_meshes))
}

/// Build the return model
fn build_output_bp_model(
    a_command: &PB_Command,
    voxel_size: f32,
    meshes: Vec<Result<Option<(PosNormMesh, Extent3i)>, tokio::task::JoinError>>,
) -> Result<PB_Model, TBError> {
    let input_pb_model = &a_command.models32[0];

    // calculate the maximum required v&f capacity
    let (vertex_capacity, face_capacity) = meshes
        .iter()
        .filter_map(|x| {
            if let Ok(Some((chunk, _))) = x {
                Some((chunk.positions.len(), chunk.indices.len()))
            } else {
                None
            }
        })
        .fold((0_usize, 0_usize), |(v, f), chunk| {
            (v + chunk.0, f + chunk.1)
        });
    if vertex_capacity >= u32::MAX as usize {
        return Err(TBError::Overflow(format!("Generated mesh contains too many vertices to be referenced by u32: {}. Reduce resolution.",vertex_capacity )));
    }

    if face_capacity >= u32::MAX as usize {
        return Err(TBError::Overflow(format!("Generated mesh contains too many faces to be referenced by u32: {}. Reduce resolution.",vertex_capacity )));
    }

    let mut pb_vertices: Vec<PB_Vertex> = Vec::with_capacity(vertex_capacity);
    let mut pb_faces: Vec<u32> = Vec::with_capacity(face_capacity);
    // translates between bit-perfect copies of vertices and indices of already know vertices.
    let mut unique_vertex_map: ahash::AHashMap<(u32, u32, u32), u32> = ahash::AHashMap::default();
    // translates between the index used by the chunks + indices_offset and the vertex index in pb_vertices
    let mut vertex_map: ahash::AHashMap<u32, u32> = ahash::AHashMap::default();

    let now = time::Instant::now();
    for mesh in meshes.into_iter() {
        if let Some((mesh, extent)) = mesh? {
            // each chunk starts counting vertices from zero
            let indices_offset = pb_vertices.len() as u32;
            // vertices this far inside a chunk should (probably?) not be used outside this chunk.
            let deep_inside_extent = Extent3f::from_min_and_shape(
                Point3f::from(extent.minimum),
                Point3f::from(extent.shape),
            )
            .padded(-1.5);
            for (pi, p) in mesh.positions.iter().enumerate() {
                let pv = PointN::<[f32; 3]>(*p);
                if !deep_inside_extent.contains(pv) {
                    // only use vertex de-duplication if the vertex was close to the edges
                    // of the extent
                    let key = transmute_to_u32(&p);
                    let _ = vertex_map.insert(
                        pi as u32 + indices_offset,
                        *unique_vertex_map.entry(key).or_insert_with(|| {
                            let n = pb_vertices.len() as u32;
                            pb_vertices.push(PB_Vertex {
                                x: (voxel_size * p[0]),
                                y: (voxel_size * p[1]),
                                z: (voxel_size * p[2]),
                            });
                            n
                        }),
                    );
                } else {
                    // vertex found deep inside chunk, skip vertex de-duplication.
                    let _ = vertex_map.insert(pi as u32 + indices_offset, {
                        let n = pb_vertices.len() as u32;
                        pb_vertices.push(PB_Vertex {
                            x: (voxel_size * p[0]),
                            y: (voxel_size * p[1]),
                            z: (voxel_size * p[2]),
                        });
                        n
                    });
                }
            }

            for vertex_id in mesh.indices.iter() {
                if let Some(vertex_id) = vertex_map.get(&(*vertex_id as u32 + indices_offset)) {
                    pb_faces.push(*vertex_id);
                } else {
                    return Err(TBError::InternalError(format!(
                        "Vertex id {} not found while de-duplicating vertices",
                        vertex_id
                    )));
                }
            }
        }
    }

    println!(
        "Vertex de-duplication and return model packaging duration: {:?}",
        now.elapsed()
    );

    //println!("unique_vertex_map.len(): {}", unique_vertex_map.len());
    //println!("vertex_map.len(): {}", vertex_map.len());

    /*println!(
        "pb_vertices:{}, vertex_capacity:{}",
        pb_vertices.len(),
        vertex_capacity
    );*/
    /*println!(
        "pb_faces:{}, face_capacity:{}",
        pb_face.len(),
        face_capacity
    );*/
    Ok(PB_Model {
        name: input_pb_model.name.clone(),
        world_orientation: input_pb_model.world_orientation.clone(),
        vertices: pb_vertices,
        faces: vec![PB_Face { vertices: pb_faces }],
    })
}

pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!(
        r#"____   ____                 .__
\   \ /   /______  ___ ____ |  |
 \   Y   /  _ \  \/  // __ \|  |
  \     (  <_> >    <\  ___/|  |__
   \___/ \____/__/\_ \\___  >____/
                    \/    \/      "#
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

    let (vertices, edges, aabb) = parse_input_pb_model(&a_command.models32[0])?;
    let (voxel_size, mesh) = build_voxel(
        cmd_arg_radius_multiplier,
        cmd_arg_divisions,
        vertices,
        edges,
        aabb,
    )?;
    let packed_faces_model = build_output_bp_model(&a_command, voxel_size, mesh)?;
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
