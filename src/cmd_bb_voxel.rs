use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face32 as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Matrix4x432 as PB_Matrix4x432;
use crate::toxicblend_pb::Model32 as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex32 as PB_Vertex;
use building_blocks::core::prelude::PointN;
use building_blocks::core::prelude::*;
use building_blocks::mesh::*;
use building_blocks::storage::prelude::*;
use rayon::prelude::*;
use std::collections::HashMap;
use std::time;

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
        Vec<Point3f>,
        Vec<(u32, u32)>,
        linestring::linestring_3d::Aabb3<f32>,
    ),
    TBError,
> {
    let mut aabb = linestring::linestring_3d::Aabb3::<f32>::default();
    if obj.vertices.len() >= u32::MAX as usize {
        return Err(TBError::Overflow(format!(
            "Input data contains too many vertices. {}",
            obj.vertices.len()
        )));
    }

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
    let mut edges = Vec::<(u32, u32)>::with_capacity(vertices.len() + 100);

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
    Ok((vertices, edges, aabb))
}

fn build_voxel(
    radius_multiplier: f32,
    divisions: f32,
    vertices: Vec<PointN<[f32; 3]>>,
    edges: Vec<(u32, u32)>,
    aabb: linestring::linestring_3d::Aabb3<f32>,
) -> Result<
    (
        f32, // <- voxel_size
        Vec<(PosNormMesh, Extent3i)>,
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

    println!("aabb.high:{:?}", aabb.get_high().unwrap());
    println!("aabb.low:{:?}", aabb.get_low().unwrap());
    println!(
        "delta:{:?}",
        aabb.get_high().unwrap() - aabb.get_low().unwrap()
    );
    let vertices: Vec<PointN<[f32; 3]>> = vertices.into_iter().map(|v| v * scale).collect();

    let chunk_chape_dim = 16_i32;
    let total_extent = {
        let min_p = (Point3f::from(aabb.get_low().unwrap()) - Point3f::fill(radius)) * scale;
        let max_p = (Point3f::from(aabb.get_high().unwrap()) + Point3f::fill(radius)) * scale;

        /*
        println!("scale {:?}", scale);
        println!("thickness {:?} radius {:?}", thickness, radius);
        println!("min_p: {:?} {}", min_p, " is the min point of the generated graphics");
        println!("max_p: {:?} {}", max_p, " is the max point of the generated graphics");
        println!("delta_p: {:?}", max_p-min_p);
        */
        let extent_min = (min_p / chunk_chape_dim as f32).floor().into_int();
        // todo why is -0.9 needed, seems like there is an off-by-one error somewhere???
        let extent_max = ((max_p / chunk_chape_dim as f32) - Point3f::fill(0.9))
            .ceil()
            .into_int();
        let extent = Extent3i::from_min_and_max(extent_min, extent_max);
        /*
        println!("extent_min {:?} coverts {:?}", extent_min, (extent_min*chunk_chape_dim));
        println!("extent_max {:?} coverts {:?}", extent_max, (extent_max*chunk_chape_dim));
        println!("extent {:?}", extent);
        println!("chunk size {}x{}x{}", chunk_chape_dim,chunk_chape_dim,chunk_chape_dim);
        println!("extent.shape*chunk size {:?}", extent.shape*chunk_chape_dim);
        */
        // todo: might result in a slightly over/under-sized extent
        // todo: there must be better to "nudge" input data to better fit integer chunks
        ChunkUnits(extent)
    };
    println!("total_extent {:?}", total_extent);

    let now = time::Instant::now();

    let sdf_chunks = generate_sdf_chunks3(
        total_extent,
        PointN::fill(chunk_chape_dim),
        &vertices,
        &edges,
        radius * scale,
    );
    //unimplemented!();
    let builder = ChunkTreeBuilder3x1::new(ChunkTreeConfig {
        chunk_shape: PointN([chunk_chape_dim; 3]),
        ambient_value: Sd16::from(99999.0),
        root_lod: 0,
    });
    let mut map = builder.build_with_hash_map_storage();
    for (chunk_min, chunk) in sdf_chunks.into_iter() {
        let _ = map.write_chunk(ChunkKey::new(0, chunk_min), chunk);
    }

    println!("generate_sdf_chunks3() duration: {:?}", now.elapsed());

    let voxel_size = 1.0 / scale;

    generate_mesh(voxel_size, &map)
}

/// Spawn off threads creating the chunks, returns a vec of chunks.
/// One thread task per chunk
pub fn generate_sdf_chunks3(
    total_extent: ChunkUnits<Extent3i>,
    chunk_shape: Point3i,
    vertices: &[Point3f],
    edges: &[(u32, u32)],
    thickness: f32,
) -> Vec<(Point3i, Array3x1<Sd16>)> {
    total_extent
        .0
        .iter_points()
        .par_bridge()
        .filter_map(move |p| {
            let chunk_min = p * chunk_shape;
            let chunk_extent = Extent3i::from_min_and_shape(chunk_min, chunk_shape);

            generate_sdf_chunk3(chunk_extent, vertices, edges, thickness).map(|c| (chunk_min, c))
        })
        .collect()
}

/// Generate the data of a single chunk
pub fn generate_sdf_chunk3(
    chunk_extent: Extent3i,
    vertices: &[Point3f],
    edges: &[(u32, u32)],
    thickness: f32,
) -> Option<Array3x1<Sd16>> {
    // filter out the edges that does not affect this chunk
    let edges = edges
        .iter()
        .filter_map(|(e0, e1)| {
            let (e0, e1) = (*e0 as usize, *e1 as usize);
            let tube_extent: Extent3i = {
                let extent_min = (vertices[e0].meet(vertices[e1]) - Point3f::fill(thickness))
                    .floor()
                    .into_int();
                let extent_max = (vertices[e0].join(vertices[e1]) + Point3f::fill(thickness))
                    .ceil()
                    .into_int();
                Extent3i::from_min_and_max(extent_min, extent_max)
            };
            if !chunk_extent.intersection(&tube_extent).is_empty() {
                Some((e0, e1))
            } else {
                None
            }
        })
        .collect::<Vec<_>>();
    #[cfg(not(feature = "display_chunks"))]
    if edges.is_empty() {
        // no tubes intersected this chunk
        return None;
    }
    let usable_edges = &edges;

    let mut array = create_chunk_array(chunk_extent, Sd16::from(99999.9));
    #[cfg(feature = "display_chunks")]
    let c = {
        let shape = (
            chunk_extent.shape.x() as f32,
            chunk_extent.shape.y() as f32,
            chunk_extent.shape.z() as f32,
        );
        (
            Point3f::from(chunk_extent.minimum),
            Point3f::from(chunk_extent.minimum) + PointN([shape.0, 0.0, 0.0]),
            Point3f::from(chunk_extent.minimum) + PointN([shape.0, shape.1, 0.0]),
            Point3f::from(chunk_extent.minimum) + PointN([0.0, shape.1, 0.0]),
            Point3f::from(chunk_extent.minimum) + PointN([0.0, 0.0, shape.2]),
            Point3f::from(chunk_extent.minimum) + PointN([shape.0, 0.0, shape.2]),
            Point3f::from(chunk_extent.minimum) + PointN([shape.0, shape.1, shape.2]),
            Point3f::from(chunk_extent.minimum) + PointN([0.0, shape.1, shape.2]),
        )
    };
    array.for_each_mut(&chunk_extent, |(p, _), x| {
        #[cfg(feature = "display_chunks")]
        {
            let pf = Point3f::from(p);
            *x = *x.min(&mut Sd16::from(c.0.l2_distance_squared(pf).sqrt() - 1.5));
            *x = *x.min(&mut Sd16::from(c.1.l2_distance_squared(pf).sqrt() - 1.5));
            *x = *x.min(&mut Sd16::from(c.2.l2_distance_squared(pf).sqrt() - 1.5));
            *x = *x.min(&mut Sd16::from(c.3.l2_distance_squared(pf).sqrt() - 1.5));
            *x = *x.min(&mut Sd16::from(c.4.l2_distance_squared(pf).sqrt() - 1.5));
            *x = *x.min(&mut Sd16::from(c.5.l2_distance_squared(pf).sqrt() - 1.5));
            *x = *x.min(&mut Sd16::from(c.6.l2_distance_squared(pf).sqrt() - 1.5));
            *x = *x.min(&mut Sd16::from(c.7.l2_distance_squared(pf).sqrt() - 1.5));
        }
        for (e0, e1) in usable_edges.iter() {
            let from_v = vertices[*e0];
            let to_v = vertices[*e1];

            let pa = Point3f::from(p) - from_v;
            let ba = to_v - from_v;
            let t = pa.dot(ba) as f32 / ba.dot(ba) as f32;
            let h = t.clamp(0.0, 1.0);
            let dist = pa - (ba * h);
            let mut dist = Sd16::from(dist.norm() - thickness);
            *x = *x.min(&mut dist);
        }
    });
    Some(array)
}

#[inline(always)]
pub fn create_chunk_array(extent: Extent3i, default_sdf_value: Sd16) -> Array3x1<Sd16> {
    let size = (extent.shape.x() * extent.shape.y() * extent.shape.z()) as usize;
    let chunk: Vec<Sd16> = vec![default_sdf_value; size];
    Array3x1::new_one_channel(extent, chunk.into_boxed_slice())
}

#[allow(clippy::type_complexity)]
pub(crate) fn generate_mesh<T: 'static + Clone + Send + Sync + SignedDistance>(
    voxel_size: f32,
    map: &HashMapChunkTree3x1<T>,
) -> Result<
    (
        f32, // <- voxel_size
        Vec<(PosNormMesh, Extent3i)>,
    ),
    TBError,
> {
    let flat_shaded = true;
    let now = time::Instant::now();
    let chunk_meshes: Vec<(PosNormMesh, Extent3i)> = map
        .lod_storage(0)
        .keys()
        .par_bridge()
        .filter_map(move |chunk_min| {
            let padded_chunk_extent = padded_surface_nets_chunk_extent(
                &map.indexer.extent_for_chunk_with_min(*chunk_min),
            );
            let mut padded_chunk = Array3x1::fill(padded_chunk_extent, map.ambient_value());
            copy_extent(&padded_chunk_extent, &map.lod_view(0), &mut padded_chunk);

            let mut surface_nets_buffer = SurfaceNetsBuffer::default();
            // do the voxel_size multiplication later, vertices pos. needs to match extent.
            surface_nets(
                &padded_chunk,
                &padded_chunk_extent,
                1.0,
                !flat_shaded,
                &mut surface_nets_buffer,
            );

            if surface_nets_buffer.mesh.indices.is_empty() {
                None
            } else {
                Some((surface_nets_buffer.mesh, padded_chunk_extent))
            }
        })
        .collect();

    println!("surface_nets() duration: {:?}", now.elapsed());
    Ok((voxel_size, chunk_meshes))
}

/// Build the return model
pub(crate) fn build_output_bp_model(
    pb_model_name: String,
    pb_world: Option<PB_Matrix4x432>,
    voxel_size: f32,
    meshes: Vec<(PosNormMesh, Extent3i)>,
) -> Result<PB_Model, TBError> {
    // calculate the maximum required v&f capacity
    let (vertex_capacity, face_capacity) =
        meshes.iter().fold((0_usize, 0_usize), |(v, f), chunk| {
            (v + chunk.0.positions.len(), f + chunk.0.indices.len())
        });
    if vertex_capacity >= u32::MAX as usize {
        return Err(TBError::Overflow(format!("Generated mesh contains too many vertices to be referenced by u32: {}. Reduce the resolution.",vertex_capacity)));
    }

    if face_capacity >= u32::MAX as usize {
        return Err(TBError::Overflow(format!("Generated mesh contains too many faces to be referenced by u32: {}. Reduce the resolution.",vertex_capacity)));
    }

    let mut pb_vertices: Vec<PB_Vertex> = Vec::with_capacity(vertex_capacity);
    let mut pb_faces: Vec<u32> = Vec::with_capacity(face_capacity);
    // translates between bit-perfect copies of vertices and indices of already know vertices.
    let mut unique_vertex_map: ahash::AHashMap<(u32, u32, u32), u32> = ahash::AHashMap::default();
    // translates between the index used by the chunks + indices_offset and the vertex index in pb_vertices
    let mut vertex_map: ahash::AHashMap<u32, u32> = ahash::AHashMap::default();

    let now = time::Instant::now();
    for (mesh, extent) in meshes.iter() {
        // each chunk starts counting vertices from zero
        let indices_offset = pb_vertices.len() as u32;
        // vertices this far inside a chunk should (probably?) not be used outside this chunk.
        let deep_inside_extent = Extent3f::from_min_and_shape(
            Point3f::from(extent.minimum),
            Point3f::from(extent.shape),
        )
        .padded(-1.5);
        for (pi, p) in mesh.positions.iter().enumerate() {
            let p: Point3f = PointN(*p);
            if !deep_inside_extent.contains(p) {
                // only use vertex de-duplication if the vertex was close to the edges
                // of the extent
                let key = transmute_to_u32(&p.0);
                let _ = vertex_map.insert(
                    pi as u32 + indices_offset,
                    *unique_vertex_map.entry(key).or_insert_with(|| {
                        let n = pb_vertices.len() as u32;
                        pb_vertices.push(PB_Vertex {
                            x: (voxel_size * p.x()),
                            y: (voxel_size * p.y()),
                            z: (voxel_size * p.z()),
                        });
                        n
                    }),
                );
            } else {
                // vertex found deep inside chunk, skip vertex de-duplication.
                let _ = vertex_map.insert(pi as u32 + indices_offset, {
                    let n = pb_vertices.len() as u32;
                    pb_vertices.push(PB_Vertex {
                        x: (voxel_size * p.x()),
                        y: (voxel_size * p.y()),
                        z: (voxel_size * p.z()),
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

    println!(
        "Vertex de-duplication and return model packaging duration: {:?}",
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
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!(
        r#"________________________   ____                 .__
\______   \______   \   \ /   /______  ___ ____ |  |
 |    |  _/|    |  _/\   Y   /  _ \  \/  // __ \|  |
 |    |   \|    |   \ \     (  <_> >    <\  ___/|  |__
 |______  /|______  /  \___/ \____/__/\_ \\___  >____/
        \/        \/                    \/    \/"#
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
    Ok(reply)
}
