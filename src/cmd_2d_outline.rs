use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex as PB_Vertex;
use crate::TBError;
use cgmath::EuclideanSpace;
use cgmath::UlpsEq;
use itertools::Itertools;
use linestring::linestring_3d;
use std::collections::HashMap;

#[inline(always)]
/// make a key from v0 and v1, lowest index will always be first
fn make_edge_key(v0: usize, v1: usize) -> (usize, usize) {
    if v0 < v1 {
        (v0, v1)
    } else {
        (v1, v0)
    }
}

#[allow(clippy::type_complexity)]
/// remove internal edges from the input model
pub fn remove_internal_edges(
    obj: &PB_Model,
) -> Result<(Vec<(usize, usize)>, Vec<PB_Vertex>), TBError> {
    let mut all_edges = ahash::AHashSet::<(usize, usize)>::default();
    let mut single_edges = ahash::AHashSet::<(usize, usize)>::default();
    let mut internal_edges = ahash::AHashSet::<(usize, usize)>::default();
    //println!("Input faces : {:?}", obj.faces);

    let mut aabb = linestring_3d::Aabb3::<f64>::default();
    for v in obj.vertices.iter() {
        aabb.update_point(&cgmath::Point3::new(v.x as f64, v.y as f64, v.z as f64))
    }
    let plane =
        linestring_3d::Plane::get_plane_relaxed(&aabb, super::EPSILON, f64::default_max_ulps()).ok_or_else(|| {
            let aabbe_d = aabb.get_high().unwrap() - aabb.get_low().unwrap();
            let aabbe_c = (aabb.get_high().unwrap().to_vec() + aabb.get_low().unwrap().to_vec())/2.0;
            TBError::InputNotPLane(format!(
                "Input data not in one plane and/or plane not intersecting origin: Δ({},{},{}) C({},{},{})",
                aabbe_d.x, aabbe_d.y, aabbe_d.z,aabbe_c.x, aabbe_c.y, aabbe_c.z
            ))
        })?;

    println!("2d_outline: data was in plane:{:?} aabb:{:?}", plane, aabb);

    for face in obj.faces.iter() {
        if face.vertices.len() == 2 {
            let key = make_edge_key(
                *face.vertices.first().unwrap() as usize,
                *face.vertices.last().unwrap() as usize,
            );
            let _ = single_edges.insert(key);
            continue;
        }
        for (v0, v1) in face
            .vertices
            .iter()
            .chain(face.vertices.first())
            .tuple_windows::<(_, _)>()
        {
            let v0 = *v0 as usize;
            let v1 = *v1 as usize;
            if v0 == v1 {
                return Err(TBError::InvalidInputData(
                    "A face contained the same vertex at least twice".to_string(),
                ));
            }
            let key = make_edge_key(v0, v1);

            if all_edges.contains(&key) {
                let _ = internal_edges.insert(key);
            } else {
                let _ = all_edges.insert(key);
            }
        }
    }

    println!("Input vertices : {:?}", obj.vertices.len());
    println!("Input internal edges: {:?}", internal_edges.len());
    println!("Input all edges: {:?}", all_edges.len());
    /*println!("Vertices: ");
    for (n, v) in obj.vertices.iter().enumerate() {
        println!("#{}, {:?}", n, v);
    }

    println!("All edges pre: ");
    for (n, v) in all_edges.iter().enumerate() {
        println!("#{}, {:?}", n, v);
    }
    println!("single_edges pre: ");
    for (n, v) in single_edges.iter().enumerate() {
        println!("#{}, {:?}", n, v);
    }
    println!("internal_edges edges: ");
    for (n, v) in internal_edges.iter().enumerate() {
        println!("#{}, {:?}", n, v);
    }*/
    #[cfg(feature = "hash_drain_filter")]
    {
        let _ = all_edges.drain_filter(|x| internal_edges.contains(x));
    }
    #[cfg(not(feature = "hash_drain_filter"))]
    {
        // inefficient version of drain_filter for +stable
        let kept_edges = all_edges
            .into_iter()
            .filter(|x| !internal_edges.contains(x))
            .collect();
        all_edges = kept_edges;
    }
    for e in single_edges.into_iter() {
        let _ = all_edges.insert(e);
    }

    /*println!("All edges post: ");
    for (n, v) in all_edges.iter().enumerate() {
        println!("#{}, {:?}", n, v);
    }*/
    /*println!("Input all edges post filter: {:?}", all_edges.len());
    println!();
    */
    // all_edges should now contain the outline and none of the internal edges.
    // no need for internal_edges any more
    drop(internal_edges);
    // vector number translation table
    let mut vector_rename_map = ahash::AHashMap::<usize, usize>::default();
    let mut rv_vertices = Vec::<PB_Vertex>::with_capacity(all_edges.len() * 6 / 5);
    let mut rv_lines = Vec::<(usize, usize)>::with_capacity(all_edges.len() * 6 / 5);

    // Iterate over each edge and store each used vertex (in no particular order)
    for (v0, v1) in all_edges.into_iter() {
        let v0 = if let Some(v0) = vector_rename_map.get(&v0) {
            *v0
        } else {
            let translated = (v0, rv_vertices.len());
            let _ = vector_rename_map.insert(translated.0, translated.1);
            let vtmp = &obj.vertices[v0];
            rv_vertices.push(PB_Vertex {
                x: vtmp.x,
                y: vtmp.y,
                z: vtmp.z,
            });
            translated.1
        };
        let v1 = if let Some(v1) = vector_rename_map.get(&v1) {
            *v1
        } else {
            let translated = (v1, rv_vertices.len());
            let _ = vector_rename_map.insert(translated.0, translated.1);
            let vtmp = &obj.vertices[v1];
            rv_vertices.push(PB_Vertex {
                x: vtmp.x,
                y: vtmp.y,
                z: vtmp.z,
            });
            translated.1
        };
        // v0 and v1 now contains the translated vertex indices.
        rv_lines.push((v0, v1));
    }
    println!("Output edges: {:?}", rv_lines.len());
    println!("Output vertices: {:?}", rv_vertices.len());

    Ok((rv_lines, rv_vertices))
}

pub fn command(
    a_command: &PB_Command,
    _options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!(
        r#"________  ________   ________          __  .__  .__               
\_____  \ \______ \  \_____  \  __ ___/  |_|  | |__| ____   ____  
 /  ____/  |    |  \  /   |   \|  |  \   __\  | |  |/    \_/ __ \ 
/       \  |    `   \/    |    \  |  /|  | |  |_|  |   |  \  ___/ 
\_______ \/_______  /\_______  /____/ |__| |____/__|___|  /\___  >
        \/        \/         \/                         \/     \/ "#
    );
    if a_command.models.len() > 1 {
        return Err(TBError::InvalidInputData(
            "This operation only supports one model as input".to_string(),
        ));
    }

    for model in a_command.models.iter() {
        println!("model.name:{:?}, ", model.name);
        println!("model.vertices:{:?}, ", model.vertices.len());
        println!("model.faces:{:?}, ", model.faces.len());
        println!(
            "model.world_orientation:{:?}, ",
            model.world_orientation.as_ref().map_or(0, |_| 16)
        );
        println!();
    }
    if !a_command.models.is_empty() {
        let input_model = &a_command.models[0];
        let (rv_lines, rv_vector) = remove_internal_edges(input_model)?;
        let mut reply = PB_Reply {
            options: vec![PB_KeyValuePair {
                key: "ONLY_EDGES".to_string(),
                value: "True".to_string(),
            }],
            models: Vec::with_capacity(1),
            models32: Vec::with_capacity(0),
        };

        let mut model = PB_Model {
            name: a_command.models[0].name.clone(),
            world_orientation: input_model.world_orientation.clone(),
            vertices: Vec::<PB_Vertex>::new(),
            faces: Vec::<PB_Face>::new(),
        };
        for v0 in rv_vector.into_iter() {
            model.vertices.push(v0);
        }
        for l in rv_lines.iter() {
            let face = vec![l.0 as u64, l.1 as u64];
            model.faces.push(PB_Face { vertices: face });
        }

        reply.models.push(model);
        Ok(reply)
    } else {
        Err(TBError::InvalidInputData(
            "Model did not contain any data".to_string(),
        ))
    }
}
