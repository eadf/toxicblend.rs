use super::TBError;
use crate::toxicblend::Command as PB_Command;
use crate::toxicblend::Face as PB_Face;
use crate::toxicblend::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend::Model as PB_Model;
use crate::toxicblend::Reply as PB_Reply;
use crate::toxicblend::Vertex as PB_Vertex;
use std::collections::HashMap;

#[allow(clippy::type_complexity)]
/// remove internal edges from the input model
pub fn remove_internal_edges(
    obj: &PB_Model,
) -> Result<(Vec<(usize, usize)>, Vec<PB_Vertex>), TBError> {
    let mut all_edges = fnv::FnvHashSet::<(usize, usize)>::default();
    let mut internal_edges = fnv::FnvHashSet::<(usize, usize)>::default();
    //println!("Input faces : {:?}", obj.faces);

    for f in obj.faces.iter() {

        let mut i0 = f.vertices.iter();
        for v1 in f.vertices.iter().skip(1).chain(f.vertices.first()) {
            let v0 = (*i0.next().unwrap()) as usize;
            let v1 = (*v1) as usize;
            //print!("{:?}->{:?},", v0, v1);
            let key = if v0 < v1 {(v0 as usize, v1 as usize)} else {(v1 as usize, v0 as usize)};
            if all_edges.contains(&key) {
                let _ = internal_edges.insert(key);
            } else {
                let _ = all_edges.insert(key);
            }
        }
        //println!();
    }

    println!("Input vertices : {:?}", obj.vertices.len());
    println!("Input internal edges: {:?}", internal_edges.len());
    println!("Input all edges: {:?}", all_edges.len());
    println!();

    //println!("Vertices: {:?}", obj.positions);
    let _ = all_edges.drain_filter(|x| internal_edges.contains(x));
    // all_edges should now contain the outline and none of the internal edges.
    // no need for internal_edges any more
    drop(internal_edges);
    // vector number translation table
    let mut vector_rename_map = fnv::FnvHashMap::<usize, usize>::default();
    let mut rv_vertices = Vec::<PB_Vertex>::new();
    let mut rv_lines = Vec::<(usize, usize)>::new();

    // Iterate over each edge and store the each used vertex (in no particular order)
    for e in all_edges.iter() {
        let v0 = e.0;
        let v1 = e.1;
        let v0 = if let Some(v0) = vector_rename_map.get(&v0) {
            *v0
        } else {
            let translated = (v0, rv_vertices.len());
            vector_rename_map.insert(translated.0, translated.1);
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
            vector_rename_map.insert(translated.0, translated.1);
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

pub fn command(a_command: &PB_Command, _options:HashMap<String, String>) -> Result<PB_Reply, TBError> {
    println!("2d_outline got command: {}", a_command.command);
    if a_command.models.len() > 1 {
        return Err(TBError::InvalidInputData("This operation only supports one model as input".to_string()))
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
        let (rv_lines, rv_vector) = remove_internal_edges(&input_model)?;
        let mut reply = PB_Reply {
            options: vec![PB_KeyValuePair {
                key: "ONLY_EDGES".to_string(),
                value: "True".to_string(),
            }],
            models: Vec::<PB_Model>::new(),
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
            "Model did not contain any data".to_string()
        ))
    }
}
