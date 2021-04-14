use super::TBError;
use crate::toxicblend::Command as PB_Command;
use crate::toxicblend::Face as PB_Face;
use crate::toxicblend::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend::Model as PB_Model;
use crate::toxicblend::Reply as PB_Reply;
use crate::toxicblend::Vertex as PB_Vertex;
use linestring::cgmath_3d;
use rayon::prelude::*;
use std::collections::HashMap;

/// Simplify the model
pub fn find_linestrings(
    obj: &PB_Model,
) -> Result<Vec<(usize, cgmath_3d::LineString3<f64>, usize)>, TBError> {
    let mut linestrings = Vec::<(usize, cgmath_3d::LineString3<f64>, usize)>::new();

    // vertex number usize is connected to u32 number of other vertices.
    let mut connections_map = fnv::FnvHashMap::<usize, smallvec::SmallVec<[usize; 2]>>::default();
    // a set of every edge
    let mut edge_set = fnv::FnvHashSet::<(usize, usize)>::default();

    for face in obj.faces.iter() {
        if face.vertices.len() > 2 {
            return Err(TBError::ModelContainsFaces);
        }
        if face.vertices.len() > 2 {
            return Err(TBError::InvalidData(
                "Edge containing only one vertex".to_string(),
            ));
        }
        let v0 = *face.vertices.get(0).unwrap() as usize;
        let v1 = *face.vertices.get(1).unwrap() as usize;

        #[allow(clippy::or_fun_call)]
        {
            let entry = connections_map
                .entry(v0)
                .or_insert(smallvec::SmallVec::<[usize; 2]>::new());
            if !entry.contains(&v1) {
                entry.push(v1);
            }
        }
        #[allow(clippy::or_fun_call)]
        {
            let entry = connections_map
                .entry(v1)
                .or_insert(smallvec::SmallVec::<[usize; 2]>::new());
            if !entry.contains(&v0) {
                entry.push(v0);
            }
        }
        edge_set.insert(make_key(v0, v1));
    }
    // we now know how many times one vertex is connected to other vertices
    //println!("connections_map.len():{}", connections_map.len());
    //println!("connections_map: {:?}", connections_map);

    // connections_map is no longer mutable
    let connections_map = connections_map;
    // the length of connections_map does not change over this loop
    //for v0 in 0..connections_map.len() {
    for connection in connections_map.iter() {
        let v0 = *connection.0;

        let neighbours = connection.1;
        // start looking for vertices with more than 2 connections, or is exactly 1
        // i.e. it is connected to at least three vertices and is therefore a valid start point
        if (neighbours.len() <= 2) && (neighbours.len() > 1) {
            continue;
        }

        /*println!(
            "vertex:{} has ngb:{:?} ngb.len():{}",
            v0,
            neighbours,
            neighbours.len()
        );*/

        for v1 in neighbours.iter().filter(|x| **x != v0) {
            //println!("testing {}->{}", v0, v1);

            let v1 = *v1;
            let key = make_key(v0, v1);
            if edge_set.contains(&key) {
                let mut rv = cgmath_3d::LineString3::<f64>::default();
                if let Some(v0_value) = obj.vertices.get(v0) {
                    rv.push(cgmath::Point3 {
                        x: v0_value.x as f64,
                        y: v0_value.y as f64,
                        z: v0_value.z as f64,
                    });
                } else {
                    return Err(TBError::InternalError(format!(
                        "Lost an vertex somehow.. index:{}",
                        v0
                    )));
                }
                //println!("recursion started at {} -> {}", v0, v1);

                let mut end_vertex = v0;
                traverse_edges(
                    v0,
                    v1,
                    &mut edge_set,
                    &connections_map,
                    &obj.vertices,
                    &mut rv,
                    &mut end_vertex,
                )?;
                //println!("got a linestring of length: {:?}", rv.len());
                linestrings.push((v0, rv, end_vertex));
            }
        }
    }
    /*println!(
        "done with the simple edges! connections_map.len():{} edge_set.len():{} linestrings.len():{}",
        connections_map.len(),
        edge_set.len(),
        linestrings.len()
    );*/
    let mut something_changed: bool = true;
    while !edge_set.is_empty() && something_changed {
        something_changed = false;
        if let Some(suggested_edge) = edge_set
            .iter()
            .find(|x| connections_map.get(&x.0).map_or(0, |y| y.len()) == 2)
        {
            something_changed = true;
            let mut rv = cgmath_3d::LineString3::<f64>::default();
            if let Some(v0_value) = obj.vertices.get(suggested_edge.0) {
                rv.push(cgmath::Point3 {
                    x: v0_value.x as f64,
                    y: v0_value.y as f64,
                    z: v0_value.z as f64,
                });
            } else {
                return Err(TBError::InternalError(format!(
                    "Lost an vertex somehow.. index:{}",
                    suggested_edge.0
                )));
            }
            //println!("recursion forced at {} -> {}", suggested_edge.0, suggested_edge.1);
            let v0 = suggested_edge.0;
            let v1 = suggested_edge.1;
            let mut end_vertex = v0;

            traverse_edges(
                v0,
                v1,
                &mut edge_set,
                &connections_map,
                &obj.vertices,
                &mut rv,
                &mut end_vertex,
            )?;
            //println!("got a linestring of length: {:?}", rv.len());
            linestrings.push((v0, rv, end_vertex));
        }
    }
    println!(
        "done! connections_map.len():{} edge_set.len():{} linestrings.len():{}",
        connections_map.len(),
        edge_set.len(),
        linestrings.len()
    );
    if !edge_set.is_empty() {
        return Err(TBError::InternalError(format!(
            "Could not convert all edges to lines:{:?}",
            edge_set
        )));
    }
    Ok(linestrings)
}

#[inline(always)]
/// make a key from v0 and v1, lowest index will always be first
fn make_key(v0: usize, v1: usize) -> (usize, usize) {
    if v0 < v1 {
        (v0, v1)
    } else {
        (v1, v0)
    }
}

fn traverse_edges(
    from_v: usize,
    to_v: usize,
    edge_set: &mut fnv::FnvHashSet<(usize, usize)>,
    connections_map: &fnv::FnvHashMap<usize, smallvec::SmallVec<[usize; 2]>>,
    vertices: &[PB_Vertex],
    result: &mut cgmath_3d::LineString3<f64>,
    end_vertex: &mut usize,
) -> Result<(), TBError> {
    let key = make_key(from_v, to_v);
    /*println!(
        "->traverse_edges {}->{} {:?} {:?}",
        from_v,
        to_v,
        connections_map.get(&from_v),
        connections_map.get(&to_v)
    );*/
    if edge_set.contains(&key) {
        if let Some(v1) = vertices.get(to_v) {
            result.push(cgmath::Point3 {
                x: v1.x as f64,
                y: v1.y as f64,
                z: v1.z as f64,
            });
            edge_set.remove(&key);
            //println!("Removing edge {:?}", key);
        } else {
            return Err(TBError::InternalError(format!(
                "Lost an vertex somehow.. index:{}",
                to_v
            )));
        }
        if let Some(connection) = connections_map.get(&to_v) {
            if !connection.contains(&from_v) {
                return Err(TBError::InternalError(format!(
                    "edge did not contain from vertex??? {} {:?}",
                    from_v, connection
                )));
            }
            if connection.len() == 2 {
                if let Some(next_v0) = connection.iter().find(|x| **x != from_v) {
                    /*println!(
                        "recursion continues at {}, connection:{:?}",
                        to_v, connection
                    );*/
                    traverse_edges(
                        to_v,
                        *next_v0,
                        edge_set,
                        connections_map,
                        vertices,
                        result,
                        end_vertex,
                    )?;
                } else {
                    return Err(TBError::InternalError(
                        "edge ended unexpectedly #2".to_string(),
                    ));
                }
            } else {
                //println!("recursion ended at {} {:?}", to_v, connection);
                *end_vertex = to_v;
                return Ok(());
            }
        } else {
            return Err(TBError::InternalError(
                "edge ended unexpectedly #1".to_string(),
            ));
        }
    } /*else {
          return Err(TBError::InternalError(format!(
              "Edge set did not contain suggested edge {}->{} key:{:?}",
              from_v, to_v, key
          )));
      }*/
    Ok(())
}

/// Simplify the line-strings using the Ramer–Douglas–Peucker algorithm
pub fn centerline(
    pb_model: &PB_Model,
    angle: f64,
    lines: Vec<(usize, cgmath_3d::LineString3<f64>, usize)>,
) -> Result<Vec<(usize, cgmath_3d::LineString3<f64>, usize)>, TBError> {
    let mut aabb = linestring::cgmath_3d::Aabb3::<f64>::default();
    for v in pb_model.vertices.iter() {
        aabb.update_point(&cgmath::Point3::new(v.x as f64, v.y as f64, v.z as f64))
    }

    println!("Centerlining using Angle={},", angle);

    let rv: Vec<(usize, cgmath_3d::LineString3<f64>, usize)> = lines
        .into_par_iter()
        .map(|(v0, ls, v1)| (v0, ls.simplify(angle), v1))
        .collect();
    Ok(rv)
}

/// Build the return model
pub fn build_bp_model(
    a_command: &PB_Command,
    lines: Vec<(usize, cgmath_3d::LineString3<f64>, usize)>,
) -> Result<PB_Model, TBError> {
    let input_pb_model = &a_command.models[0];
    let mut output_pb_model = PB_Model {
        name: input_pb_model.name.clone(),
        world_orientation: input_pb_model.world_orientation.clone(),
        vertices: Vec::<PB_Vertex>::new(),
        faces: Vec::<PB_Face>::new(),
    };

    // map between old and new vertex number
    let mut v_map = fnv::FnvHashMap::<usize, usize>::default();

    for ls in lines.into_iter() {
        let v0 = ls.0;
        let l = ls.1;
        let v1 = ls.2;
        let v0_mapped = *v_map.entry(v0).or_insert(output_pb_model.vertices.len());
        if v0_mapped == output_pb_model.vertices.len() {
            output_pb_model
                .vertices
                .push(input_pb_model.vertices[v0].clone());
        }
        output_pb_model.faces.push(PB_Face {
            vertices: vec![v0_mapped as u64, (v0_mapped + 1) as u64],
        });
        let mut last_p = v0_mapped;
        for p in l.points().iter().skip(1).take(l.points().len() - 2) {
            output_pb_model.faces.push(PB_Face {
                vertices: vec![last_p as u64, output_pb_model.vertices.len() as u64],
            });
            last_p = output_pb_model.vertices.len();
            output_pb_model.vertices.push(PB_Vertex {
                x: p.x as f64,
                y: p.y as f64,
                z: p.z as f64,
            });
        }
        let v1_mapped = *v_map.entry(v1).or_insert(output_pb_model.vertices.len());
        if v1_mapped == output_pb_model.vertices.len() {
            output_pb_model
                .vertices
                .push(input_pb_model.vertices[v1].clone());
        }
        output_pb_model.faces.push(PB_Face {
            vertices: vec![last_p as u64, v1_mapped as u64],
        });
    }
    Ok(output_pb_model)
    //return Err(TBError::InvalidData(format!("not implemented")));
}

pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!("centerline got command: {}", a_command.command);
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
    if a_command.models.is_empty() {
        return Err(TBError::InvalidData(
            "Model did not contain any data".to_string(),
        ));
    }
    let angle = options
        .get("CENTERLINE_ANGLE")
        .ok_or_else(||TBError::InvalidData(
            "Missing the CENTERLINE_ANGLE parameter".to_string(),
        ))?
        .parse::<f64>()
        .map_err(|_|TBError::InvalidData(
            "Could not parse the CENTERLINE_ANGLE parameter".to_string(),
        ))?;

    let lines = find_linestrings(&a_command.models[0])?;
    let lines = centerline(&a_command.models[0], angle, lines)?;
    let model = build_bp_model(&a_command, lines)?;

    let mut reply = PB_Reply {
        options: vec![PB_KeyValuePair {
            key: "ONLY_EDGES".to_string(),
            value: "True".to_string(),
        }],
        models: Vec::<PB_Model>::new(),
    };

    reply.models.push(model);
    Ok(reply)
}
