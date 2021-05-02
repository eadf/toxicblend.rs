use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex as PB_Vertex;
use building_blocks::core::prelude::*;
use itertools::Itertools;
use linestring::cgmath_3d;
use std::collections::HashMap;
//use building_blocks::core::point::*;
use building_blocks::core::prelude::PointN;
use building_blocks::core::sdfu::{self, SDF};
use building_blocks::mesh::*;
use building_blocks::storage::prelude::*;
//use rayon::prelude::*;

/// Simplify the model into a sequence of connected vertices.
/// The end and start points has duplicate-checked vertex numbers, the 'in the middle' vertexes do not.
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
            return Err(TBError::ModelContainsFaces("Model can't contain any faces, only edges. Use the 2d_outline tool to remove faces".to_string()));
        }
        if face.vertices.len() < 2 {
            return Err(TBError::InvalidInputData(
                "Edge containing none or only one vertex".to_string(),
            ));
        }
        let v0 = face.vertices[0] as usize;
        let v1 = face.vertices[1] as usize;

        {
            let entry = connections_map
                .entry(v0)
                .or_insert_with(smallvec::SmallVec::<[usize; 2]>::new);
            if !entry.contains(&v1) {
                entry.push(v1);
            }

            let entry = connections_map
                .entry(v1)
                .or_insert_with(smallvec::SmallVec::<[usize; 2]>::new);
            if !entry.contains(&v0) {
                entry.push(v0);
            }
        }
        let _ = edge_set.insert(make_key(v0, v1));
    }
    // we now know how many times one vertex is connected to other vertices
    println!("Built connections_map.len():{}", connections_map.len());
    //println!("connections_map: {:?}", connections_map);

    // connections_map is no longer mutable
    let connections_map = connections_map;

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
                let (end_vertex, rv) =
                    walk_single_line_edges(v0, v1, &mut edge_set, &connections_map, &obj.vertices)?;
                //println!("got a linestring of length: {:?}", rv.len());
                linestrings.push((v0, rv, end_vertex));
            }
        }
    }
    println!(
        "done with the simple edges! connections_map.len():{} edge_set.len():{} linestrings.len():{}",
        connections_map.len(),
        edge_set.len(),
        linestrings.len()
    );
    /*
    for ls in linestrings.iter() {
        println!("ls: {:?} {:?} {:?}", ls.0, ls.1.len(), ls.2);
    }*/

    let mut something_changed: bool = true;
    while (!edge_set.is_empty()) && something_changed {
        something_changed = false;
        if let Some(suggested_edge) = edge_set
            .iter()
            .find(|x| connections_map.get(&x.0).map_or(0, |y| y.len()) == 2)
        {
            something_changed = true;
            let v0 = suggested_edge.0;
            let v1 = suggested_edge.1;
            let (end_vertex, rv) =
                walk_single_line_edges(v0, v1, &mut edge_set, &connections_map, &obj.vertices)?;

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
    /*for ls in linestrings.iter() {
        println!("ls: {:?} {:?} {:?}", ls.0, ls.1.len(), ls.2);
    }*/
    Ok(linestrings)
}

#[inline(always)]
/// make a key from v0 and v1, lowest index first
fn make_key(v0: usize, v1: usize) -> (usize, usize) {
    if v0 < v1 {
        (v0, v1)
    } else {
        (v1, v0)
    }
}

/// fill the 'result' with a sequence of connected vertices
fn walk_single_line_edges(
    from_v: usize,
    to_v: usize,
    edge_set: &mut fnv::FnvHashSet<(usize, usize)>,
    connections_map: &fnv::FnvHashMap<usize, smallvec::SmallVec<[usize; 2]>>,
    vertices: &[PB_Vertex],
) -> Result<(usize, cgmath_3d::LineString3<f64>), TBError> {
    let started_at_vertex = from_v;
    let mut kill_pill = edge_set.len() + 2;

    let mut from_v = from_v;
    let mut to_v = to_v;
    let mut end_vertex = from_v;

    //let mut first_loop = true;
    //let end_key = make_key(from_v, to_v);

    let mut result = cgmath_3d::LineString3::<f64>::default();
    /*
    println!(
        "--> from:{}, to:{}",
        from_v,
        to_v,
    );*/
    {
        let v0_value = vertices.get(from_v).ok_or_else(|| {
            TBError::InternalError(format!("Lost an vertex somehow.. vertex index:{}", from_v))
        })?;
        result.push(cgmath::Point3 {
            x: v0_value.x,
            y: v0_value.y,
            z: v0_value.z,
        });
        //println!("#1 Pushing vertex {:?}, {}->{}", from_v, from_v, to_v);
    }

    'outer: loop {
        if to_v == started_at_vertex {
            //println!("to_v == started_at_vertex == {}", started_at_vertex);
            end_vertex = to_v;
            //println!("<-- meta recursion ended #1 size={}", result.len());
            break 'outer;
        }
        kill_pill -= 1;
        if kill_pill == 0 {
            return Err(TBError::InternalError("Detected infinite loop".to_string()));
        }
        if from_v == to_v {
            return Err(TBError::InternalError(
                "Detected from_v == to_v".to_string(),
            ));
        }

        let key = make_key(from_v, to_v);

        if edge_set.contains(&key) {
            let v1 = vertices.get(to_v).ok_or_else(|| {
                TBError::InternalError(format!("Lost a vertex somehow.. index:{}", to_v))
            })?;
            result.push(cgmath::Point3 {
                x: v1.x,
                y: v1.y,
                z: v1.z,
            });
            let _ = edge_set.remove(&key);
            //println!("#2 Pushing vertex {:?}, {}->{} dropping:{:?}", to_v, from_v, to_v, key);

            let connection = connections_map
                .get(&to_v)
                .ok_or_else(|| TBError::InternalError("edge ended unexpectedly #2".to_string()))?;
            // todo: remove this sanity test when stable
            if !connection.contains(&from_v) {
                return Err(TBError::InternalError(format!(
                    "edge did not contain 'from' vertex??? {} {:?}",
                    from_v, connection
                )));
            }
            if connection.len() == 2 {
                // todo: I think it would be faster to just test [0] and [1] instead of using find()
                let next_v = *connection.iter().find(|x| **x != from_v).ok_or_else(|| {
                    TBError::InternalError("edge ended unexpectedly #1".to_string())
                })?;

                from_v = to_v;
                to_v = next_v;

                continue 'outer;
            } else {
                //println!("#1 breaking at to_v={}", to_v);
                end_vertex = to_v;
                break 'outer;
            }
        }
        break 'outer;
    }

    // Push the final vertex

    let end_vertex_value = vertices.get(end_vertex).ok_or_else(|| {
        TBError::InternalError(format!(
            "Lost a vertex somehow.. vertex index:{}",
            end_vertex
        ))
    })?;
    let result_point_last = result.points().last().unwrap();
    if !(result_point_last.x == end_vertex_value.x
        && result_point_last.y == end_vertex_value.y
        && result_point_last.z == end_vertex_value.z)
    {
        result.push(cgmath::Point3 {
            x: end_vertex_value.x,
            y: end_vertex_value.y,
            z: end_vertex_value.z,
        });
    }
    if end_vertex == started_at_vertex {
        //let key = make_key(from_v,to_v);
        //println!("#2 Pushing vertex {:?}, {}->{} dropping:{:?}", to_v, from_v, to_v, key);
        let _ = edge_set.remove(&make_key(from_v, to_v));
    } else {
        //println!("#3 Pushing vertex {:?}, {}->{} ", end_vertex, from_v, to_v);
    }
    //println!("<-- meta recursion ended @{} result.len()={} edge_set.len():{}", end_vertex, result.len(), edge_set.len());
    Ok((end_vertex, result))
}

/// Build the voxel data from the input
fn build_voxel(
    pb_model: &PB_Model,
    radius_multiplier: f64,
    lines: Vec<(usize, cgmath_3d::LineString3<f64>, usize)>,
) -> Result<PosNormMesh, TBError> {
    let mut aabb = linestring::cgmath_3d::Aabb3::<f64>::default();
    for v in pb_model.vertices.iter() {
        aabb.update_point(&cgmath::Point3::new(v.x, v.y, v.z))
    }
    let dimensions = aabb.get_high().unwrap() - aabb.get_low().unwrap();
    let dimensions = dimensions.x.max(dimensions.y).max(dimensions.z);
    let radius = dimensions * radius_multiplier;
    println!(
        "Voxelizing using tube radius. {} = {}*{}.",
        radius, dimensions, radius_multiplier
    );
    println!("lines:{:?}", lines);
    println!("aabb.high:{:?}", aabb.get_high().unwrap());
    println!("aabb.low:{:?}", aabb.get_low().unwrap());
    println!(
        "delta:{:?}",
        aabb.get_high().unwrap() - aabb.get_low().unwrap()
    );

    let thickness = (radius * 2.0) as f32;

    let mut sdfu_vec = Vec::<(
        PointN<[f32; 3]>,
        PointN<[f32; 3]>,
        Box<dyn Fn(Point3f) -> f32>,
    )>::new();
    if !lines.is_empty() {
        for (_, lines, _) in lines.into_iter() {
            //println!("lines.len():{}", lines.len());
            for (lf, lt) in lines.points().iter().tuple_windows::<(_, _)>() {
                //println!("{:?}->{:?}", line.start, line.end);
                let f: PointN<[f32; 3]> = PointN([lf.x as f32, lf.y as f32, lf.z as f32]);
                let t: PointN<[f32; 3]> = PointN([lt.x as f32, lt.y as f32, lt.z as f32]);
                //let sdf = sdfu::Line::new(f, t, 5.0);
                /*println!(
                    "({},{},{})->({},{},{})",
                    f.x(),
                    f.y(),
                    f.z(),
                    t.x(),
                    t.y(),
                    t.z()
                );*/
                //sdfus.push( (f,t,Box::new(move |p:Point3f|->f32 {sdfu::Line::new(f, t, thickness).dist(Point3f::from(p.yzx()))})));
                sdfu_vec.push((
                    f,
                    t,
                    Box::new(move |p: Point3f| -> f32 { sdfu::Line::new(f, t, thickness).dist(p) }),
                ));
            }
            //println!("end line");
        }
    }

    let union_sdf_func = move |p: Point3i| {
        let pf = Point3f::from(p);
        let sdfirst = sdfu_vec.first().unwrap();
        let mut min_dist = sdfirst.2(pf);
        for s in sdfu_vec.iter().skip(1) {
            let dist = s.2(pf);
            if dist < min_dist {
                min_dist = dist;
            }
        }
        Sd16::from(min_dist)
    };

    let extent = {
        let aabb_min = aabb.get_low().unwrap();
        let aabb_max = aabb.get_high().unwrap();
        Extent3i::from_min_and_max(
            PointN([aabb_min.x as i32, aabb_min.y as i32, aabb_min.z as i32]),
            PointN([aabb_max.x as i32, aabb_max.y as i32, aabb_max.z as i32]),
        ).padded(thickness as i32 + 2)
    };
    println!("extent:{:?}", extent);

    let samples = Array3x1::fill_with(extent, union_sdf_func);
    let mut mesh_buffer = SurfaceNetsBuffer::default();
    let voxel_size = 1.0;
    surface_nets(&samples, samples.extent(), voxel_size, &mut mesh_buffer);
    Ok(mesh_buffer.mesh)
}

/// Build the return model
/// lines contains a vector of (<first vertex index>,<a list of points><last vertex index>)
fn build_bp_model(a_command: &PB_Command, mesh: PosNormMesh) -> Result<PB_Model, TBError> {
    let input_pb_model = &a_command.models[0];

    let pb_vertices = mesh
        .positions
        .iter()
        .map(|[x, y, z]| PB_Vertex {
            x: *x as f64,
            y: *y as f64,
            z: *z as f64,
        })
        .collect();
    let pb_faces = mesh
        .indices
        .iter()
        .tuple_windows::<(_, _, _)>()
        .step_by(3)
        .map(|(a, b, c)| PB_Face {
            vertices: vec![*a as u64, *b as u64, *c as u64],
        })
        .collect();

    Ok(PB_Model {
        name: input_pb_model.name.clone(),
        world_orientation: input_pb_model.world_orientation.clone(),
        vertices: pb_vertices,
        faces: pb_faces,
    })
}

pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!("Voxel got command: \"{}\"", a_command.command);
    if a_command.models.len() > 1 {
        return Err(TBError::InvalidInputData(format!(
            "This operation only supports one model as input:{}",
            a_command.models.len()
        )));
    }
    if a_command.models.is_empty() {
        return Err(TBError::InvalidInputData(
            "Model did not contain any data".to_string(),
        ));
    }
    let radius = options
        .get("RADIUS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the RADIUS parameter".to_string()))?
        .parse::<f64>()
        .map_err(|_| {
            TBError::InvalidInputData("Could not parse the RADIUS parameter".to_string())
        })?
        / 100.0;

    for model in a_command.models.iter() {
        println!("model.name:{:?}, ", model.name);
        println!("model.vertices:{:?}, ", model.vertices.len());
        println!("model.faces:{:?}, ", model.faces.len());
        println!(
            "model.world_orientation:{:?}, ",
            model.world_orientation.as_ref().map_or(0, |_| 16)
        );
        println!("Radius:{:?}% multiplier ", radius);
        println!();
    }

    let lines = find_linestrings(&a_command.models[0])?;
    let mesh = build_voxel(&a_command.models[0], radius, lines)?;
    let model = build_bp_model(&a_command, mesh)?;

    let reply = PB_Reply {
        options: vec![PB_KeyValuePair {
            key: "ONLY_EDGES".to_string(),
            value: "False".to_string(),
        }],
        models: vec![model],
    };
    Ok(reply)
}
