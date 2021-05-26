use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex as PB_Vertex;
use cgmath::EuclideanSpace;
use cgmath::UlpsEq;
use itertools::Itertools;
use linestring::cgmath_3d::Plane;
use std::collections::HashMap;

/// converts from a private, comparable and hashable format
/// only use this for floats that are f64::is_finite()
#[inline(always)]
fn transmute_to_f64(a: &(u64, u64)) -> cgmath::Point2<f64> {
    cgmath::Point2 {
        x: f64::from_bits(a.0),
        y: f64::from_bits(a.1),
    }
}

/// converts to a private, comparable and hashable format
/// only use this for floats that are f64::is_finite()
#[inline(always)]
fn transmute_to_u64(a: cgmath::Point2<f64>) -> (u64, u64) {
    (a.x.to_bits(), a.y.to_bits())
}

/// detect self intersections and cut those lines at the intersection
fn knife_intersect(input_pb_model: &PB_Model) -> Result<PB_Model, TBError> {
    let mut aabb = linestring::cgmath_3d::Aabb3::<f64>::default();
    for v in input_pb_model.vertices.iter() {
        aabb.update_point(&cgmath::Point3::new(v.x as f64, v.y as f64, v.z as f64))
    }

    let plane = Plane::get_plane_relaxed(&aabb, super::EPSILON, f64::default_max_ulps()).ok_or_else(|| {
        let aabbe_d = aabb.get_high().unwrap() - aabb.get_low().unwrap();
        let aabbe_c = (aabb.get_high().unwrap().to_vec() + aabb.get_low().unwrap().to_vec())/2.0;
        TBError::InputNotPLane(format!(
            "Input data not in one plane and/or plane not intersecting origin: Δ({},{},{}) C({},{},{})",
            aabbe_d.x, aabbe_d.y, aabbe_d.z,aabbe_c.x, aabbe_c.y, aabbe_c.z
        ))
    })?;
    println!(
        "knife_intersect: data was in plane:{:?} aabb:{:?}",
        plane, aabb
    );
    //println!("input Lines:{:?}", input_pb_model.vertices);

    let vertices_2d: Vec<cgmath::Point2<f64>> = match plane {
        Plane::XY => input_pb_model
            .vertices
            .iter()
            .map(|v| cgmath::Point2 {
                x: v.x as f64,
                y: v.y as f64,
            })
            .collect(),
        Plane::XZ => input_pb_model
            .vertices
            .iter()
            .map(|v| cgmath::Point2 {
                x: v.x as f64,
                y: v.z as f64,
            })
            .collect(),
        Plane::ZY => input_pb_model
            .vertices
            .iter()
            .map(|v| cgmath::Point2 {
                x: v.z as f64,
                y: v.y as f64,
            })
            .collect(),
    };

    let mut lines =
        Vec::<linestring::cgmath_2d::Line2<f64>>::with_capacity(input_pb_model.faces.len());

    // todo, the enumeration is never used
    for f in input_pb_model.faces.iter().enumerate() {
        match f.1.vertices.len() {
            3..=usize::MAX => return Err(TBError::ModelContainsFaces("Model can't contain any faces, only edges. Use the 2d_outline tool to remove faces".to_string())),
            2 => lines.push(linestring::cgmath_2d::Line2 {
                start: vertices_2d[f.1.vertices[0] as usize],
                end: vertices_2d[f.1.vertices[1] as usize],
            }),
            _ => (),
        }
    }
    //println!("Lines:{:?}", lines);
    let mut edge_split = ahash::AHashMap::<usize, smallvec::SmallVec<[(u64, u64); 1]>>::default();
    let mut edge_is_split = yabf::Yabf::with_capacity(input_pb_model.faces.len());
    {
        let intersection_result =
            linestring::cgmath_2d::intersection::IntersectionData::<f64>::default()
                .with_ignore_end_point_intersections(true)?
                .with_stop_at_first_intersection(false)?
                .with_lines(lines.into_iter())?
                .compute()?;
        if intersection_result.len() == 0 {
            println!("No intersections detected!!");
        }
        for (p, l) in intersection_result {
            println!("Intersection detected @{:?} Involved lines:{:?}", p, l);
            for e in l.iter() {
                edge_is_split.set_bit(*e, true);
                if !p.x.is_finite() || !p.x.is_finite() {
                    return Err(TBError::InternalError(format!(
                        "The found intersection is not valid: x:{:?}, y:{:?}",
                        p.x, p.y
                    )));
                }
                edge_split
                    .entry(*e)
                    .or_insert_with(smallvec::SmallVec::<[(u64, u64); 1]>::new)
                    .push(transmute_to_u64(p));
            }
        }
    }
    //println!("Input vertices : {:?}", input_pb_model.vertices.len());
    //println!();

    let mut output_pb_model = PB_Model {
        name: input_pb_model.name.clone(),
        world_orientation: input_pb_model.world_orientation.clone(),
        vertices: Vec::<PB_Vertex>::with_capacity(input_pb_model.vertices.len() + edge_split.len()),
        faces: Vec::<PB_Face>::with_capacity(input_pb_model.faces.len() + edge_split.len()),
    };

    for v in input_pb_model.vertices.iter() {
        output_pb_model.vertices.push(v.clone());
    }

    for f in input_pb_model.faces.iter().enumerate() {
        if !edge_is_split.bit(f.0) {
            output_pb_model.faces.push(f.1.clone());
        }
    }
    // output_pb_model now contains a copy of input_pb_model except for the edges with an intersection
    // Add the intersecting edges, but split them first

    // a map of hashable point to vertex number
    let mut new_vertex_map = ahash::AHashMap::<(u64, u64), usize>::default();
    for (edge, hash_pos) in edge_split.into_iter() {
        let mut new_edge = smallvec::SmallVec::<[u64; 4]>::new();
        let old_face = &input_pb_model.faces[edge];
        new_edge.push(old_face.vertices[0]);
        for hash_pos in hash_pos.iter() {
            let new_vertex_index = new_vertex_map.entry(*hash_pos).or_insert_with_key(|k| {
                let new_vertex = transmute_to_f64(k);
                let vertex = match plane {
                    Plane::XY => PB_Vertex {
                        x: new_vertex.x as f64,
                        y: new_vertex.y as f64,
                        z: 0.0,
                    },
                    Plane::XZ => PB_Vertex {
                        x: new_vertex.x as f64,
                        y: 0.0,
                        z: new_vertex.y as f64,
                    },
                    Plane::ZY => PB_Vertex {
                        x: 0.0,
                        y: new_vertex.y as f64,
                        z: new_vertex.x as f64,
                    },
                };
                output_pb_model.vertices.push(vertex);
                output_pb_model.vertices.len() - 1
            });
            new_edge.push(*new_vertex_index as u64);
        }
        //println!("hash pos:{:?} has index:{:?} is {:?}",hash_pos, new_vertex_index, transmute_to_f64(&hash_pos));
        new_edge.push(old_face.vertices[1]);
        //println!("new edge is {:?}", new_edge);

        let origin = &output_pb_model.vertices[old_face.vertices[0] as usize];

        /*for new_edge in new_edge
        .iter()
        .map(|x| (origin
            .distance_squared(&output_pb_model.vertices[*x as usize]), x))
        .tuple_windows::<(_, _)>()*/

        for new_edge in new_edge
            .iter()
            .sorted_unstable_by(|a, b| {
                (origin
                    .distance_squared(&output_pb_model.vertices[**a as usize])
                    .partial_cmp(&origin.distance_squared(&output_pb_model.vertices[**b as usize])))
                .unwrap()
            })
            .tuple_windows::<(_, _)>()
        {
            output_pb_model.faces.push(PB_Face {
                vertices: vec![*new_edge.0, *new_edge.1],
            });
            //println!(" step new edge is {:?}-{:?}", new_edge.0, new_edge.1);
        }
    }
    //for (k,v) in new_vertex_map.into_iter() {
    //    println!("k:{:?} v:{:?} p:{:?}", k, v,  output_pb_model.vertices[v]);
    //}

    Ok(output_pb_model)
}

pub fn command(
    a_command: &PB_Command,
    _options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!(
        r#" ____  __.      .__  _____      .___        __                                     __
|    |/ _| ____ |__|/ ____\____ |   | _____/  |_  ___________  ______ ____   _____/  |_
|      <  /    \|  \   __\/ __ \|   |/    \   __\/ __ \_  __ \/  ___// __ \_/ ___\   __\
|    |  \|   |  \  ||  | \  ___/|   |   |  \  | \  ___/|  | \/\___ \\  ___/\  \___|  |
|____|__ \___|  /__||__|  \___  >___|___|  /__|  \___  >__|  /____  >\___  >\___  >__|
        \/    \/              \/         \/          \/           \/     \/     \/      "#
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
        let output_model = knife_intersect(&input_model)?;
        let mut reply = PB_Reply {
            options: vec![PB_KeyValuePair {
                key: "ONLY_EDGES".to_string(),
                value: "True".to_string(),
            }],
            models: Vec::<PB_Model>::new(),
        };

        reply.models.push(output_model);
        Ok(reply)
    } else {
        Err(TBError::InvalidInputData(
            "Model did not contain any data".to_string(),
        ))
    }
}
