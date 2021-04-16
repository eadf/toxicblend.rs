use super::TBError;
use crate::toxicblend::Command as PB_Command;
use crate::toxicblend::Face as PB_Face;
use crate::toxicblend::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend::Model as PB_Model;
use crate::toxicblend::Reply as PB_Reply;
use crate::toxicblend::Vertex as PB_Vertex;
use cgmath::{Angle, SquareMatrix};
use cgmath::{Transform, UlpsEq};
use itertools::Itertools;
use linestring::cgmath_3d;
use linestring::cgmath_3d::Plane;
use rayon::prelude::*;
use std::collections::HashMap;

const EPSILON: f64 = 0.0001;
// the largest dimension of the voronoi input, totally arbitrarily selected.
const MAX_VORONOI_DIMENSION: f64 = 256.0 * 800.0;

#[inline(always)]
/// make a key from v0 and v1, lowest index will always be first
fn make_edge_key(v0: usize, v1: usize) -> (usize, usize) {
    if v0 < v1 {
        (v0, v1)
    } else {
        (v1, v0)
    }
}

/// converts from a private, comparable and hashable format
/// only use this for floats that are f64::is_finite()
#[allow(dead_code)]
#[inline(always)]
fn transmute_to_f64(a: &(u64, u64, u64)) -> cgmath::Point3<f64> {
    cgmath::Point3 {
        x: f64::from_bits(a.0),
        y: f64::from_bits(a.1),
        z: f64::from_bits(a.2),
    }
}

/// converts to a private, comparable and hashable format
/// only use this for floats that are f64::is_finite()
#[inline(always)]
fn transmute_to_u64(a: &cgmath::Point3<f64>) -> (u64, u64, u64) {
    (a.x.to_bits(), a.y.to_bits(), a.z.to_bits())
}

/// reformat the input into a useful structure
#[allow(clippy::type_complexity)]
pub fn parse_input(
    input_pb_model: &PB_Model,
) -> Result<
    (
        fnv::FnvHashSet<(usize, usize)>,
        Vec<cgmath::Point3<f64>>,
        cgmath_3d::Aabb3<f64>,
    ),
    TBError,
> {
    let mut aabb = cgmath_3d::Aabb3::<f64>::default();
    for v in input_pb_model.vertices.iter() {
        aabb.update_point(&cgmath::Point3::new(v.x as f64, v.y as f64, v.z as f64))
    }

    let plane =
        Plane::get_plane_relaxed(&aabb, EPSILON, f64::default_max_ulps()).ok_or_else(|| {
            let aabbe_v = aabb.get_high().unwrap() - aabb.get_low().unwrap();
            TBError::InputNotPLane(format!(
                "Input data not in one plane and/or not intersecting origo: Δ({},{},{})",
                aabbe_v.x, aabbe_v.y, aabbe_v.z
            ))
        })?;
    println!("centerline: data was in plane:{:?} aabb:{:?}", plane, aabb);

    let mut edge_set = fnv::FnvHashSet::<(usize, usize)>::default();

    for face in input_pb_model.faces.iter() {
        if face.vertices.len() > 2 {
            return Err(TBError::InvalidInputData(
                "Model contains faces, only lines are supported for this operation".to_string(),
            ));
        }
        if face.vertices.len() < 2 {
            return Err(TBError::InvalidInputData(
                "Points are not supported for this operation".to_string(),
            ));
        }
        let v0 = *face.vertices.get(0).unwrap() as usize;
        let v1 = *face.vertices.get(1).unwrap() as usize;
        let key = make_edge_key(v0, v1);
        let _ = edge_set.insert(key);
    }
    for p in input_pb_model.vertices.iter() {
        if !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
            return Err(TBError::InvalidInputData(format!(
                "Only valid coordinates are allowed ({},{},{})",
                p.x, p.y, p.z
            )));
        }
    }
    let p: Vec<cgmath::Point3<f64>> = input_pb_model
        .vertices
        .iter()
        .map(|v| cgmath::Point3 {
            x: v.x,
            y: v.y,
            z: v.z,
        })
        .collect();
    Ok((edge_set, p, aabb))
}

/// Build the return model
pub fn build_bp_model(
    a_command: &PB_Command,
    shapes: Vec<(
        linestring::cgmath_2d::LineStringSet2<f64>,
        centerline::Centerline<i64, f64>,
    )>,
    inverted_transform: cgmath::Matrix4<f64>,
) -> Result<PB_Model, TBError> {
    let input_pb_model = &a_command.models[0];

    let count:usize = (shapes.iter().map::<usize,_>(|(ls, cent)|{
        ls.set().iter().map(|ls|{ls.points().len()}).sum::<usize>() +
           cent.lines.iter().flatten().count() +
            cent.line_strings.iter().flatten().map(|ls|ls.len()).sum::<usize>()
    }).sum::<usize>()*5)/4;

    let mut output_pb_model_vertices = Vec::<PB_Vertex>::with_capacity(count);
    let mut output_pb_model_faces = Vec::<PB_Face>::with_capacity(count);

    // map between 'meta-vertex' and vertex index
    let mut v_map = fnv::FnvHashMap::<(u64, u64, u64), usize>::default();

    for shape in shapes.into_iter() {
        // Draw the input segments
        // todo: should this be optional?
        for linestring in shape.0.set().iter() {
            if linestring.points().len() < 2 {
                return Err(TBError::InternalError(
                    "Linestring with less than 2 points found".to_string(),
                ));
            }
            // unwrap of first and last is safe now that we know there are at least 2 vertices in the list
            let v0 = Plane::XY.to_3d(&linestring.points().first().unwrap());
            let v1 = Plane::XY.to_3d(&linestring.points().last().unwrap());
            let v0_key = transmute_to_u64(&v0);
            let v0_index = *v_map.entry(v0_key).or_insert_with(|| {
                let new_index = output_pb_model_vertices.len();
                output_pb_model_vertices
                    .push(PB_Vertex::from(inverted_transform.transform_point(v0)));
                new_index
            });
            let v1_key = transmute_to_u64(&v1);
            let v1_index = *v_map.entry(v1_key).or_insert_with(|| {
                let new_index = output_pb_model_vertices.len();
                output_pb_model_vertices
                    .push(PB_Vertex::from(inverted_transform.transform_point(v1)));
                new_index
            });
            let vertex_index_iterator = Some(v0_index)
                .into_iter()
                .chain(
                    linestring
                        .points()
                        .iter()
                        .skip(1)
                        .take(linestring.points().len() - 2)
                        .map(|p| {
                            let new_index = output_pb_model_vertices.len();
                            output_pb_model_vertices.push(PB_Vertex::from(
                                inverted_transform.transform_point(Plane::XY.to_3d(p)),
                            ));
                            new_index
                        }),
                )
                .chain(Some(v1_index).into_iter());
            for p in vertex_index_iterator.tuple_windows::<(_, _)>() {
                output_pb_model_faces.push(PB_Face {
                    vertices: vec![p.0 as u64, p.1 as u64],
                });
            }
        }
        // draw the straight edges of the voronoi output
        for line in shape.1.lines.iter().flatten() {
            let v0 = line.start;
            let v1 = line.end;
            if v0 == v1 {
                continue;
            }
            let v0_key = transmute_to_u64(&v0);
            let v0_index = *v_map.entry(v0_key).or_insert_with(|| {
                let new_index = output_pb_model_vertices.len();
                output_pb_model_vertices
                    .push(PB_Vertex::from(inverted_transform.transform_point(v0)));
                new_index
            });

            let v1_key = transmute_to_u64(&v1);
            let v1_index = *v_map.entry(v1_key).or_insert_with(|| {
                let new_index = output_pb_model_vertices.len();
                output_pb_model_vertices
                    .push(PB_Vertex::from(inverted_transform.transform_point(v1)));
                new_index
            });

            if v0_index == v1_index {
                println!(
                    "v0_index==v1_index, but v0!=v1 v0:{:?} v1:{:?} v0_index:{:?} v1_index:{:?}",
                    v0, v1, v0_index, v1_index
                );
                continue;
            }
            output_pb_model_faces.push(PB_Face {
                vertices: vec![v0_index as u64, v1_index as u64],
            });
        }
        // draw the concatenated line strings of the voronoi output
        //for linestring in shape.1.line_strings.iter().flatten() {
        for linestring in shape.1.line_strings.iter().flatten() {
            if linestring.points().len() < 2 {
                return Err(TBError::InternalError(
                    "Linestring with less than 2 points found".to_string(),
                ));
            }
            // unwrap of first and last is safe now that we know there are at least 2 vertices in the list
            let v0 = linestring.points().first().unwrap();
            let v1 = linestring.points().last().unwrap();
            let v0_key = transmute_to_u64(&v0);
            let v0_index = *v_map.entry(v0_key).or_insert_with(|| {
                let new_index = output_pb_model_vertices.len();
                output_pb_model_vertices
                    .push(PB_Vertex::from(inverted_transform.transform_point(*v0)));
                new_index
            });
            let v1_key = transmute_to_u64(&v1);
            let v1_index = *v_map.entry(v1_key).or_insert_with(|| {
                let new_index = output_pb_model_vertices.len();
                output_pb_model_vertices
                    .push(PB_Vertex::from(inverted_transform.transform_point(*v1)));
                new_index
            });
            let vertex_index_iterator = Some(v0_index)
                .into_iter()
                .chain(
                    linestring
                        .points()
                        .iter()
                        .skip(1)
                        .take(linestring.points().len() - 2)
                        .map(|p| {
                            let new_index = output_pb_model_vertices.len();
                            output_pb_model_vertices.push(PB_Vertex::from(
                                inverted_transform.transform_point(*p),
                            ));
                            new_index
                        }),
                )
                .chain(Some(v1_index).into_iter());
            for p in vertex_index_iterator.tuple_windows::<(_, _)>() {
                output_pb_model_faces.push(PB_Face {
                    vertices: vec![p.0 as u64, p.1 as u64],
                });
            }
        }
    }
    //println!("allocated {} needed {} and {}", count, output_pb_model_vertices.len(), output_pb_model_faces.len());
    Ok(PB_Model {
        name: input_pb_model.name.clone(),
        world_orientation: input_pb_model.world_orientation.clone(),
        vertices: output_pb_model_vertices,
        faces: output_pb_model_faces,
    })
}

pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    // angle is supposed to be in degrees
    let cmd_arg_angle = {
        let value = options
            .get("ANGLE")
            .ok_or_else(|| TBError::InvalidInputData("Missing the ANGLE parameter".to_string()))?;
        value.parse::<f64>().map_err(|_| {
            TBError::InvalidInputData(format!("Could not parse the ANGLE parameter:{:?}", value))
        })?
    };
    if !(0.0..=90.0).contains(&cmd_arg_angle) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of ANGLE is [0..90] :({})",
            cmd_arg_angle
        )));
    }
    let cmd_arg_remove_internals = {
        let tmp_true = "true".to_string();
        let value = options.get("REMOVE_INTERNALS").unwrap_or(&tmp_true);
        value.parse::<bool>().map_err(|_| {
            TBError::InvalidInputData(format!(
                "Could not parse the REMOVE_INTERNALS parameter {:?}",
                value
            ))
        })?
    };
    let cmd_arg_distance = {
        let value = options.get("DISTANCE").ok_or_else(|| {
            TBError::InvalidInputData("Missing the DISTANCE parameter".to_string())
        })?;
        value.parse::<f64>().map_err(|_| {
            TBError::InvalidInputData(format!(
                "Could not parse the DISTANCE parameter {:?}",
                value
            ))
        })?
    };
    if !(0.004..100.0).contains(&cmd_arg_distance) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of DISTANCE is [0.005..100[% :({})",
            cmd_arg_distance
        )));
    }
    let cmd_arg_max_voronoi_dimension = {
        let tmp_value = MAX_VORONOI_DIMENSION.to_string();
        let value = options.get("MAX_VORONOI_DIMENSION").unwrap_or(&tmp_value);
        value.parse::<f64>().map_err(|_| {
            TBError::InvalidInputData(format!(
                "Could not parse the MAX_VORONOI_DIMENSION parameter {:?}",
                value
            ))
        })?
    };
    if !(MAX_VORONOI_DIMENSION..100_000_000.0).contains(&cmd_arg_max_voronoi_dimension) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of DISTANCE is [{}..100_000_000[% :({})",
            MAX_VORONOI_DIMENSION, cmd_arg_max_voronoi_dimension
        )));
    }
    let cmd_arg_simplify = {
        let tmp_true = "true".to_string();
        let value = options.get("SIMPLIFY").unwrap_or(&tmp_true);
        value.parse::<bool>().map_err(|_| {
            TBError::InvalidInputData(format!(
                "Could not parse the SIMPLIFY parameter {:?}",
                value
            ))
        })?
    };

    // used for simplification and discretization distance
    let max_distance = cmd_arg_max_voronoi_dimension * cmd_arg_distance / 100.0;

    if a_command.models.is_empty() || a_command.models[0].vertices.is_empty() {
        return Err(TBError::InvalidInputData(
            "Model did not contain any data".to_string(),
        ));
    }

    // The dot product between normalized vectors of edge and the segment that created it.
    // Can also be described as cos(angle) between edge and segment.
    let dot_limit = cgmath::Deg::<f64>(cmd_arg_angle).cos();

    if a_command.models.len() > 1 {
        println!("centerline models.len(): {}", a_command.models.len());
        return Err(TBError::InvalidInputData(
            "This operation only supports one model as input".to_string(),
        ));
    }
    println!("centerline got command: {}", a_command.command);
    for model in a_command.models.iter() {
        println!("model.name:{:?}", model.name);
        println!("model.vertices:{:?}", model.vertices.len());
        println!("model.faces:{:?}", model.faces.len());
        println!(
            "model.world_orientation:{:?}",
            model.world_orientation.as_ref().map_or(0, |_| 16)
        );
        println!("ANGLE:{:?}°", cmd_arg_angle);
        println!("REMOVE_INTERNALS:{:?}", cmd_arg_remove_internals);
        println!("SIMPLIFY:{:?}", cmd_arg_simplify);
        println!("DISTANCE:{:?}%", cmd_arg_distance);
        println!("MAX_VORONOI_DIMENSION:{:?}", cmd_arg_max_voronoi_dimension);
        println!("max_distance:{:?}", max_distance);
        println!();
    }
    //println!("-> parse_input");
    let (edges, points, total_aabb) = parse_input(&a_command.models[0])?;
    //println!("-> divide_into_shapes");
    let lines = centerline::divide_into_shapes(edges, points)?;
    //println!("-> get_transform_relaxed");
    let (_plane, transform, _voronoi_input_aabb) = centerline::get_transform_relaxed(
        &total_aabb,
        cmd_arg_max_voronoi_dimension,
        EPSILON,
        f64::default_max_ulps(),
    )?;

    let invers_transform = transform
        .invert()
        .ok_or(TBError::CouldNotCalculateInverseMatrix)?;

    //println!("-> transform");
    // transform each linestring to 2d
    let mut raw_data: Vec<linestring::cgmath_2d::LineStringSet2<f64>> = lines
        .par_iter()
        .map(|x| x.transform(&transform).copy_to_2d(cgmath_3d::Plane::XY))
        .collect();
    {
        // truncate the floats to nearest int
        let truncate_float = |x: f64| -> f64 { x as i64 as f64 };
        for r in raw_data.iter_mut() {
            r.operation(&truncate_float);
        }
    }

    //println!("->calculate hull");

    // calculate the hull of each shape
    let raw_data: Vec<linestring::cgmath_2d::LineStringSet2<f64>> = raw_data
        .into_par_iter()
        .map(|mut x| {
            x.calculate_convex_hull();
            x
        })
        .collect();

    //println!("Started with {} shapes", raw_data.len());
    let raw_data = centerline::consolidate_shapes(raw_data)?;
    //println!("Reduced to {} shapes", raw_data.len());

    let shapes = raw_data
        .into_par_iter()
        .map(|shape| {
            let mut segments = Vec::<boostvoronoi::Line<i64>>::with_capacity(
                shape.set().iter().map(|x| x.len()).sum(),
            );
            for lines in shape.set().iter() {
                for lineseq in lines.as_lines().iter() {
                    segments.push(boostvoronoi::Line::new(
                        boostvoronoi::Point {
                            x: lineseq.start.x as i64,
                            y: lineseq.start.y as i64,
                        },
                        boostvoronoi::Point {
                            x: lineseq.end.x as i64,
                            y: lineseq.end.y as i64,
                        },
                    ))
                }
            }
            let mut c = centerline::Centerline::<i64, f64>::with_segments(segments);
            if let Err(centerline_error) = c.build_voronoi() {
                return Err(centerline_error.into());
            }
            if cmd_arg_remove_internals {
                if let Err(centerline_error) =
                    c.calculate_centerline(dot_limit, max_distance, shape.get_internals())
                {
                    return Err(centerline_error.into());
                }
            } else {
                if let Err(centerline_error) = c.calculate_centerline(dot_limit, max_distance, None)
                {
                    return Err(centerline_error.into());
                }
            }
            if cmd_arg_simplify && c.line_strings.is_some() {
                // simplify every line string with rayon
                c.line_strings = Some(
                    c.line_strings
                        .take()
                        .unwrap()
                        .into_par_iter()
                        .map(|ls| {
                            //let pre = ls.len();
                            ls.simplify(max_distance)
                            ////println!("simplified ls from {} to {}", pre, ls.len());
                            //ls
                        })
                        .collect(),
                );
            }
            Ok((shape, c))
        })
        .collect::<Result<
            Vec<(
                linestring::cgmath_2d::LineStringSet2<f64>,
                centerline::Centerline<i64, f64>,
            )>,
            TBError,
        >>()?;
    //println!("<-build_voronoi");

    let model = build_bp_model(&a_command, shapes, invers_transform)?;

    //println!("<-build_bp_model");
    let mut reply = PB_Reply {
        options: vec![PB_KeyValuePair {
            key: "ONLY_EDGES".to_string(),
            value: "True".to_string(),
        }],
        models: Vec::<PB_Model>::new(),
    };
    //println!(
    //    "<-PB_Reply vertices:{:?}, faces:{:?}",
    //    model.vertices.len(),
    //    model.faces.len()
    //);
    reply.models.push(model);
    Ok(reply)
}
