use super::lsystems_3d::Turtle;
use super::lsystems_3d::TurtleCommand;
use super::lsystems_3d::TurtleRules;
use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex as PB_Vertex;

use cgmath::{EuclideanSpace, Rad};
use std::collections::HashMap;
use std::time;

/// converts to a private, comparable and hash-able format
/// only use this for floats that are f64::is_finite()
#[inline(always)]
fn transmute_to_u64(a: &cgmath::Point3<f64>) -> (u64, u64, u64) {
    (a.x.to_bits(), a.y.to_bits(), a.z.to_bits())
}

/// Code directly copy & pasted from dcc-lsystem examples
fn build_dragon_curve(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();

    let result = TurtleRules::default()
        .add_token('X', TurtleCommand::Nop)?
        .add_token('Y', TurtleCommand::Nop)?
        .add_token('F', TurtleCommand::Forward(1.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(90.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-90.0f64.to_radians())))?
        .add_axiom("F X".to_string())?
        .add_rule("X => X + Y F +".to_string())?
        .add_rule("Y => - F X - Y".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;

    println!("build_dragon_curve render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Code directly copy & pasted from dcc-lsystem examples
fn sierpinski_triangle(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('F', TurtleCommand::Forward(200.0))?
        .add_token('G', TurtleCommand::Forward(200.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(120.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-120.0f64.to_radians())))?
        .add_axiom("F - G - G".to_string())?
        .add_rule("F => F - G + F + G - F".to_string())?
        .add_rule("G => G G".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("sierpinski_triangle render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Code directly copy & pasted from dcc-lsystem examples
fn sierpinski_arrowhead(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('A', TurtleCommand::Forward(200.0))?
        .add_token('B', TurtleCommand::Forward(200.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(60.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-60.0f64.to_radians())))?
        .add_axiom("A".to_string())?
        .add_rule("A => B - A - B".to_string())?
        .add_rule("B => A + B + A".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("sierpinski_triangle render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Code directly copy & pasted from dcc-lsystem examples
fn koch_curve(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('F', TurtleCommand::Forward(30.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(90.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-90.0f64.to_radians())))?
        .add_axiom("F".to_string())?
        .add_rule("F => F + F - F - F + F".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("koch_curve render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Code directly copy & pasted from dcc-lsystem examples
fn fractal_binary_tree(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('0', TurtleCommand::Forward(50.0))?
        .add_token('1', TurtleCommand::Forward(50.0))?
        .add_token('L', TurtleCommand::Yaw(Rad(45.0f64.to_radians())))?
        .add_token('R', TurtleCommand::Yaw(Rad(-45.0f64.to_radians())))?
        .add_token('[', TurtleCommand::Push)?
        .add_token(']', TurtleCommand::Pop)?
        .add_axiom("0".to_string())?
        .add_rule("1 => 1 1".to_string())?
        .add_rule("0 => 1 [ L 0 ] R 0".to_string())?
        .rotate(Rad(90.0f64.to_radians()), Rad(0.0), Rad(0.0))?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("fractal_binary_tree render() duration: {:?}", now.elapsed());

    Ok(result)
}

/// Rather messy implementation of a text parser for a custom turtle builder.
/// It takes a lot of shortcuts, all parenthesis are ignored, it assumes there is one command per
/// line etc. etc.
fn custom_turtle(
    cmd_arg_iterations: u8,
    cmd_custom_turtle: Option<&String>,
) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    if cmd_custom_turtle.is_none() || cmd_custom_turtle.unwrap().is_empty() {
        return Err(TBError::InvalidInputData(
            "Custom turtle text is empty".to_string(),
        ));
    }
    let cmd_custom_turtle = cmd_custom_turtle.unwrap();

    let now = time::Instant::now();
    let result = TurtleRules::default()
        .parse(cmd_custom_turtle)?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("custom_turtle render() duration: {:?}", now.elapsed());

    Ok(result)
}

/// Code directly copy & pasted from dcc-lsystem examples
fn fractal_plant(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('X', TurtleCommand::Nop)?
        .add_token('F', TurtleCommand::Forward(200.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(25.0f64.to_radians())))?
        .add_token('a', TurtleCommand::Roll(Rad(5.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-25.0f64.to_radians())))?
        .add_token('s', TurtleCommand::Roll(Rad(-5.0f64.to_radians())))?
        .add_token('[', TurtleCommand::Push)?
        .add_token(']', TurtleCommand::Pop)?
        .add_axiom("X".to_string())?
        .add_rule("X => F + a [ [ X ] - s X ] - F [ - s F X ] + a X".to_string())?
        .add_rule("F => F a F".to_string())?
        .rotate(Rad(70.0f64.to_radians()), Rad(0.0), Rad(0.0))?
        .exec(cmd_arg_iterations, Turtle::default())?;

    println!("fractal_binary_tree render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Algorithmic_botany, page 20
fn hilbert_curve_3d(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('A', TurtleCommand::Nop)?
        .add_token('B', TurtleCommand::Nop)?
        .add_token('C', TurtleCommand::Nop)?
        .add_token('D', TurtleCommand::Nop)?
        .add_token('F', TurtleCommand::Forward(10.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(90f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-90f64.to_radians())))?
        .add_token('&', TurtleCommand::Pitch(Rad(90f64.to_radians())))?
        .add_token('∧', TurtleCommand::Pitch(Rad(-90f64.to_radians())))?
        .add_token('\\', TurtleCommand::Roll(Rad(90f64.to_radians())))?
        .add_token('/', TurtleCommand::Roll(Rad(-90f64.to_radians())))?
        .add_token('|', TurtleCommand::Yaw(Rad(180f64.to_radians())))?
        .add_axiom("A".to_string())?
        .add_rule("A => B-F+CFC+F-D&F∧D-F+&&CFC+F+B//".to_string())?
        .add_rule("B => A&F∧CFB∧F∧D∧∧-F-D∧|F∧B|FC∧F∧A//".to_string())?
        .add_rule("C => |D∧|F∧B-F+C∧F∧A&&FA&F∧C+F+B∧F∧D//".to_string())?
        .add_rule("D => |CFB-F+B|FA&F∧A&&FB-F+B|FC//".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;

    println!("fractal_binary_tree render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Build the return model
/// lines contains a vector of (<first vertex index>,<a list of points><last vertex index>)
fn build_output_bp_model(
    _a_command: &PB_Command,
    lines: Vec<[cgmath::Point3<f64>; 2]>,
) -> Result<PB_Model, TBError> {
    let mut vertices_map = ahash::AHashMap::<(u64, u64, u64), usize>::new();
    // Default capacity is probably a little bit too low
    let mut pb_vertices = Vec::<PB_Vertex>::with_capacity(lines.len());
    let mut pb_faces = Vec::<PB_Face>::with_capacity(lines.len());
    let mut src_aabb = linestring::cgmath_3d::Aabb3::default();

    for [p0, p1] in lines.into_iter() {
        let key = transmute_to_u64(&p0);
        let i0 = *vertices_map.entry(key).or_insert_with(|| {
            let n = pb_vertices.len();

            src_aabb.update_point(&p0);
            pb_vertices.push(PB_Vertex {
                x: p0.x,
                y: p0.y,
                z: p0.z,
            });
            n
        });

        let key = transmute_to_u64(&p1);
        let i1 = *vertices_map.entry(key).or_insert_with(|| {
            let n = pb_vertices.len();

            src_aabb.update_point(&p1);
            pb_vertices.push(PB_Vertex {
                x: p1.x,
                y: p1.y,
                z: p1.z,
            });
            n
        });

        pb_faces.push(PB_Face {
            vertices: vec![i0 as u64, i1 as u64],
        });
    }
    if !pb_vertices.is_empty() {
        let to_center =
            -(src_aabb.get_high().unwrap().to_vec() + src_aabb.get_low().unwrap().to_vec()) / 2.0;

        let max_delta = {
            let delta = src_aabb.get_high().unwrap() - src_aabb.get_low().unwrap();
            delta.x.max(delta.y).max(delta.z)
        };

        let scale = 3.0 / max_delta;
        let scale = if scale.is_finite() { scale } else { 1.0 };
        println!(
            "Aabb:{:?} to center: {:?}, scale: {:?}",
            src_aabb, to_center, scale
        );
        // center and scale the result so that no axis is larger than 3 'units'
        for v in pb_vertices.iter_mut() {
            v.x = (v.x + to_center.x) * scale;
            v.y = (v.y + to_center.y) * scale;
            v.z = (v.z + to_center.z) * scale;
        }
    }

    Ok(PB_Model {
        name: "Dragon Curve".to_string(),
        world_orientation: None,
        vertices: pb_vertices,
        faces: pb_faces,
    })
}

pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!("L Systems got command: \"{}\"", a_command.command);
    if !a_command.models.is_empty() {
        return Err(TBError::InvalidInputData(format!(
            "This operation does not use any models as input:{}",
            a_command.models.len()
        )));
    }
    let cmd_arg_iterations = options
        .get("ITERATIONS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the ITERATIONS parameter".to_string()))?
        .parse::<u8>()
        .map_err(|_| {
            TBError::InvalidInputData("Could not parse the ITERATIONS parameter".to_string())
        })?;

    if !(1..21).contains(&cmd_arg_iterations) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of ITERATIONS is [{}..{}[ :({})",
            1, 21, cmd_arg_iterations
        )));
    }
    let cmd_arg_variant = options.get("CMD_VARIANT").ok_or_else(|| {
        TBError::InvalidInputData("Missing the CMD_VARIANT parameter".to_string())
    })?;

    let cmd_custom_turtle = options.get("CUSTOM_TURTLE");

    println!(
        "Lindenmayer systems: cmd_arg_iterations:{:?} ",
        cmd_arg_iterations
    );
    println!(
        "Lindenmayer systems: cmd_arg_variant:{:?} ",
        cmd_arg_variant
    );

    let mesh = match cmd_arg_variant.as_str() {
        "DRAGON_CURVE" => build_dragon_curve(cmd_arg_iterations),
        "FRACTAL_BINARY_TREE" => fractal_binary_tree(cmd_arg_iterations),
        "FRACTAL_PLANT" => fractal_plant(cmd_arg_iterations),
        "SIERPINSKI_TRIANGLE" => sierpinski_triangle(cmd_arg_iterations),
        "SIERPINSKI_ARROWHEAD" => sierpinski_arrowhead(cmd_arg_iterations),
        "KOCH_CURVE" => koch_curve(cmd_arg_iterations),
        "CUSTOM_TURTLE" => custom_turtle(cmd_arg_iterations, cmd_custom_turtle),
        "HILBERT_CURVE_3D" => hilbert_curve_3d(cmd_arg_iterations),
        _ => Err(TBError::InvalidInputData(format!(
            "Invalid CMD_VARIANT parameter:{}",
            cmd_arg_variant
        ))),
    }?;

    let pb_model = build_output_bp_model(&a_command, mesh)?;
    println!("pb_model.vertices.len() {}", pb_model.vertices.len());
    println!("pb_model.faces.len() {}", pb_model.faces.len());

    let reply = PB_Reply {
        options: vec![PB_KeyValuePair {
            key: "ONLY_EDGES".to_string(),
            value: "True".to_string(),
        }],
        models: vec![pb_model],
    };
    Ok(reply)
}
