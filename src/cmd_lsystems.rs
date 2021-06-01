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

/// converts to a private, comparable and hash-able format.
/// Only use this for bit perfect float copies that are f64::is_finite()
#[inline(always)]
fn transmute_to_u64(p: &cgmath::Point3<f64>) -> (u64, u64, u64) {
    let mut x = p.x;
    let mut y = p.y;
    let mut z = p.z;

    if x == -0.0 {
        x = 0.0;
    }
    if y == -0.0 {
        y = 0.0;
    }
    if z == -0.0 {
        z = 0.0;
    }
    (x.to_bits(), y.to_bits(), z.to_bits())
}

/// Algorithmic_botany, page 11 (http://algorithmicbotany.org/papers/#abop)
fn build_dragon_curve(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();

    let result = TurtleRules::default()
        .add_token('L', TurtleCommand::Forward(1.0))?
        .add_token('R', TurtleCommand::Forward(1.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(90.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-90.0f64.to_radians())))?
        .add_axiom("L".to_string())?
        .add_rule('L', "L + R +".to_string())?
        .add_rule('R', "- L - R".to_string())?
        .round()?
        .exec(cmd_arg_iterations, Turtle::default())?;

    println!("build_dragon_curve render() duration: {:?}", now.elapsed());
    Ok(result)
}

fn build_dragon_curve_3d(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();

    let result = TurtleRules::default()
        .add_token('L', TurtleCommand::Forward(1.0))?
        .add_token('R', TurtleCommand::Forward(1.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(-90.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Pitch(Rad(90.0f64.to_radians())))?
        .add_axiom("L".to_string())?
        .add_rule('L', "L + R +".to_string())?
        .add_rule('R', "- L - R".to_string())?
        .round()?
        .exec(cmd_arg_iterations, Turtle::default())?;

    println!(
        "build_dragon_curve_3d render() duration: {:?}",
        now.elapsed()
    );
    Ok(result)
}

/// https://en.wikipedia.org/wiki/L-system#Examples_of_L-systems
fn sierpinski_triangle(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('F', TurtleCommand::Forward(1.0))?
        .add_token('G', TurtleCommand::Forward(1.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(120.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-120.0f64.to_radians())))?
        .add_axiom("F-G-G".to_string())?
        .add_rule('F', " F-G+F+G-F".to_string())?
        .add_rule('G', " GG".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("sierpinski_triangle render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Algorithmic_botany, page 11 (http://algorithmicbotany.org/papers/#abop)
fn sierpinski_gasket(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('R', TurtleCommand::Forward(200.0))?
        .add_token('L', TurtleCommand::Forward(200.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(60.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-60.0f64.to_radians())))?
        .add_axiom("R".to_string())?
        .add_rule('R', " L - R - L".to_string())?
        .add_rule('L', " R + L + R".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("sierpinski_gasket render() duration: {:?}", now.elapsed());
    Ok(result)
}

fn sierpinski_gasket_3d(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('R', TurtleCommand::Forward(200.0))?
        .add_token('L', TurtleCommand::Forward(200.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(60.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Pitch(Rad(-60.0f64.to_radians())))?
        .add_axiom("R".to_string())?
        .add_rule('R', " L - R - L".to_string())?
        .add_rule('L', " R + L + R".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("sierpinski_gasket render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Algorithmic_botany, page 12 (http://algorithmicbotany.org/papers/#abop)
fn gosper_curve(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('R', TurtleCommand::Forward(1.0))?
        .add_token('L', TurtleCommand::Forward(1.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(60.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-60.0f64.to_radians())))?
        .add_axiom("L".to_string())?
        .add_rule('L', " L+R++R-L--LL-R+".to_string())?
        .add_rule('R', " -L+RR++R+L--L-R".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("sierpinski_gasket render() duration: {:?}", now.elapsed());
    Ok(result)
}

fn gosper_curve_3d(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('R', TurtleCommand::Forward(1.0))?
        .add_token('L', TurtleCommand::Forward(1.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(60.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-60.0f64.to_radians())))?
        .add_token('P', TurtleCommand::Roll(Rad(60.0f64.to_radians())))?
        .add_token('p', TurtleCommand::Roll(Rad(-60.0f64.to_radians())))?
        .add_axiom("L".to_string())?
        .add_rule('L', " L+R++R-L--LL-R+P".to_string())?
        .add_rule('R', " -L+RR++R+L--L-Rp".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("sierpinski_gasket render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Algorithmic_botany, page 9 (http://algorithmicbotany.org/papers/#abop)
fn koch_curve(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('F', TurtleCommand::Forward(30.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(90.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-90.0f64.to_radians())))?
        .add_axiom("F".to_string())?
        .add_rule('F', " F + F - F - F + F".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("koch_curve render() duration: {:?}", now.elapsed());
    Ok(result)
}

fn koch_curve_3d(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('F', TurtleCommand::Forward(30.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(90.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Roll(Rad(-90.0f64.to_radians())))?
        .add_axiom("F".to_string())?
        .add_rule('F', " F + F - F - F + F".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("koch_curve_3d render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Algorithmic_botany, page 9 (http://algorithmicbotany.org/papers/#abop)
fn quadratic_koch_curve_island(
    cmd_arg_iterations: u8,
) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('F', TurtleCommand::Forward(30.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(90.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-90.0f64.to_radians())))?
        .add_axiom("F-F-F-F".to_string())?
        .add_rule('F', " F+FF-FF-F-F+F+FF-F-F+F+FF+FF-F".to_string())?
        .round()?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!(
        "quadratic_koch_curve_island render() duration: {:?}",
        now.elapsed()
    );
    Ok(result)
}

fn quadratic_koch_curve_island_3d(
    cmd_arg_iterations: u8,
) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('F', TurtleCommand::Forward(30.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(90.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-90.0f64.to_radians())))?
        .add_token('R', TurtleCommand::Roll(Rad(45.0f64.to_radians())))?
        .add_token('r', TurtleCommand::Roll(Rad(-45.0f64.to_radians())))?
        .add_axiom("F-F-F-F".to_string())?
        .add_rule('F', " F+FRFr-FRFr-F-F+F+FRFr-F-F+F+FRFr+FRFr-F".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!(
        "quadratic_koch_curve_island render() duration: {:?}",
        now.elapsed()
    );
    Ok(result)
}

/// https://en.wikipedia.org/wiki/L-system#Examples_of_L-systems
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
        .add_rule('1', " 11".to_string())?
        .add_rule('0', " 1[L0]R0".to_string())?
        .rotate(Rad(90.0f64.to_radians()), Rad(0.0), Rad(0.0))?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("fractal_binary_tree render() duration: {:?}", now.elapsed());

    Ok(result)
}

fn fractal_binary_tree_3d(
    cmd_arg_iterations: u8,
) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('0', TurtleCommand::Forward(50.0))?
        .add_token('1', TurtleCommand::Forward(50.0))?
        .add_token('L', TurtleCommand::Yaw(Rad(45.0f64.to_radians())))?
        .add_token('R', TurtleCommand::Yaw(Rad(-45.0f64.to_radians())))?
        .add_token('(', TurtleCommand::Roll(Rad(15.0f64.to_radians())))?
        .add_token(')', TurtleCommand::Roll(Rad(-45.0f64.to_radians())))?
        .add_token('[', TurtleCommand::Push)?
        .add_token(']', TurtleCommand::Pop)?
        .add_axiom("0".to_string())?
        .add_rule('1', " 11".to_string())?
        .add_rule('0', " 1[L)0]R(0".to_string())?
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

/// https://en.wikipedia.org/wiki/L-system#Examples_of_L-systems
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
        .add_rule(
            'X',
            " F + a [ [ X ] - s X ] - F [ - s F X ] + a X".to_string(),
        )?
        .add_rule('F', " F a F".to_string())?
        .rotate(Rad(70.0f64.to_radians()), Rad(0.0), Rad(0.0))?
        .exec(cmd_arg_iterations, Turtle::default())?;

    println!("fractal_binary_tree render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Algorithmic_botany, page 20 (http://algorithmicbotany.org/papers/#abop)
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
        .add_rule('A', " B-F+CFC+F-D&F∧D-F+&&CFC+F+B//".to_string())?
        .add_rule('B', " A&F∧CFB∧F∧D∧∧-F-D∧|F∧B|FC∧F∧A//".to_string())?
        .add_rule('C', " |D∧|F∧B-F+C∧F∧A&&FA&F∧C+F+B∧F∧D//".to_string())?
        .add_rule('D', " |CFB-F+B|FA&F∧A&&FB-F+B|FC//".to_string())?
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
    // vertex de-duplication mechanism
    // todo: this is broken since each vertex isn't rounded to int anymore
    let mut vertices_map = ahash::AHashMap::<(u64, u64, u64), usize>::new();
    // make sure to not add the same edge twice, set key is sorted (lower,higher)
    let mut dup_edges = ahash::AHashSet::<(usize, usize)>::new();

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

        //println!("p0: {:?} is {}", p0, i0);
        //println!("p1: {:?} is {}", p1, i1);

        let key = {
            if i0 < i1 {
                (i0, i1)
            } else {
                (i1, i0)
            }
        };
        if !dup_edges.contains(&key) {
            pb_faces.push(PB_Face {
                vertices: vec![i0 as u64, i1 as u64],
            });
        } else {
            let _ = dup_edges.insert(key);
        }
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
        name: "LSystems".to_string(),
        world_orientation: None,
        vertices: pb_vertices,
        faces: pb_faces,
    })
}

pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!(
        r#".____       _________               __
|    |     /   _____/__.__. _______/  |_  ____   _____   ______
|    |     \_____  <   |  |/  ___/\   __\/ __ \ /     \ /  ___/
|    |___  /        \___  |\___ \  |  | \  ___/|  Y Y  \\___ \
|_______ \/_______  / ____/____  > |__|  \___  >__|_|  /____  >
        \/        \/\/         \/            \/      \/     \/ "#
    );
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
            TBError::InvalidInputData(format!(
                "Could not parse the ITERATIONS parameter: '{}'",
                options.get("ITERATIONS").unwrap()
            ))
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
        "DRAGON_CURVE_3D" => build_dragon_curve_3d(cmd_arg_iterations),
        "FRACTAL_BINARY_TREE" => fractal_binary_tree(cmd_arg_iterations),
        "FRACTAL_BINARY_TREE_3D" => fractal_binary_tree_3d(cmd_arg_iterations),
        "FRACTAL_PLANT" => fractal_plant(cmd_arg_iterations),
        "SIERPINSKI_TRIANGLE" => sierpinski_triangle(cmd_arg_iterations),
        "SIERPINSKI_GASKET" => sierpinski_gasket(cmd_arg_iterations),
        "SIERPINSKI_GASKET_3D" => sierpinski_gasket_3d(cmd_arg_iterations),
        "KOCH_CURVE" => koch_curve(cmd_arg_iterations),
        "KOCH_CURVE_3D" => koch_curve_3d(cmd_arg_iterations),
        "GOSPER_CURVE" => gosper_curve(cmd_arg_iterations),
        "GOSPER_CURVE_3D" => gosper_curve_3d(cmd_arg_iterations),
        "KOCH_CURVE_ISLAND" => quadratic_koch_curve_island(cmd_arg_iterations),
        "KOCH_CURVE_ISLAND_3D" => quadratic_koch_curve_island_3d(cmd_arg_iterations),
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
        models32: Vec::with_capacity(0),
    };
    Ok(reply)
}
