use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex as PB_Vertex;
use std::collections::HashMap;
use std::time;

use dcc_lsystem::renderer::DataRendererOptions;
use dcc_lsystem::renderer::Renderer;
use dcc_lsystem::turtle::{TurtleAction, TurtleLSystemBuilder};
use dcc_lsystem::LSystemBuilder;

/// Code directly copy & pasted from dcc-lsystem examples
fn build_dragon_curve(cmd_arg_iterations: usize) -> Result<Vec<(i32, i32, i32, i32)>, TBError> {
    let mut builder = TurtleLSystemBuilder::new();

    let _ = builder
        .token("X", TurtleAction::Nothing)
        .token("Y", TurtleAction::Nothing)
        .token("F", TurtleAction::Forward(30))
        .token("+", TurtleAction::Rotate(-90))
        .token("-", TurtleAction::Rotate(90))
        .axiom("F X")
        .rule("X => X + Y F +")
        .rule("Y => - F X - Y");

    let (mut system, renderer) = builder.finish();
    system.step_by(cmd_arg_iterations);

    let now = time::Instant::now();
    let options = DataRendererOptions::default();
    let rv = renderer.render(&system, &options);
    println!("build_dragon_curve render() duration: {:?}", now.elapsed());

    Ok(rv)
}

/// Code directly copy & pasted from dcc-lsystem examples
fn sierpinski_triangle(cmd_arg_iterations: usize) -> Result<Vec<(i32, i32, i32, i32)>, TBError> {
    let mut builder = TurtleLSystemBuilder::new();

    let _ = builder
        .token("F", TurtleAction::Forward(200))
        .token("G", TurtleAction::Forward(200))
        .token("+", TurtleAction::Rotate(120))
        .token("-", TurtleAction::Rotate(-120))
        .axiom("F - G - G")
        .rule("F => F - G + F + G - F")
        .rule("G => G G");

    let (mut system, renderer) = builder.finish();
    system.step_by(cmd_arg_iterations);

    let now = time::Instant::now();
    let options = DataRendererOptions::default();
    let rv = renderer.render(&system, &options);
    println!("sierpinski_triangle render() duration: {:?}", now.elapsed());

    Ok(rv)
}

/// Code directly copy & pasted from dcc-lsystem examples
fn sierpinski_arrowhead(cmd_arg_iterations: usize) -> Result<Vec<(i32, i32, i32, i32)>, TBError> {
    let mut builder = TurtleLSystemBuilder::new();

    let _ = builder
        .token("A", TurtleAction::Forward(200))
        .token("B", TurtleAction::Forward(200))
        .token("+", TurtleAction::Rotate(60))
        .token("-", TurtleAction::Rotate(-60))
        .axiom("A")
        .rule("A => B - A - B")
        .rule("B => A + B + A");

    let (mut system, renderer) = builder.finish();
    system.step_by(cmd_arg_iterations);

    let now = time::Instant::now();
    let options = DataRendererOptions::default();
    let rv = renderer.render(&system, &options);
    println!("sierpinski_triangle render() duration: {:?}", now.elapsed());

    Ok(rv)
}

/// Code directly copy & pasted from dcc-lsystem examples
fn koch_curve(cmd_arg_iterations: usize) -> Result<Vec<(i32, i32, i32, i32)>, TBError> {
    let mut builder = TurtleLSystemBuilder::new();

    let _ = builder
        .token("F", TurtleAction::Forward(30))
        .token("+", TurtleAction::Rotate(90))
        .token("-", TurtleAction::Rotate(-90))
        .axiom("F")
        .rule("F => F + F - F - F + F");

    let (mut system, renderer) = builder.finish();
    system.step_by(cmd_arg_iterations);

    let now = time::Instant::now();
    let options = DataRendererOptions::default();
    let rv = renderer.render(&system, &options);
    println!("koch_curve render() duration: {:?}", now.elapsed());

    Ok(rv)
}

/// Code directly copy & pasted from dcc-lsystem examples
fn build_cantor_sets(cmd_arg_iterations: usize) -> Result<Vec<(i32, i32, i32, i32)>, TBError> {
    let mut builder = LSystemBuilder::new();

    let a = builder.token("A");
    let b = builder.token("B");

    builder.axiom(vec![a]);
    builder.transformation_rule(a, vec![a, b, a]);
    builder.transformation_rule(b, vec![b, b, b]);

    let mut system = builder.finish();

    // the total number of states (including the initial state!) to render
    let step_limit: u32 = cmd_arg_iterations as u32;

    // At state number `step_limit`, our diagram has 3^(step_limit - 1) bars,
    // so we make the width of our image an integer multiple of this number.
    let width = 3_u32.pow(step_limit - 1) * 2;

    // the vertical spacing between each bar in the render
    let vertical_spacing = 5;

    let mut lines = Vec::new();
    let now = time::Instant::now();
    for index in 0..step_limit {
        let state = system.get_state();
        let bar_width: u32 = width / state.len() as u32;

        let mut x: u32 = 0;
        let y = vertical_spacing * index;

        for token in state {
            if *token == a {
                // draw a line
                lines.push((x as i32, y as i32, (x + bar_width) as i32, y as i32));
            }
            x += bar_width;
        }

        system.step();
    }
    println!("build_cantor_sets render() duration: {:?}", now.elapsed());

    Ok(lines)
}

/// Code directly copy & pasted from dcc-lsystem examples
fn fractal_binary_tree(cmd_arg_iterations: usize) -> Result<Vec<(i32, i32, i32, i32)>, TBError> {
    let mut builder = TurtleLSystemBuilder::new();

    let _ = builder
        .token("0", TurtleAction::Forward(50))
        .token("1", TurtleAction::Forward(50))
        .token("L", TurtleAction::Rotate(45))
        .token("R", TurtleAction::Rotate(-45))
        .token("[", TurtleAction::Push)
        .token("]", TurtleAction::Pop)
        .axiom("0")
        .rule("1 => 1 1")
        .rule("0 => 1 [ L 0 ] R 0")
        .rotate(90);

    let (mut system, renderer) = builder.finish();
    system.step_by(cmd_arg_iterations);

    let now = time::Instant::now();
    let options = DataRendererOptions::default();
    let rv = renderer.render(&system, &options);
    println!("fractal_binary_tree render() duration: {:?}", now.elapsed());

    Ok(rv)
}

/// Code directly copy & pasted from dcc-lsystem examples
fn fractal_plant(cmd_arg_iterations: usize) -> Result<Vec<(i32, i32, i32, i32)>, TBError> {
    let mut builder = TurtleLSystemBuilder::new();

    let _ = builder
        .token("X", TurtleAction::Nothing)
        .token("F", TurtleAction::Forward(200))
        .token("+", TurtleAction::Rotate(25))
        .token("-", TurtleAction::Rotate(-25))
        .token("[", TurtleAction::Push)
        .token("]", TurtleAction::Pop)
        .axiom("X")
        .rule("X => F + [ [ X ] - X ] - F [ - F X ] + X")
        .rule("F => F F")
        .rotate(70);

    let (mut system, renderer) = builder.finish();
    system.step_by(cmd_arg_iterations);

    let now = time::Instant::now();
    let options = DataRendererOptions::default();
    let rv = renderer.render(&system, &options);
    println!("fractal_binary_tree render() duration: {:?}", now.elapsed());

    Ok(rv)
}

/// Build the return model
/// lines contains a vector of (<first vertex index>,<a list of points><last vertex index>)
fn build_output_bp_model(
    _a_command: &PB_Command,
    lines: Vec<(i32, i32, i32, i32)>,
) -> Result<PB_Model, TBError> {
    let mut vertices_map = ahash::AHashMap::<(i32, i32), usize>::new();
    // Default capacity is probably a little bit too low
    let mut pb_vertices = Vec::<PB_Vertex>::with_capacity(lines.len());
    let mut pb_faces = Vec::<PB_Face>::with_capacity(lines.len());
    let mut src_aabb = linestring::cgmath_2d::Aabb2::default();

    for (x0, y0, x1, y1) in lines.into_iter() {
        let key = (x0, y0);
        let i0 = *vertices_map.entry(key).or_insert_with(|| {
            let n = pb_vertices.len();
            let point = cgmath::Point2 {
                x: x0 as f64,
                y: y0 as f64,
            };
            src_aabb.update_point(&point);
            pb_vertices.push(PB_Vertex {
                x: point.x,
                y: point.y,
                z: 0.0,
            });
            n
        });

        let key = (x1, y1);
        let i1 = *vertices_map.entry(key).or_insert_with(|| {
            let n = pb_vertices.len();
            let point = cgmath::Point2 {
                x: x1 as f64,
                y: y1 as f64,
            };
            src_aabb.update_point(&point);
            pb_vertices.push(PB_Vertex {
                x: point.x,
                y: point.y,
                z: 0.0,
            });
            n
        });

        pb_faces.push(PB_Face {
            vertices: vec![i0 as u64, i1 as u64],
        });
    }

    let to_center = cgmath::Vector2 {
        x: -(src_aabb.get_high().unwrap().x + src_aabb.get_low().unwrap().x) / 2.0,
        y: -(src_aabb.get_high().unwrap().y + src_aabb.get_low().unwrap().y) / 2.0,
    };
    let scale = if (src_aabb.get_high().unwrap().x - src_aabb.get_low().unwrap().x)
        > (src_aabb.get_high().unwrap().y - src_aabb.get_low().unwrap().y)
    {
        3.0 / (src_aabb.get_high().unwrap().x - src_aabb.get_low().unwrap().x)
    } else {
        3.0 / (src_aabb.get_high().unwrap().y - src_aabb.get_low().unwrap().y)
    };
    //println!("Aabb:{:?} to center: {:?}, scale: {:?}", src_aabb, to_center, scale);
    // center and scale the result so that no axis is larger than 3 'units'
    for v in pb_vertices.iter_mut() {
        v.x = (v.x + to_center.x) * scale;
        v.y = (v.y + to_center.y) * scale;
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
    if a_command.models.len() > 0 {
        return Err(TBError::InvalidInputData(format!(
            "This operation does not use any models as input:{}",
            a_command.models.len()
        )));
    }
    let cmd_arg_iterations = options
        .get("ITERATIONS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the ITERATIONS parameter".to_string()))?
        .parse::<usize>()
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
        "CANTOR_SETS" => build_cantor_sets(cmd_arg_iterations),
        "FRACTAL_BINARY_TREE" => fractal_binary_tree(cmd_arg_iterations),
        "FRACTAL_PLANT" => fractal_plant(cmd_arg_iterations),
        "SIERPINSKI_TRIANGLE" => sierpinski_triangle(cmd_arg_iterations),
        "SIERPINSKI_ARROWHEAD" => sierpinski_arrowhead(cmd_arg_iterations),
        "KOCH_CURVE" => koch_curve(cmd_arg_iterations),
        _ => Err(TBError::InvalidInputData(format!(
            "Invalid CMD_VARIANT parameter:{}",
            cmd_arg_variant
        ))),
    }?;

    let pb_model = build_output_bp_model(&a_command, mesh)?;
    println!("pb_model.vertices.len() {}", pb_model.vertices.len());
    println!("pb_model.faces.len() {}", pb_model.faces.len());
    println!(
        "pb_model.faces[0].vertices.len() {}",
        pb_model.faces[0].vertices.len()
    );

    let reply = PB_Reply {
        options: vec![PB_KeyValuePair {
            key: "ONLY_EDGES".to_string(),
            value: "True".to_string(),
        }],
        models: vec![pb_model],
    };
    Ok(reply)
}
