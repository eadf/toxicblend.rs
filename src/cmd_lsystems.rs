use crate::{PB_Command, PB_Face, PB_KeyValuePair, PB_Model, PB_Reply, PB_Vertex, TBError};

use crate::lsystems_3d::{Turtle, TurtleCommand, TurtleRules};

use cgmath::{EuclideanSpace, Rad};
use std::collections::HashMap;
use std::hash::Hash;
use std::time;

#[derive(PartialEq, Eq, Hash)]
/// Converts a `cgmath::Point3<f64>` to a comparable and hash-able opaque.
/// Only use this as a hashmap key for bit perfect Point3s that are `f64::is_finite()`
struct F64Key {
    x: u64,
    y: u64,
    z: u64,
}

impl F64Key {
    fn new(p: &cgmath::Point3<f64>) -> Self {
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
        Self {
            x: x.to_bits(),
            y: y.to_bits(),
            z: z.to_bits(),
        }
    }
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

/// https://en.wikipedia.org/wiki/Lévy_C_curve
fn build_levy_curve(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();

    let result = TurtleRules::default()
        .add_token('F', TurtleCommand::Forward(100.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(45.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-45.0f64.to_radians())))?
        .add_axiom("F".to_string())?
        .add_rule('F', "+ F - - F +".to_string())?
        .round()?
        .exec(cmd_arg_iterations, Turtle::default())?;

    println!("build_levy_curve render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// build a crooked Lévy C curve in 3d
fn build_levy_curve_3d(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();

    let result = TurtleRules::default()
        .add_token('F', TurtleCommand::Forward(100.0))?
        .add_token(
            '+',
            TurtleCommand::Rotate(
                Rad(45.0f64.to_radians()),
                Rad(0.0f64.to_radians()),
                Rad(0.15f64.to_radians()),
            ),
        )?
        .add_token(
            '-',
            TurtleCommand::Rotate(
                Rad(-45.0f64.to_radians()),
                Rad(0.0f64.to_radians()),
                Rad(-0.15f64.to_radians()),
            ),
        )?
        .add_axiom("F".to_string())?
        .add_rule('F', "+ F - - F +".to_string())?
        .round()?
        .exec(cmd_arg_iterations, Turtle::default())?;

    println!("build_levy_curve_3d render() duration: {:?}", now.elapsed());
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
fn build_sierpinski_triangle(
    cmd_arg_iterations: u8,
) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
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
    println!(
        "build_sierpinski_triangle render() duration: {:?}",
        now.elapsed()
    );
    Ok(result)
}

/// Algorithmic_botany, page 11 (http://algorithmicbotany.org/papers/#abop)
fn build_sierpinski_gasket(
    cmd_arg_iterations: u8,
) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
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
    println!(
        "build_sierpinski_gasket render() duration: {:?}",
        now.elapsed()
    );
    Ok(result)
}

fn build_sierpinski_gasket_3d(
    cmd_arg_iterations: u8,
) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('R', TurtleCommand::Forward(200.0))?
        .add_token('L', TurtleCommand::Forward(200.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(60.0f64.to_radians())))?
        .add_token(
            '-',
            TurtleCommand::Rotate(
                Rad(-61.0f64.to_radians()),
                Rad(0.0f64.to_radians()),
                Rad(4.0f64.to_radians()),
            ),
        )?
        .add_axiom("R".to_string())?
        .add_rule('R', " L - R - L".to_string())?
        .add_rule('L', " R + L + R".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!(
        "build_sierpinski_gasket_3d render() duration: {:?}",
        now.elapsed()
    );
    Ok(result)
}

/// Algorithmic_botany, page 12 (http://algorithmicbotany.org/papers/#abop)
fn build_gosper_curve(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
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
    println!("build_gosper_curve render() duration: {:?}", now.elapsed());
    Ok(result)
}

fn build_gosper_curve_3d(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let f = Box::new(move |mut p: cgmath::Point3<f64>| -> cgmath::Point3<f64> {
        p.z = p.x.sin() + p.y.sin();
        p
    });
    let result = TurtleRules::default()
        .add_token('R', TurtleCommand::ForwardF(1.0, f.clone()))?
        .add_token('L', TurtleCommand::ForwardF(1.0, f))?
        .add_token('+', TurtleCommand::Yaw(Rad(60.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-60.0f64.to_radians())))?
        .add_axiom("L".to_string())?
        .add_rule('L', "L+R++R-L--LL-R+".to_string())?
        .add_rule('R', "-L+RR++R+L--L-R".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!(
        "build_gosper_curve_3d render() duration: {:?}",
        now.elapsed()
    );
    Ok(result)
}

/// Algorithmic_botany, page 9 (http://algorithmicbotany.org/papers/#abop)
fn build_koch_curve(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('F', TurtleCommand::Forward(30.0))?
        .add_token('+', TurtleCommand::Yaw(Rad(90.0f64.to_radians())))?
        .add_token('-', TurtleCommand::Yaw(Rad(-90.0f64.to_radians())))?
        .add_axiom("F".to_string())?
        .add_rule('F', " F + F - F - F + F".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("build_koch_curve render() duration: {:?}", now.elapsed());
    Ok(result)
}

fn build_koch_curve_3d(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
    let now = time::Instant::now();
    let result = TurtleRules::default()
        .add_token('F', TurtleCommand::Forward(30.0))?
        .add_token('+', TurtleCommand::Pitch(Rad(90.0f64.to_radians())))?
        .add_token(
            '-',
            TurtleCommand::Rotate(
                Rad(40.0f64.to_radians()),
                Rad(-90.0f64.to_radians()),
                Rad(0.0f64.to_radians()),
            ),
        )?
        .add_axiom("F".to_string())?
        .add_rule('F', " F + F - F - F + F".to_string())?
        .exec(cmd_arg_iterations, Turtle::default())?;
    println!("build_koch_curve_3d render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Algorithmic_botany, page 9 (http://algorithmicbotany.org/papers/#abop)
fn build_quadratic_koch_curve_island(
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
        "build_quadratic_koch_curve_island render() duration: {:?}",
        now.elapsed()
    );
    Ok(result)
}

fn build_quadratic_koch_curve_island_3d(
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
        "build_quadratic_koch_curve_island_3d render() duration: {:?}",
        now.elapsed()
    );
    Ok(result)
}

/// https://en.wikipedia.org/wiki/L-system#Examples_of_L-systems
fn build_fractal_binary_tree(
    cmd_arg_iterations: u8,
) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
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
    println!(
        "build_fractal_binary_tree render() duration: {:?}",
        now.elapsed()
    );

    Ok(result)
}

fn build_fractal_binary_tree_3d(
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
    println!(
        "build_fractal_binary_tree_3d render() duration: {:?}",
        now.elapsed()
    );

    Ok(result)
}

/// Rather messy implementation of a text parser for a custom turtle builder.
/// It takes a lot of shortcuts, all parenthesis are ignored, it assumes there is one command per
/// line etc. etc.
fn build_custom_turtle(
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
    println!("build_custom_turtle render() duration: {:?}", now.elapsed());

    Ok(result)
}

/// https://en.wikipedia.org/wiki/L-system#Examples_of_L-systems
fn build_fractal_plant(cmd_arg_iterations: u8) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
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

    println!("build_fractal_plant render() duration: {:?}", now.elapsed());
    Ok(result)
}

/// Algorithmic_botany, page 20 (http://algorithmicbotany.org/papers/#abop)
fn build_hilbert_curve_3d(
    cmd_arg_iterations: u8,
) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
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

    println!(
        "build_fractal_binary_tree_3d render() duration: {:?}",
        now.elapsed()
    );
    Ok(result)
}

/// Build the return model
/// lines contains a vector of (<first vertex index>,<a list of points><last vertex index>)
fn build_output_bp_model(
    _a_command: &PB_Command,
    lines: Vec<[cgmath::Point3<f64>; 2]>,
) -> Result<PB_Model, TBError> {
    // vertex de-duplication mechanism
    let mut vertices_map = ahash::AHashMap::<F64Key, usize>::new();
    // make sure to not add the same edge twice, set key is sorted (lower,higher)
    let mut dup_edges = ahash::AHashSet::<(usize, usize)>::new();

    // Default capacity is probably a little bit too low
    let mut pb_vertices = Vec::<PB_Vertex>::with_capacity(lines.len());
    let mut pb_faces = Vec::<PB_Face>::with_capacity(lines.len());
    let mut src_aabb = linestring::linestring_3d::Aabb3::default();

    for [p0, p1] in lines.into_iter() {
        let key = F64Key::new(&p0);
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

        let key = F64Key::new(&p1);
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

/// Run the lsystems command
pub(crate) fn command(
    a_command: PB_Command,
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
        "FRACTAL_BINARY_TREE" => build_fractal_binary_tree(cmd_arg_iterations),
        "FRACTAL_BINARY_TREE_3D" => build_fractal_binary_tree_3d(cmd_arg_iterations),
        "FRACTAL_PLANT" => build_fractal_plant(cmd_arg_iterations),
        "SIERPINSKI_TRIANGLE" => build_sierpinski_triangle(cmd_arg_iterations),
        "SIERPINSKI_GASKET" => build_sierpinski_gasket(cmd_arg_iterations),
        "SIERPINSKI_GASKET_3D" => build_sierpinski_gasket_3d(cmd_arg_iterations),
        "KOCH_CURVE" => build_koch_curve(cmd_arg_iterations),
        "KOCH_CURVE_3D" => build_koch_curve_3d(cmd_arg_iterations),
        "GOSPER_CURVE" => build_gosper_curve(cmd_arg_iterations),
        "GOSPER_CURVE_3D" => build_gosper_curve_3d(cmd_arg_iterations),
        "KOCH_CURVE_ISLAND" => build_quadratic_koch_curve_island(cmd_arg_iterations),
        "KOCH_CURVE_ISLAND_3D" => build_quadratic_koch_curve_island_3d(cmd_arg_iterations),
        "CUSTOM_TURTLE" => build_custom_turtle(cmd_arg_iterations, cmd_custom_turtle),
        "HILBERT_CURVE_3D" => build_hilbert_curve_3d(cmd_arg_iterations),
        "LEVY_C_CURVE" => build_levy_curve(cmd_arg_iterations),
        "LEVY_C_CURVE_3D" => build_levy_curve_3d(cmd_arg_iterations),
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
