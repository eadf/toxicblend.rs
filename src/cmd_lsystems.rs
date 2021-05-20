use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex as PB_Vertex;
use dcc_lsystem::renderer::DataRendererOptions;
use dcc_lsystem::renderer::Renderer;
use dcc_lsystem::turtle::{TurtleAction, TurtleLSystemBuilder};
use dcc_lsystem::LSystemBuilder;
use logos::Logos;
use rand::seq::SliceRandom;
use rand::{thread_rng, Rng};
use std::collections::HashMap;
use std::time;

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

fn valid_rule(rule: &[&str]) -> bool {
    if rule.is_empty() {
        return false;
    }

    let r = rule.join("");

    if r.contains("+-") || !r.contains('+') {
        return false;
    }

    let mut level = 0;

    for c in rule {
        if c == &"+" {
            level += 1;
        } else if c == &"-" {
            if level == 0 {
                return false;
            }
            level -= 1;
        }
    }

    level == 0
}

/// Code directly copy & pasted from dcc-lsystem examples
fn random_fractal_generator(
    cmd_arg_iterations: usize,
) -> Result<Vec<(i32, i32, i32, i32)>, TBError> {
    let mut rv = Vec::<(i32, i32, i32, i32)>::new();
    let now = time::Instant::now();

    'processing: loop {
        // generate random axiom out of L, R, F, X, Y
        // and random rules for X, Y
        let mut rng = thread_rng();

        // generate a random axiom
        let axiom_length = rng.gen_range(0..=2);
        let mut axiom = vec!["X"];
        let choices = ["L", "R", "F", "X", "Y"];
        let weighted_choices = [
            ("F", rng.gen_range(1..=8)),
            ("X", rng.gen_range(2..=4)),
            ("Y", rng.gen_range(2..=4)),
            ("L", rng.gen_range(2..=6)),
            ("R", rng.gen_range(2..=6)),
            ("+", rng.gen_range(4..=8)),
            ("-", rng.gen_range(4..=8)),
        ];

        for _ in 0..axiom_length {
            axiom.push(<&str>::clone(choices.choose(&mut rng).unwrap()));
        }

        // generate a random X rule
        let mut x_rule = Vec::new();

        while !valid_rule(&x_rule) {
            x_rule.clear();

            let x_rule_length = rng.gen_range(4..=10);

            for _ in 0..x_rule_length {
                x_rule.push(<&str>::clone(
                    &weighted_choices
                        .choose_weighted(&mut rng, |item| item.1)
                        .unwrap()
                        .0,
                ));
            }
        }

        let mut y_rule = Vec::new();

        while !valid_rule(&y_rule) {
            y_rule.clear();
            // generate a random Y rule
            let y_rule_length = rng.gen_range(4..=10);

            for _ in 0..y_rule_length {
                y_rule.push(<&str>::clone(
                    &weighted_choices
                        .choose_weighted(&mut rng, |item| item.1)
                        .unwrap()
                        .0,
                ));
            }
        }

        let mut builder = TurtleLSystemBuilder::new();

        // Build our system up
        let _ = builder
            .token("L", TurtleAction::Rotate(25))
            .token("R", TurtleAction::Rotate(-25))
            .token("F", TurtleAction::Forward(100))
            .token("+", TurtleAction::Push)
            .token("-", TurtleAction::Pop)
            .token("X", TurtleAction::Nothing)
            .token("Y", TurtleAction::Nothing)
            .axiom(&axiom.join(" "))
            .rule(format!("X => {}", x_rule.join(" ")).as_str())
            .rule(format!("Y => {}", y_rule.join(" ")).as_str());

        // Consume the builder to construct an LSystem and the associated renderer
        let (mut system, renderer) = builder.finish();
        system.step_by(cmd_arg_iterations);

        let options = DataRendererOptions::default();
        rv.append(&mut renderer.render(&system, &options));

        // totally arbitrary value
        if rv.len() < 500 * cmd_arg_iterations {
            continue 'processing;
        }

        break;
    }
    println!(
        "random_fractal_generator render() duration: {:?} lines:{}",
        now.elapsed(),
        rv.len()
    );

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

/// Rather messy implementation of a text parser for a custom turtle builder.
/// It takes a lot of shortcuts, all parenthesis are ignored, it assumes there is one command per
/// line etc. etc.
fn custom_turtle(
    cmd_arg_iterations: usize,
    cmd_custom_turtle: Option<&String>,
) -> Result<Vec<(i32, i32, i32, i32)>, TBError> {
    #[derive(Debug, PartialEq, Eq)]
    enum ParseTurtleAction {
        Forward,
        Rotate,
        // These 'states' does not carry any value, so they can be executed directly
        //Push,
        //Pop,
        //Nothing,
    }

    #[allow(clippy::upper_case_acronyms)]
    #[derive(Logos, Debug, PartialEq)]
    enum ParseToken {
        #[regex("\\.?token")]
        Token,

        #[regex("\\.?axiom")]
        Axiom,

        #[regex("\\.?rule")]
        Rule,

        #[regex("\\.?rotate")]
        Rotate,

        #[token("TurtleAction::Forward")]
        TurtleActionForward,

        #[token("TurtleAction::Rotate")]
        TurtleActionRotate,

        #[token("TurtleAction::Nothing")]
        TurtleActionNothing,

        #[token("TurtleAction::Pop")]
        TurtleActionPop,

        #[token("TurtleAction::Push")]
        TurtleActionPush,

        #[token("\n")]
        EOL,

        #[regex("-?[0-9]+(.[0-9]+)?")]
        Number,

        #[regex("\"[=>a-zA-Z 0-9\\+\\-\\+\\]\\[]+\"")]
        QuotedText,

        #[regex(r"[ \t\f(),\?;]+", logos::skip)]
        Skip,

        #[error]
        Error,
    }

    #[derive(Debug, PartialEq, Eq)]
    enum ParseState {
        Start,
        Token(Option<String>, Option<ParseTurtleAction>),
        Axiom,
        Rule,
        Rotate,
    }

    if cmd_custom_turtle.is_none() {
        return Err(TBError::InvalidInputData(
            "Custom turtle text is empty".to_string(),
        ));
    }
    let cmd_custom_turtle = cmd_custom_turtle.unwrap();

    let mut builder = TurtleLSystemBuilder::new();
    println!("Will try to parse custom_turtle : {:?}", cmd_custom_turtle);

    let mut lex = ParseToken::lexer(cmd_custom_turtle);
    let mut state = ParseState::Start;
    let mut line = 0_i32;

    while let Some(token) = lex.next() {
        match token {
            ParseToken::Token => {
                if state != ParseState::Start {
                    return Err(TBError::ParseError(format!(
                        "Expected to be in Start state, was in state:{:?} when reading:{} at line {}.",
                        state, lex.slice(), line
                    )));
                }
                state = ParseState::Token(None, None);
            }
            ParseToken::Axiom => {
                if state != ParseState::Start {
                    return Err(TBError::ParseError(format!(
                        "Expected to be in Start state, was in state:{:?} when reading:{} at line {}.",
                        state, lex.slice(), line
                    )));
                }
                state = ParseState::Axiom;
            }
            ParseToken::Rule => {
                if state != ParseState::Start {
                    return Err(TBError::ParseError(format!(
                        "Expected to be in Start state, was in state:{:?} when reading:{} at line {}.",
                        state, lex.slice(), line
                    )));
                }
                state = ParseState::Rule;
            }
            ParseToken::Rotate => {
                if state != ParseState::Start {
                    return Err(TBError::ParseError(format!(
                        "Expected to be in Start state, was in state:{:?} when reading:{} at line {}.",
                        state, lex.slice(), line
                    )));
                }
                state = ParseState::Rotate;
            }
            ParseToken::QuotedText => {
                let text = &lex.slice()[1..lex.slice().len() - 1];
                match state {
                    ParseState::Axiom => {
                        println!("Got .axiom(\"{}\")", text);
                        let _ = builder.axiom(text);
                        state = ParseState::Start;
                    }
                    ParseState::Token(None, None) => {
                        state = ParseState::Token(Some(text.to_string()), None);
                    }
                    ParseState::Rule => {
                        println!("Got .rule(\"{}\")", text);
                        let _ = builder.rule(text);
                        state = ParseState::Start;
                    }
                    _ => {
                        return Err(TBError::ParseError(format!(
                            "Bad state for QuotedText:{:?} at line {}",
                            state, line
                        )));
                    }
                }
            }
            ParseToken::TurtleActionForward => match state {
                ParseState::Token(Some(text), None) => {
                    state = ParseState::Token(Some(text), Some(ParseTurtleAction::Forward));
                }
                _ => {
                    return Err(TBError::ParseError(format!(
                        "Bad state for TurtleActionForward:{:?} at line {}",
                        state, line
                    )));
                }
            },
            ParseToken::TurtleActionRotate => match state {
                ParseState::Token(Some(text), None) => {
                    state = ParseState::Token(Some(text), Some(ParseTurtleAction::Rotate));
                }
                _ => {
                    return Err(TBError::ParseError(format!(
                        "Bad state for TurtleActionRotate:{:?} at line {}",
                        state, line
                    )));
                }
            },
            ParseToken::TurtleActionNothing => match state {
                ParseState::Token(Some(text), None) => {
                    println!("Got .token(\"{}\", TurtleAction::Nothing)", text);
                    let _ = builder.token(text, TurtleAction::Nothing);
                    state = ParseState::Start;
                }
                _ => {
                    return Err(TBError::ParseError(format!(
                        "Bad state for TurtleActionNothing:{:?} at line {}",
                        state, line
                    )));
                }
            },
            ParseToken::TurtleActionPop => match state {
                ParseState::Token(Some(text), None) => {
                    println!("Got .token(\"{}\", TurtleAction::Pop)", text);
                    let _ = builder.token(text, TurtleAction::Pop);
                    state = ParseState::Start;
                }
                _ => {
                    return Err(TBError::ParseError(format!(
                        "Bad state for TurtleActionPop:{:?} at line {}",
                        state, line
                    )));
                }
            },
            ParseToken::TurtleActionPush => match state {
                ParseState::Token(Some(text), None) => {
                    println!("Got .token(\"{}\", TurtleAction::Push)", text);
                    let _ = builder.token(text, TurtleAction::Push);
                    state = ParseState::Start;
                }
                _ => {
                    return Err(TBError::ParseError(format!(
                        "Bad state for TurtleActionPush:{:?} at line {}",
                        state, line
                    )));
                }
            },
            ParseToken::EOL => {
                line += 1;
                state = ParseState::Start;
            }
            ParseToken::Number => match state {
                ParseState::Token(Some(text), Some(turtle)) => {
                    println!(
                        "Got .token(\"{}\", TurtleAction::{:?}({}))",
                        text,
                        turtle,
                        lex.slice()
                    );
                    let value = lex.slice().parse::<i32>().map_err(|e| {
                        TBError::ParseError(format!(
                            "Could not parse number :{} at line {}. {:?}",
                            lex.slice(),
                            line,
                            e
                        ))
                    })?;
                    let _ = builder.token(
                        text,
                        match turtle {
                            ParseTurtleAction::Rotate => TurtleAction::Rotate(value),
                            ParseTurtleAction::Forward => TurtleAction::Forward(value),
                        },
                    );
                    state = ParseState::Start;
                }
                ParseState::Rotate => {
                    println!("Got .rotate({})", lex.slice());
                    let _ = builder.rotate(lex.slice().parse::<i32>().map_err(|e| {
                        TBError::ParseError(format!(
                            "Could not parse number :{} at line {}. {:?}",
                            lex.slice(),
                            line,
                            e
                        ))
                    })?);
                    state = ParseState::Start;
                }
                _ => {
                    return Err(TBError::ParseError(format!(
                        "Bad state for Integer:{:?} at line {}",
                        state, line
                    )));
                }
            },
            _ => {
                return Err(TBError::ParseError(format!(
                    "Bad token: {:?} at line {}",
                    lex.slice(),
                    line
                )));
            }
        }
    }

    let (mut system, renderer) = builder.finish();
    system.step_by(cmd_arg_iterations);

    let now = time::Instant::now();
    let options = DataRendererOptions::default();
    let rv = renderer.render(&system, &options);
    println!("custom_turtle render() duration: {:?}", now.elapsed());

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
    if !pb_vertices.is_empty() {
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
        let scale = if scale.is_finite() { scale } else { 1.0 };
        //println!("Aabb:{:?} to center: {:?}, scale: {:?}", src_aabb, to_center, scale);
        // center and scale the result so that no axis is larger than 3 'units'
        for v in pb_vertices.iter_mut() {
            v.x = (v.x + to_center.x) * scale;
            v.y = (v.y + to_center.y) * scale;
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
        "CANTOR_SETS" => build_cantor_sets(cmd_arg_iterations),
        "FRACTAL_BINARY_TREE" => fractal_binary_tree(cmd_arg_iterations),
        "FRACTAL_PLANT" => fractal_plant(cmd_arg_iterations),
        "SIERPINSKI_TRIANGLE" => sierpinski_triangle(cmd_arg_iterations),
        "SIERPINSKI_ARROWHEAD" => sierpinski_arrowhead(cmd_arg_iterations),
        "KOCH_CURVE" => koch_curve(cmd_arg_iterations),
        "RANDOM_FRACTAL_GENERATOR" => random_fractal_generator(cmd_arg_iterations),
        "CUSTOM_TURTLE" => custom_turtle(cmd_arg_iterations, cmd_custom_turtle),
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
