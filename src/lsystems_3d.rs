use super::TBError;
use cgmath::Vector3;
use cgmath::{relative_ne, Basis3, InnerSpace, Rad, Rotation, Rotation3};
use logos::Logos;
use std::f64;

#[derive(Debug, Clone)]
struct Heading {
    heading: Vector3<f64>,
    up: Vector3<f64>,
}

impl Default for Heading {
    fn default() -> Self {
        Self {
            heading: Vector3::<f64>::unit_y(),
            up: Vector3::<f64>::unit_z(),
        }
    }
}

impl Heading {
    #[allow(dead_code)]
    fn new(forward: Vector3<f64>, up: Vector3<f64>) -> Self {
        Self {
            heading: forward,
            up,
        }
    }

    // rotate around 'forward' or longitudinal axis
    fn roll(&self, angle: Rad<f64>) -> Heading {
        let rot: Basis3<f64> = Rotation3::from_axis_angle(self.heading, angle);
        //let rv =
        Self {
            heading: self.heading,
            up: rot.rotate_vector(self.up).normalize(),
        }
        //println!("Roll angle:{}rad heading:{:?}->{:?}  up:{:?}->{:?}", angle.0, self.heading, rv.heading, self.up, rv.up);
        //rv
    }

    // rotate around 'up' or vertical axis
    fn yaw(&self, angle: Rad<f64>) -> Heading {
        let rot: Basis3<f64> = Rotation3::from_axis_angle(self.up, angle);
        //let rv =
        Self {
            heading: rot.rotate_vector(self.heading).normalize(),
            up: self.up,
        }
        //println!("Yaw angle:{}rad heading:{:?}->{:?}  up:{:?}->{:?}", angle.0, self.heading, rv.heading, self.up, rv.up);
        //rv
    }

    // rotate around axis perpendicular to 'up' and 'forward' - i.e. lateral/traverse axis
    fn pitch(&self, angle: Rad<f64>) -> Heading {
        let pitch_v = self.heading.cross(self.up).normalize();
        let rot: Basis3<f64> = Rotation3::from_axis_angle(pitch_v, angle);
        //let rv =
        Self {
            heading: rot.rotate_vector(self.heading).normalize(),
            up: rot.rotate_vector(self.up).normalize(),
        }
        //println!("Pitch angle:{}rad heading:{:?}->{:?}  up:{:?}->{:?}", angle.0, self.heading, rv.heading, self.up, rv.up);
        //rv
    }

    // roll, yaw and pitch in order
    fn rotate(&self, roll: Rad<f64>, yaw: Rad<f64>, pitch: Rad<f64>) -> Heading {
        self.roll(roll).yaw(yaw).pitch(pitch)
    }
}

pub struct Turtle {
    heading: Heading,
    position: cgmath::Point3<f64>,
    stack: Vec<(Heading, cgmath::Point3<f64>)>,
    result: Vec<[cgmath::Point3<f64>; 2]>,
    pen_up: bool,
}

impl Default for Turtle {
    fn default() -> Self {
        Self {
            heading: Heading::default(),
            position: cgmath::Point3::<f64>::new(0., 0., 0.),
            result: Vec::new(),
            stack: Vec::new(),
            pen_up: false,
        }
    }
}

impl Turtle {
    fn apply(&mut self, action: &TurtleCommand) -> Result<(), TBError> {
        match action {
            TurtleCommand::Nop => {}
            TurtleCommand::Yaw(angle) => self.heading = self.heading.yaw(*angle),
            TurtleCommand::Pitch(angle) => self.heading = self.heading.pitch(*angle),
            TurtleCommand::Roll(angle) => self.heading = self.heading.roll(*angle),
            TurtleCommand::Rotate(yaw, pitch, roll) => {
                self.heading = self.heading.rotate(*yaw, *pitch, *roll)
            }
            TurtleCommand::Forward(distance) => {
                let l1 = self.position;
                self.position += self.heading.heading * *distance;
                //println!("Self.heading: {:?}, Self.heading*distance={:?}", self.heading.heading, self.heading.heading * *distance);
                if !self.pen_up {
                    self.result.push([l1, self.position]);
                }
            }
            TurtleCommand::PenUp => self.pen_up = true,
            TurtleCommand::PenDown => self.pen_up = false,
            TurtleCommand::Push => self.stack.push((self.heading.clone(), self.position)),
            TurtleCommand::Pop => {
                if let Some(pop) = self.stack.pop() {
                    self.heading = pop.0;
                    self.position = pop.1;
                } else {
                    return Err(TBError::LSystems3D("Could not pop stack".to_string()));
                }
            }
        };
        Ok(())
    }
}

#[allow(dead_code)]
pub enum TurtleCommand {
    Nop,
    Forward(f64),
    Roll(Rad<f64>),
    Pitch(Rad<f64>),
    Yaw(Rad<f64>),
    // yaw, pitch, roll
    Rotate(Rad<f64>, Rad<f64>, Rad<f64>),
    PenUp,
    PenDown,
    Push,
    Pop,
}

#[derive(Default)]
pub struct TurtleRules {
    rules: ahash::AHashMap<char, String>,
    axiom: String,
    tokens: ahash::AHashMap<char, TurtleCommand>,
    yaw: Option<Rad<f64>>,
    pitch: Option<Rad<f64>>,
    roll: Option<Rad<f64>>,
}

impl TurtleRules {
    pub fn add_token(&mut self, token: char, ta: TurtleCommand) -> Result<&mut Self, TBError> {
        if self.tokens.contains_key(&token) {
            return Err(TBError::LSystems3D(format!(
                "already contain the token {}",
                token
            )));
        }
        let _ = self.tokens.insert(token, ta);
        Ok(self)
    }

    pub fn add_axiom(&mut self, axiom: String) -> Result<&mut Self, TBError> {
        if !self.axiom.is_empty() {
            return Err(TBError::LSystems3D(format!(
                "already contains an axiom {}",
                axiom
            )));
        }
        self.axiom = axiom;
        Ok(self)
    }

    pub fn add_rule(&mut self, rule: String) -> Result<&mut Self, TBError> {
        if rule.len() < 6 {
            return Err(TBError::LSystems3D(format!("Rule too short {}", rule)));
        }
        let id = rule[0..1]
            .chars()
            .next()
            .ok_or_else(|| TBError::LSystems3D(format!("Could not find rule key {}", rule)))?;
        let assign = &rule[1..5];
        if assign != " => " {
            println!("assign = '{}'", assign);
            return Err(TBError::LSystems3D(format!("Rule not understood {}", rule)));
        }
        let rule = rule[4..rule.len()].to_string().replace(" ", "");

        println!("Adding rule '{}' => '{}'", id, &rule);
        if self.rules.insert(id, rule).is_some() {
            return Err(TBError::LSystems3D(format!(
                "Rule {} overwriting previous rule",
                id
            )));
        }
        Ok(self)
    }

    /// Set the initial heading of the, not yet known, turtle
    pub fn rotate(
        &mut self,
        yaw: Rad<f64>,
        pitch: Rad<f64>,
        roll: Rad<f64>,
    ) -> Result<&mut Self, TBError> {
        if relative_ne!(yaw.0, 0.0) {
            self.yaw = Some(yaw);
        } else {
            self.yaw = None;
        }
        if relative_ne!(pitch.0, 0.0) {
            self.pitch = Some(pitch);
        } else {
            self.pitch = None;
        }
        if relative_ne!(roll.0, 0.0) {
            self.roll = Some(roll);
        } else {
            self.roll = None;
        }
        Ok(self)
    }

    /// Expands the rules over the axiom 'n' times
    fn expand(&self, n: u8) -> Result<Vec<char>, TBError> {
        let mut rv: Vec<char> = self.axiom.chars().collect();
        for _ in 0..n {
            let mut tmp = Vec::<char>::with_capacity(rv.len() * 2);
            for v in rv.iter() {
                if v == &' ' {
                    continue;
                } else if let Some(rule) = self.rules.get(&v) {
                    // it was a rule
                    tmp.append(&mut rule.chars().collect());
                } else {
                    // maybe a token?
                    let _ = self.tokens.get(&v).ok_or_else(|| {
                        eprintln!("tokens: {:?}", self.tokens.keys());
                        eprintln!("rules: {:?}", self.rules.keys());
                        TBError::LSystems3D(format!("Could not find rule or token:'{}'", &v))
                    })?;
                    // do not expand tokens
                    tmp.push(*v);
                }
            }
            rv = tmp;
        }
        Ok(rv)
    }

    /// sets the axioms, rules and tokens from a text string.
    pub fn parse(&mut self, cmd_custom_turtle: &str) -> Result<&mut Self, TBError> {
        #[derive(Debug, PartialEq, Eq)]
        enum ParseTurtleAction {
            Forward,
            Yaw,
            Pitch,
            Roll,
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

            #[regex("\\.?yaw")]
            Yaw,

            #[regex("\\.?pitch")]
            Pitch,

            #[regex("\\.?roll")]
            Roll,

            #[token("Turtle::Forward")]
            TurtleActionForward,

            #[token("Turtle::Yaw")]
            TurtleActionYaw,

            #[token("Turtle::Pitch")]
            TurtleActionPitch,

            #[token("Turtle::Roll")]
            TurtleActionRoll,

            #[token("Turtle::Nop")]
            TurtleActionNop,

            #[token("Turtle::Nothing")]
            TurtleActionNothing,

            #[token("Turtle::Pop")]
            TurtleActionPop,

            #[token("Turtle::Push")]
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

        #[derive(Debug, PartialEq)]
        enum ParseState {
            Start,
            Token(Option<char>, Option<ParseTurtleAction>),
            Axiom,
            Rule,
            Yaw,
            Rotate(Option<Rad<f64>>, Option<Rad<f64>>, Option<Rad<f64>>),
        }

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
                ParseToken::Yaw => {
                    if state != ParseState::Start {
                        return Err(TBError::ParseError(format!(
                            "Expected to be in Start state, was in state:{:?} when reading:{} at line {}.",
                            state, lex.slice(), line
                        )));
                    }
                    state = ParseState::Yaw;
                }
                ParseToken::QuotedText => {
                    let text = &lex.slice()[1..lex.slice().len() - 1];
                    match state {
                        ParseState::Axiom => {
                            println!("Got .add_axiom(\"{}\")", text);
                            let _ = self.add_axiom(text.to_string());
                            state = ParseState::Start;
                        }
                        ParseState::Token(None, None) => {
                            state = ParseState::Token(
                                Some(text.chars().next().ok_or_else(|| {
                                    TBError::ParseError(format!(
                                        "Could not get token id as line {}",
                                        line
                                    ))
                                })?),
                                None,
                            );
                        }
                        ParseState::Rule => {
                            println!("Got .add_rule(\"{}\")", text);
                            let _ = self.add_rule(text.to_string());
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
                ParseToken::TurtleActionYaw => match state {
                    ParseState::Token(Some(text), None) => {
                        state = ParseState::Token(Some(text), Some(ParseTurtleAction::Yaw));
                    }
                    _ => {
                        return Err(TBError::ParseError(format!(
                            "Bad state for TurtleActionYaw:{:?} at line {}",
                            state, line
                        )));
                    }
                },
                ParseToken::TurtleActionPitch => match state {
                    ParseState::Token(Some(text), None) => {
                        state = ParseState::Token(Some(text), Some(ParseTurtleAction::Pitch));
                    }
                    _ => {
                        return Err(TBError::ParseError(format!(
                            "Bad state for TurtleActionPitch:{:?} at line {}",
                            state, line
                        )));
                    }
                },
                ParseToken::TurtleActionRoll => match state {
                    ParseState::Token(Some(text), None) => {
                        state = ParseState::Token(Some(text), Some(ParseTurtleAction::Roll));
                    }
                    _ => {
                        return Err(TBError::ParseError(format!(
                            "Bad state for TurtleActionRoll:{:?} at line {}",
                            state, line
                        )));
                    }
                },
                ParseToken::TurtleActionNop | ParseToken::TurtleActionNothing => match state {
                    ParseState::Token(Some(text), None) => {
                        println!("Got .add_token(\"{}\", TurtleAction::Nop)", text);
                        let _ = self.add_token(text, TurtleCommand::Nop);
                        state = ParseState::Start;
                    }
                    _ => {
                        return Err(TBError::ParseError(format!(
                            "Bad state for TurtleActionNop:{:?} at line {}",
                            state, line
                        )));
                    }
                },
                ParseToken::TurtleActionPop => match state {
                    ParseState::Token(Some(text), None) => {
                        println!("Got .add_token(\"{}\", TurtleAction::Pop)", text);
                        let _ = self.add_token(text, TurtleCommand::Pop);
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
                        println!("Got .add_token(\"{}\", TurtleAction::Push)", text);
                        let _ = self.add_token(text, TurtleCommand::Push);
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
                ParseToken::Rotate => {
                    state = ParseState::Rotate(None, None, None);
                }
                ParseToken::Number => {
                    let value = lex.slice().parse::<f64>().map_err(|e| {
                        TBError::ParseError(format!(
                            "Could not parse number :{} at line {}. {:?}",
                            lex.slice(),
                            line,
                            e
                        ))
                    })?;

                    match state {
                        ParseState::Token(Some(text), Some(turtle)) => {
                            println!(
                                "Got .add_token(\"{}\", TurtleAction::{:?}({}))",
                                text,
                                turtle,
                                lex.slice()
                            );

                            let _ = self.add_token(
                                text,
                                match turtle {
                                    ParseTurtleAction::Yaw => {
                                        TurtleCommand::Yaw(Rad(value.to_radians()))
                                    }
                                    ParseTurtleAction::Pitch => {
                                        TurtleCommand::Pitch(Rad(value.to_radians()))
                                    }
                                    ParseTurtleAction::Roll => {
                                        TurtleCommand::Roll(Rad(value.to_radians()))
                                    }
                                    ParseTurtleAction::Forward => TurtleCommand::Forward(value),
                                },
                            );
                            state = ParseState::Start;
                        }
                        ParseState::Rotate(None, None, None) => {
                            state = ParseState::Rotate(Some(Rad(value.to_radians())), None, None);
                        }
                        ParseState::Rotate(Some(yaw), None, None) => {
                            state =
                                ParseState::Rotate(Some(yaw), Some(Rad(value.to_radians())), None);
                        }
                        ParseState::Rotate(Some(yaw), Some(pitch), None) => {
                            let roll = Rad(value.to_radians());
                            println!("Got .rotate({}, {}, {})", yaw.0, pitch.0, roll.0);
                            let _ = self.rotate(yaw, pitch, roll);
                            state = ParseState::Start;
                        }
                        _ => {
                            return Err(TBError::ParseError(format!(
                                "Bad state for Integer:{:?} at line {}",
                                state, line
                            )));
                        }
                    }
                }
                _ => {
                    return Err(TBError::ParseError(format!(
                        "Bad token: {:?} at line {}",
                        lex.slice(),
                        line
                    )));
                }
            }
        }
        Ok(self)
    }

    /// expands the rules and run the turtle over the result.
    pub fn exec(
        &self,
        n: u8,
        mut turtle: Turtle,
    ) -> Result<Vec<[cgmath::Point3<f64>; 2]>, TBError> {
        let path = self.expand(n)?;

        // Apply initial rotations
        if let Some(yaw) = self.yaw {
            turtle.apply(&TurtleCommand::Yaw(yaw))?;
        }
        if let Some(roll) = self.roll {
            turtle.apply(&TurtleCommand::Roll(roll))?;
        }
        if let Some(pitch) = self.pitch {
            turtle.apply(&TurtleCommand::Pitch(pitch))?;
        }

        for step in path.into_iter() {
            if step == ' ' {
                continue;
            }
            let action = self.tokens.get(&step).ok_or_else(|| {
                eprintln!("tokens: {:?}", self.tokens.keys());
                eprintln!("rules: {:?}", self.rules.keys());
                TBError::LSystems3D(format!("Could not find rule or token:'{}'", &step))
            })?;
            turtle.apply(action)?;
        }
        Ok(turtle.result)
    }
}
