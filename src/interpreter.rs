///! G-code interpreter + parser
///! This module uses the following units:
///!   length: mm
///!   mass: g
///!   inertia: g·mm^2
use super::segments::{ArcSeg, LineSeg, Segment};
use gcode::{Mnemonic, parse};
use nalgebra::Vector3;
use std::f32::consts::PI;

#[derive(Debug)]
struct Extruder {
    /// the nominal E‑position reported by the G‑code
    pos: f32,
    /// the highest E‑position we’ve ever extruded (used to ignore retraction pay‑back)
    max_pos: f32,
    /// false=M82 (absolute), true=M83 (relative)
    relative: bool,
}

impl Extruder {
    fn new() -> Self {
        Self {
            pos: 0.0,
            max_pos: 0.0,
            relative: false,
        }
    }

    fn consume(&mut self, word: Option<f32>) -> f32 {
        match (self.relative, word) {
            // ABSOLUTE mode: word is the new absolute E
            (false, Some(abs)) => {
                self.pos = abs;
                if abs > self.max_pos {
                    let d = abs - self.max_pos;
                    self.max_pos = abs;
                    d
                } else {
                    0.0
                }
            }

            // RELATIVE mode: word is ΔE
            (true, Some(rel)) => {
                // step the current E‑pos (can go down on retract)
                self.pos += rel;
                // if we’ve extruded past our previous high‑water mark, count it
                if self.pos > self.max_pos {
                    let extruded = self.pos - self.max_pos;
                    self.max_pos = self.pos;
                    extruded
                } else {
                    // any retract or zero‑move ⇒ no new extrusion
                    0.0
                }
            }

            // no E word → nothing extruded
            _ => 0.0,
        }
    }

    /// call this on G92 E… to zero both counters
    fn reset(&mut self, e: f32) {
        self.pos = e;
        self.max_pos = e;
    }
}
#[derive(Debug)]
struct Filament {
    diameter: f32, // mm
    density: f32,  // g/mm^3
}
#[derive(Debug)]
struct Machine {
    pos: Vector3<f32>,
    xyz_rel: bool,
    extr: Extruder,
    filament: Filament,
}
impl Machine {
    /// Define a new machine
    ///
    /// The filament density is in g/mm^3
    /// The filament diameter is in mm
    fn new(diameter: f32, density: f32) -> Self {
        Self {
            pos: Vector3::zeros(),
            xyz_rel: false,
            extr: Extruder::new(),
            filament: Filament {
                diameter,
                density: density / 1000.0, // g/mm^3
            },
        }
    }
    fn next_coord(cur: f32, word: Option<f32>, rel: bool) -> f32 {
        match (rel, word) {
            (false, Some(v)) => v,
            (true, Some(v)) => cur + v,
            _ => cur,
        }
    }
}

pub struct Interpreter {
    m: Machine,
}

impl Interpreter {
    pub fn new(filament_diameter: f32, filament_density: f32) -> Self {
        Self {
            m: Machine::new(filament_diameter, filament_density),
        }
    }

    /// Feed one parsed command; returns at most one Segment
    pub fn process_cmd(&mut self, cmd: gcode::GCode) -> Option<Box<dyn Segment>> {
        match cmd.mnemonic() {
            Mnemonic::General => match cmd.major_number() {
                90 => {
                    self.m.xyz_rel = false;
                    return None;
                } // G90
                91 => {
                    self.m.xyz_rel = true;
                    return None;
                } // G91
                92 => {
                    // G92 → reset coordinates
                    // zero out whichever axes are present:
                    if let Some(x) = cmd.value_for('X') {
                        self.m.pos.x = x;
                    }
                    if let Some(y) = cmd.value_for('Y') {
                        self.m.pos.y = y;
                    }
                    if let Some(z) = cmd.value_for('Z') {
                        self.m.pos.z = z;
                    }
                    // reset extruder position too:
                    if let Some(e) = cmd.value_for('E') {
                        self.m.extr.reset(e);
                    }
                    None
                }
                0 | 1 | 2 | 3 => self.handle_motion(cmd),
                _ => None,
            },
            Mnemonic::Miscellaneous => match cmd.major_number() {
                82 => {
                    self.m.extr.relative = false;
                    None
                } // M82
                83 => {
                    self.m.extr.relative = true;
                    None
                } // M83
                _ => None,
            },
            _ => None,
        }
    }

    fn handle_motion(&mut self, cmd: gcode::GCode) -> Option<Box<dyn Segment>> {
        let xr = self.m.xyz_rel;
        let tgt_x = Machine::next_coord(self.m.pos.x, cmd.value_for('X'), xr);
        let tgt_y = Machine::next_coord(self.m.pos.y, cmd.value_for('Y'), xr);
        let tgt_z = Machine::next_coord(self.m.pos.z, cmd.value_for('Z'), xr);
        let tgt = Vector3::new(tgt_x, tgt_y, tgt_z);

        let de = self.m.extr.consume(cmd.value_for('E'));
        let mass = de * PI * self.m.filament.diameter.powi(2) / 4.0 * self.m.filament.density;

        let seg = match cmd.major_number() {
            0 | 1 if de > 0.0 => Some(Box::new(LineSeg {
                start: self.m.pos,
                end: tgt,
                mass,
            }) as Box<dyn Segment>),

            2 | 3 if de > 0.0 => {
                let i = cmd.value_for('I').unwrap_or(0.0);
                let j = cmd.value_for('J').unwrap_or(0.0);
                let ctr = Vector3::new(self.m.pos.x + i, self.m.pos.y + j, self.m.pos.z);
                let s_rel = self.m.pos - ctr;
                let e_rel = tgt - ctr;

                let θ0 = s_rel.y.atan2(s_rel.x);
                let θ1 = e_rel.y.atan2(e_rel.x);
                let cw = cmd.major_number() == 2;
                let mut dθ = if cw { θ0 - θ1 } else { θ1 - θ0 };
                if dθ <= 0.0 {
                    dθ += 2.0 * PI;
                }
                if cw {
                    dθ = -dθ;
                }

                Some(Box::new(ArcSeg {
                    center: ctr,
                    radius: s_rel.norm(),
                    start_ang: θ0,
                    delta_ang: dθ,
                    mass,
                }) as Box<dyn Segment>)
            }
            _ => None,
        };

        self.m.pos = tgt; // update position *after* segment emission
        seg
    }
}

pub fn parse_segments(
    gcode_src: &str,
    filament_diameter: f32,
    filament_density: f32,
) -> Vec<Box<dyn Segment>> {
    let mut interp = Interpreter::new(filament_diameter, filament_density);
    let mut out = Vec::<Box<dyn Segment>>::new();

    for cmd in parse(gcode_src) {
        if let Some(seg) = interp.process_cmd(cmd) {
            out.push(seg);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    const EPS: f32 = 1e-3;

    #[test]
    fn test_parse_single_line() {
        let src = "G90\nG1 X10.0 Y20.0 E5.0";
        let segs = parse_segments(src, 1.75, 1.25);
        assert_eq!(segs.len(), 1);
        let seg = &segs[0];
        // center should be midpoint of (0,0) and (10,20)
        let c = seg.center();
        assert!((c.x - 5.0).abs() < EPS);
        assert!((c.y - 10.0).abs() < EPS);
        assert!((c.z).abs() < EPS);
        // mass matches extrusion->mass conversion
        let expected_mass = 5.0 * std::f32::consts::PI * 1.75_f32.powi(2) / 4.0 * (1.25 / 1000.0);
        assert!((seg.mass() - expected_mass).abs() < EPS);
    }

    #[test]
    fn test_relative_extrusion_mode() {
        let src = "M83\nG1 X1 Y0 E2.0\nG1 X2 Y0 E3.0";
        let segs = parse_segments(src, 1.75, 1.25);
        // Two extrusion segments: first E=2.0, then relative E=3.0
        assert_eq!(segs.len(), 2);
        let first = &segs[0];
        let second = &segs[1];
        assert!(
            (first.mass() - 2.0 * std::f32::consts::PI * 1.75_f32.powi(2) / 4.0 * (1.25 / 1000.0))
                .abs()
                < EPS
        );
        assert!(
            (second.mass() - 3.0 * std::f32::consts::PI * 1.75_f32.powi(2) / 4.0 * (1.25 / 1000.0))
                .abs()
                < EPS
        );
    }

    #[test]
    fn test_arc_segment_center() {
        // Move to (10,0), then CCW quarter circle to (0,10) with center at origin
        let src = "G90\nG1 X10 Y0 E1.0\nG3 X0 Y10 I-10 J0 E2.0";
        let segs = parse_segments(src, 1.75, 1.25);
        assert_eq!(segs.len(), 2);
        let arc = &segs[1];
        let c = arc.center();
        // computed center of mass for quarter circle of radius 10
        let r = 10.0;
        let dtheta = std::f32::consts::FRAC_PI_2;
        let ic = (std::f32::consts::PI / 2.0).sin() - 0.0;
        let expected_offset = r * ic / dtheta;
        assert!((c.x - expected_offset).abs() < EPS);
        assert!((c.y - expected_offset).abs() < EPS);
    }

    #[test]
    fn test_no_extrusion_no_segment() {
        // Move without extrusion should produce no segments
        let src = "G1 X5 Y5";
        let segs = parse_segments(src, 1.75, 1.25);
        assert!(segs.is_empty());
    }
}
