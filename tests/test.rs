extern crate printdynamic;
use anyhow::Result;
use nalgebra::{Matrix3, Vector3};
use std::io::Read;

use printdynamic::interpreter::parse_segments;

#[test]
fn integration() -> Result<()> {
    // 1. load file
    let mut text = String::new();
    std::io::BufReader::new(std::fs::File::open("arc.gcode")?).read_to_string(&mut text)?;

    // 2. parse → segments
    let segments = parse_segments(&text, 1.75, 1.25);

    // 3. aggregate results
    let total_mass: f32 = segments.iter().map(|s| s.mass()).sum();
    let total_inertia: Matrix3<f32> = segments.iter().map(|s| s.inertia()).sum();
    let total_center: Vector3<f32> = segments
        .iter()
        .map(|s| s.center() * s.mass())
        .sum::<Vector3<f32>>()
        / total_mass;

    println!("Segments parsed : {}", segments.len());
    println!("Total mass      : {:.6}", total_mass);
    println!("Total inertia   :\n{}", total_inertia);
    println!(
        "Total center    : ({:.6}, {:.6}, {:.6})",
        total_center.x, total_center.y, total_center.z
    );

    Ok(())
}
