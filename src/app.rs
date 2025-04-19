use leptos::{prelude::*, task::spawn_local};
use printdynamic::interpreter::parse_segments;
use printdynamic::segments::Segment;
use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

#[derive(Deserialize, Serialize, Debug, Clone)]
pub struct SegmentData {
    kind: String,
    center: [f32; 3],
    volume: f32,
    inertia: [[f32; 3]; 3],
}

fn to_data(seg: Box<dyn Segment>) -> SegmentData {
    let c = seg.center();
    let m = seg.volume();
    let I = seg.inertia();
    SegmentData {
        kind: if I[(0, 0)] == 0.0 && I[(1, 1)] > 0.0 {
            "Line".into()
        } else {
            "Arc".into()
        },
        center: [c.x, c.y, c.z],
        volume: m,
        inertia: [
            [I[(0, 0)], I[(0, 1)], I[(0, 2)]],
            [I[(1, 0)], I[(1, 1)], I[(1, 2)]],
            [I[(2, 0)], I[(2, 1)], I[(2, 2)]],
        ],
    }
}

/// Exposed parse function for JS/WASM
#[wasm_bindgen]
pub fn parse_gcode(src: &str) -> JsValue {
    let segments = parse_segments(src);
    let data: Vec<SegmentData> = segments.into_iter().map(to_data).collect();
    serde_wasm_bindgen::to_value(&data).unwrap()
}

// src/main.rs (Leptos app)
#[component]
fn App() -> impl IntoView {
    let (input, set_input) = create_signal(String::new());
    let (segments, set_segments) = create_signal(Vec::new());
    let (com, set_com) = create_signal([0.0f32; 3]);
    let (inertia_cm, set_inertia_cm) = create_signal([[0.0f32; 3]; 3]);

    let parse = spawn_local(async move {
        let src = input.get();
        let segs = parse_segments(&src);
        set_segments.set(segs);

        let total_mass: f32 = segs.iter().map(|s| s.mass).sum();
    });

    let parse_action = create_action(async move |_| {
        let src = input.get();
        // Call into the WASM module, receive JsValue
        let js_val = parse_gcode(&src);
        // Deserialize back into Vec<SegmentData>
        let segs: Vec<SegmentData> = serde_wasm_bindgen::from_value(js_val).unwrap(); //  [oai_citation_attribution:10‡Docs.rs](https://docs.rs/serde-wasm-bindgen/latest/serde_wasm_bindgen/fn.to_value.html?utm_source=chatgpt.com)

        // Compute total mass & center of mass
        let total_mass: f32 = segs.iter().map(|s| s.volume).sum();
        let mut weighted = [0.0f32; 3];
        for s in &segs {
            for i in 0..3 {
                weighted[i] += s.center[i] * s.volume;
            }
        }
        let com = [
            weighted[0] / total_mass,
            weighted[1] / total_mass,
            weighted[2] / total_mass,
        ];

        // Sum inertias about origin
        let mut I0 = [[0.0f32; 3]; 3];
        for s in &segs {
            for i in 0..3 {
                for j in 0..3 {
                    I0[i][j] += s.inertia[i][j];
                }
            }
        }

        // Parallel-axis transform: I_cm = I0 - M*(||r||² I - r⊗r)
        let r = com;
        let r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        let mut Icm = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let eye = if i == j { r2 } else { 0.0 };
                Icm[i][j] = I0[i][j] - total_mass * (eye - r[i] * r[j]);
            }
        }

        set_segments.set(segs);
        set_com.set(com);
        set_inertia_cm.set(Icm);
    });

    todo!()
}
