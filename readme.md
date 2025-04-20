# G‑Code Mass & Inertia Analyzer

Parse any extrusion‑based G‑code file, convert the deposited filament to **mass, center‑of‑mass and full inertia tensors**, and explore the results in a WebAssembly (WASM) app built with **Leptos**.

 ![docs](https://img.shields.io/badge/docs.rs-online-blue)  [**Live demo**](https://<user>.github.io/<repo>/) 

**TL;DR**  
1. `cargo install trunk`  
2. `trunk serve` → open <http://127.0.0.1:8080>  
3. Drag‑and‑drop a `.gcode` file and inspect mass / COM / inertia— both about the origin **and** the true COM.

---

## Features
- **Robust Segment Model** — generic `Segment` trait with `LineSeg` & `ArcSeg` implementations.
  - TODO: `CubicSeg` & `BezierSeg`?
- **Extrusion‑aware Interpreter** — handles absolute/relative axes (`G90/G91`), extruder modes (`M82/M83`), and resets (`G92`).
- **Accurate Physics** — all outputs in _mm, g, g·mm²_; correct parallel‑axis shift to the center of mass.
- **Leptos WASM UI** — instant client‑side parsing, no server; shows mass, COM, `I₀` (origin) and `Iᴄᴏᴍ`.

Made with Rust and Leptos.
