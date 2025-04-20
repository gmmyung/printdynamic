use gloo_timers::future::TimeoutFuture;
use leptos::{html::Input, prelude::*, task::spawn_local};
use log::info;
use nalgebra::{Matrix3, Vector3};
use printdynamic::interpreter::parse_segments;
use web_sys::HtmlInputElement;

// src/main.rs (Leptos app)
#[component]
pub fn App() -> impl IntoView {
    let file_input: NodeRef<Input> = NodeRef::new();
    // Signals to hold inputs + outputs
    let (input, set_input) = signal(String::new());
    let (filament_width, set_filament_width) = signal("1.75".to_string());
    let (filament_density, set_filament_density) = signal("1.24".to_string());
    let (total_mass, set_total_mass) = signal(0.0f32);
    let (com, set_com) = signal(Vector3::<f32>::zeros());
    let (inertia, set_inertia) = signal(Matrix3::<f32>::zeros());
    let (inertia_cm, set_inertia_cm) = signal(Matrix3::<f32>::zeros());

    let parse_action = Action::new(move |input: &String| {
        let input = input.to_owned();
        async move {
            info!("Parsing...");
            send_wrapper::SendWrapper::new(TimeoutFuture::new(10)).await;
            let segs = parse_segments(
                &input,
                filament_width.get_untracked().parse().unwrap(),
                filament_density.get_untracked().parse().unwrap(),
            );

            let total = segs.iter().map(|s| s.as_ref().mass()).sum();
            set_total_mass.set(total);

            let com_val = segs
                .iter()
                .map(|s| s.center() * s.mass())
                .sum::<Vector3<f32>>()
                / total;
            set_com.set(com_val);

            let inertia_val = segs.iter().map(|s| s.inertia()).sum();
            set_inertia.set(inertia_val);

            let r2 = com_val.dot(&com_val); // ||COM||²
            let I_pa = Matrix3::identity() * r2 - com_val * com_val.transpose();
            let inertia_cm_val = inertia_val - total * I_pa;
            set_inertia_cm.set(inertia_cm_val);

            info!("Parsed → mass={}", total);
            info!("Parsed → com={:?}", com_val);
            info!("Parsed → inertia={:?}", inertia_val);
        }
    });

    let pending = parse_action.pending();

    async fn getFileContent(input: Option<HtmlInputElement>) -> String {
        let value = input.unwrap().files();
        let value_unwrapped = value.unwrap();
        let get_file = value_unwrapped.get(0);
        let file_text = get_file.unwrap().text();
        let result = wasm_bindgen_futures::JsFuture::from(file_text).await;
        result.unwrap().as_string().unwrap()
    }

    // Bind input, call parse on click, and render signals reactively
    view! {
        <div>
            <label for="file">"Select a gcode file"</label>
            <input
                type="file"
                id="file"
                node_ref=file_input
                on:change=move |_| {
                let file_input_value = file_input.get();
                spawn_local(async move {
                    let content = getFileContent(file_input_value).await;
                    set_input.set(content);
                })
            }
            />
            <br/>
            <label for="filament_width">"Filament Width (mm)"</label>
            <input
                type="number"
                id="filament_width"
                bind:value=(filament_width, set_filament_width)
            />
            <br/>
            <label for="filament_density">"Filament Density (g/cm^3)"</label>
            <input
                type="number"
                id="filament_density"
                bind:value=(filament_density, set_filament_density)
            />
            <br/>
            <button on:click=move |_| {parse_action.dispatch(input.get_untracked());}>"Parse"</button>
            {
                move || match pending.get() {
                    true => view! {<p>{"Loading..."}</p>}.into_any(),
                    false =>view! {
                        <p>{move || format!("Total Mass: {}g", total_mass.get())}</p>
                        <p>{move || format!("Center of Mass: {:?}mm", com.get())}</p>
                        <p>{move || format!("Inertia Tensor from Origin: {:?}g*mm^2", inertia.get())}</p>
                        <p>{move || format!("Inertia Tensor from Center of Mass: {:?}g*mm^2", inertia_cm.get())}</p>
                    }.into_any()
                }
            }
        </div>
    }
}
