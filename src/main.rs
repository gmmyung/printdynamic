use leptos::prelude::*;
use log::Level;
use printdynamic_pages::app::App;

fn main() {
    console_error_panic_hook::set_once();
    console_log::init_with_level(Level::Debug).unwrap();
    leptos::mount::mount_to_body(|| view! { <App/> })
}
