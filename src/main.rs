use leptos::prelude::*;
use printdynamic_pages::app::App;

fn main() {
    console_error_panic_hook::set_once();
    leptos::mount::mount_to_body(|| view! { <App/> })
}
