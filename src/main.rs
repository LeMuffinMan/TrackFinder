mod app;
mod dem;
mod geo;
mod graph;
mod map;
mod tiles;
mod track;
mod trails;

use app::TrackFinderApp;

#[cfg(target_arch = "wasm32")]
fn main() {
    console_error_panic_hook::set_once();
    let _ = console_log::init_with_level(log::Level::Info);

    let web_options = eframe::WebOptions::default();

    wasm_bindgen_futures::spawn_local(async {
        use eframe::wasm_bindgen::JsCast as _;

        let canvas = web_sys::window()
            .expect("pas de window")
            .document()
            .expect("pas de document")
            .get_element_by_id("trackfinder_canvas")
            .expect("canvas trackfinder_canvas introuvable")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("l'element n'est pas un canvas");

        eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|_cc| Ok(Box::new(TrackFinderApp::new()))),
            )
            .await
            .expect("echec du demarrage eframe");
    });
}

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result<()> {
    env_logger::init();
    eframe::run_native(
        "TrackFinder",
        eframe::NativeOptions::default(),
        Box::new(|_cc| Ok(Box::new(TrackFinderApp::new()))),
    )
}
