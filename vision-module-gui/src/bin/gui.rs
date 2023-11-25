use std::fs::File;
use std::sync::Arc;

use anyhow::Result;
use iui::prelude::*;
use leptos_reactive::{create_effect, SignalGet, SignalWith};
use serde::Serialize;
use vision_module_gui::{config_window::config_window, CloneButShorter, MotState};
use tokio::sync::Mutex;
use tokio::task::{AbortHandle};
use iui::controls::{Area, Button, HorizontalBox};
use vision_module_gui::mot_runner::MotRunner;
use vision_module_gui::run_canvas::RunCanvas;
use vision_module_gui::test_canvas::{TestCanvas};

// Things to avoid doing
// * Accessing signals outside of the main thread
//     * Getting panics, setting fails silently
//     * by extension, don't access signals from tokio::spawn, use ui.spawn
// * Excessive blocking in the main thread or in callbacks
//     * Freezes the GUI
// * Creating libui uiControls dynamically
//     * They are leaky

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    let tokio_rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap();
    let tokio_handle = tokio_rt.handle();
    let _enter = tokio_handle.enter();
    let leptos_rt = leptos_reactive::create_runtime();
    // Initialize the UI library
    let ui = UI::init().expect("Couldn't initialize UI library");
    let ui_ctx = ui.async_context();

    // Create a main_window into which controls can be placed
    let mut main_win = Window::new(&ui, "ATS Vision Tool", 640, 480, WindowType::NoMenubar);
    let (mut config_win, device_rs) = config_window(&ui, tokio_handle);

    iui::layout! { &ui,
        let form_vbox = VerticalBox(padded: true) {
            Compact: let form = Form(padded: true) {
                (Compact, "Datapoints added:"): let collected_text = Label("")
                (Compact, "Dataset"): let dataset_controls_group = HorizontalBox(padded: true) {
                    Compact: let add_datapoint_btn = Button("Add datapoint")
                    Compact: let remove_datapoint_btn = Button("Remove datapoint")
                    Compact: let clear_datapoints_btn = Button("Clear datapoints")
                    Compact: let save_datapoints_btn = Button("Save to file")
                }
            }
            Compact: let separator = HorizontalSeparator()
        }
    }

    form_vbox.hide(&ui);

    #[derive(Serialize)]
    struct Frame {
        nf_aim_point_x: Option<f32>,
        nf_aim_point_y: Option<f32>,
        wf_aim_point_x: Option<f32>,
        wf_aim_point_y: Option<f32>,
        nf_p1_x: Option<u16>,
        nf_p1_y: Option<u16>,
        nf_p2_x: Option<u16>,
        nf_p2_y: Option<u16>,
        nf_p3_x: Option<u16>,
        nf_p3_y: Option<u16>,
        nf_p4_x: Option<u16>,
        nf_p4_y: Option<u16>,
        wf_p1_x: Option<u16>,
        wf_p1_y: Option<u16>,
        wf_p2_x: Option<u16>,
        wf_p2_y: Option<u16>,
        wf_p3_x: Option<u16>,
        wf_p3_y: Option<u16>,
        wf_p4_x: Option<u16>,
        wf_p4_y: Option<u16>,
    }

    let mot_runner_abort_handle = Arc::new(Mutex::new(None::<AbortHandle>));

    let mut test_win = Window::new(&ui, "Aimpoint Test", 10, 10, WindowType::NoMenubar);
    test_win.set_margined(&ui, false);
    test_win.set_borderless(&ui, true);

    let state: Arc<Mutex<MotState>> = Default::default();
    let run_area = Area::new(&ui, Box::new(RunCanvas { ctx: ui.c(), state: state.c() }));
    let mut run_hbox = HorizontalBox::new(&ui);

    let stop_tracking = {
        let ui = ui.c();
        let run_hbox = run_hbox.c();
        let test_win = test_win.c();
        let mot_runner_abort_handle = mot_runner_abort_handle.c();
        move |button:&mut Button| {
            let mut run_hbox = run_hbox.c();
            run_hbox.hide(&ui);
            button.set_text(&ui, "Start Tracking");
            if !test_win.visible(&ui) {
                let mot_runner_abort_handle = &mot_runner_abort_handle.blocking_lock().take();
                if mot_runner_abort_handle.is_some() {
                    mot_runner_abort_handle.as_ref().expect("MOT Runner abort handle is None").abort();
                }
            }
        }
    };

    let test_win_on_closing = {
        let ui = ui.c();
        let form_vbox = form_vbox.c();
        let run_hbox = run_hbox.c();
        let mot_runner_abort_handle = mot_runner_abort_handle.c();
        move |win:&mut Window| {
            let mut form_vbox = form_vbox.c();
            win.hide(&ui);
            form_vbox.hide(&ui);
            if !run_hbox.visible(&ui) {
                let mot_runner_abort_handle = &mot_runner_abort_handle.blocking_lock().take();
                if mot_runner_abort_handle.is_some() {
                    mot_runner_abort_handle.as_ref().expect("MOT Runner abort handle is None").abort();
                }
            }
        }
    };

    test_win.on_closing(&ui, test_win_on_closing.c());

    let test_area = Area::new(&ui, Box::new(TestCanvas { ctx: ui.c(), window: test_win.c(), on_closing: Box::new(test_win_on_closing.c()), state: state.c() }));

    iui::layout! { &ui,
        let vert_box = VerticalBox(padded: true) {
            Compact: let grid = LayoutGrid(padded: false) {
                (0, 0)(1, 1) Vertical (Fill, Fill) : let config_button = Button("Config")
                (1, 0)(1, 1) Vertical (Fill, Fill) : let track_button = Button("Start Tracking")
                (2, 0)(1, 1) Vertical (Fill, Fill) : let test_button = Button("Run Test")
            }
            Compact: let separator = HorizontalSeparator()
            Compact: let spacer = Spacer()
        }
    }

    create_effect({
        let ui = ui.c();
        let test_win = test_win.c();
        let track_button = track_button.c();
        let test_button = test_button.c();
        let test_win_on_closing = test_win_on_closing.c();
        move |_| {
            let mut test_win = test_win.c();
            let mut track_button = track_button.c();
            let mut test_button = test_button.c();
            device_rs.with(|device| {
                if device.is_none() {
                    test_win_on_closing.c()(&mut test_win);
                    track_button.disable(&ui);
                    test_button.disable(&ui);
                } else {
                    track_button.enable(&ui);
                    test_button.enable(&ui);
                }
            });
        }
    });

    let datapoints: Arc<Mutex<Vec<Frame>>> = Arc::new(Mutex::new(Vec::new()));

    add_datapoint_btn.on_clicked(&ui, {
        let ui = ui.c();
        let datapoints = datapoints.c();
        let collected_text = collected_text.c();
        let state = state.c();
        move |_| {
            let datapoints = datapoints.c();
            let mut datapoints = datapoints.blocking_lock();
            let mut collected_text = collected_text.c();

            let mut frame = Frame {
                              nf_aim_point_x: None,
                              nf_aim_point_y: None,
                              wf_aim_point_x: None,
                              wf_aim_point_y: None,
                              nf_p1_x: None,
                              nf_p1_y: None,
                              nf_p2_x: None,
                              nf_p2_y: None,
                              nf_p3_x: None,
                              nf_p3_y: None,
                              nf_p4_x: None,
                              nf_p4_y: None,
                              wf_p1_x: None,
                              wf_p1_y: None,
                              wf_p2_x: None,
                              wf_p2_y: None,
                              wf_p3_x: None,
                              wf_p3_y: None,
                              wf_p4_x: None,
                              wf_p4_y: None,
                              };

            let state = state.blocking_lock();

            if let Some(nf_aim_point) = state.nf_aim_point {
                frame.nf_aim_point_x = Some(nf_aim_point.x as f32);
                frame.nf_aim_point_y = Some(nf_aim_point.y as f32);
            }

            if let Some(wf_aim_point) = state.wf_aim_point {
                frame.wf_aim_point_x = Some(wf_aim_point.x as f32);
                frame.wf_aim_point_y = Some(wf_aim_point.y as f32);
            }

            if let Some(nf_data) = &state.nf_data {
                if nf_data.len() > 0 {
                    frame.nf_p1_x = Some(nf_data[0].cx);
                    frame.nf_p1_y = Some(nf_data[0].cy);
                }
                if nf_data.len() > 1 {
                    frame.nf_p2_x = Some(nf_data[1].cx);
                    frame.nf_p2_y = Some(nf_data[1].cy);
                }
                if nf_data.len() > 2 {
                    frame.nf_p3_x = Some(nf_data[2].cx);
                    frame.nf_p3_y = Some(nf_data[2].cy);
                }
                if nf_data.len() > 3 {
                    frame.nf_p4_x = Some(nf_data[3].cx);
                    frame.nf_p4_y = Some(nf_data[3].cy);
                }
            }

            if let Some(wf_data) = &state.wf_data {
                if wf_data.len() > 0 {
                    frame.wf_p1_x = Some(wf_data[0].cx);
                    frame.wf_p1_y = Some(wf_data[0].cy);
                }
                if wf_data.len() > 1 {
                    frame.wf_p2_x = Some(wf_data[1].cx);
                    frame.wf_p2_y = Some(wf_data[1].cy);
                }
                if wf_data.len() > 2 {
                    frame.wf_p3_x = Some(wf_data[2].cx);
                    frame.wf_p3_y = Some(wf_data[2].cy);
                }
                if wf_data.len() > 3 {
                    frame.wf_p4_x = Some(wf_data[3].cx);
                    frame.wf_p4_y = Some(wf_data[3].cy);
                }
            }

            datapoints.push(frame);
            collected_text.set_text(&ui, datapoints.len().to_string().as_str());
        }
    });

    remove_datapoint_btn.on_clicked(&ui, {
        let ui = ui.c();
        let datapoints = datapoints.c();
        let collected_text = collected_text.c();
        move |_| {
            let datapoints = datapoints.c();
            let mut datapoints = datapoints.blocking_lock();
            let mut collected_text = collected_text.c();
            datapoints.pop();
            collected_text.set_text(&ui, datapoints.len().to_string().as_str());
        }
    });

    clear_datapoints_btn.on_clicked(&ui, {
        let ui = ui.c();
        let datapoints = datapoints.c();
        move |_| {
            let datapoints = datapoints.c();
            let mut datapoints = datapoints.blocking_lock();
            datapoints.clear();
            collected_text.set_text(&ui, datapoints.len().to_string().as_str());
        }
    });

    save_datapoints_btn.on_clicked(&ui, {
        let ui = ui.c();
        let main_win = main_win.c();
        move |_| {
            let datapoints = datapoints.c();
            let datapoints = datapoints.blocking_lock();
            if let Some(path_buf) = main_win.save_file(&ui) {
                let file = File::create(path_buf).expect("Could not create file");
                let mut writer = csv::Writer::from_writer(file);

                for datapoint in datapoints.as_slice() {
                    writer.serialize(datapoint).expect("Could not serialize");
                }
                writer.flush().expect("Could not flush");
            }
        }
    });

    main_win.set_child(&ui, vert_box.clone());

    config_button.on_clicked(&ui, {
        let ui = ui.c();
        move |_| {
            config_win.show(&ui);
        }
    });

    vert_box.append(&ui, form_vbox.c(), LayoutStrategy::Compact);

    let mot_runner = Arc::new(Mutex::new(MotRunner { state: state.c(), device: None }));

    create_effect({
        let view = mot_runner.c();
        move |_| {
            let view = view.c();
            view.blocking_lock().device = device_rs.get();
        }
    });

    run_hbox.append(&ui, run_area.c(), LayoutStrategy::Stretchy);

    vert_box.append(&ui, run_hbox.c(), LayoutStrategy::Stretchy);

    run_hbox.hide(&ui);

    track_button.on_clicked(&ui, {
        let ui = ui.c();
        let mut run_hbox = run_hbox.c();
        let mot_runner = mot_runner.c();
        let mot_runner_abort_handle = mot_runner_abort_handle.c();
        move |button| {
            if run_hbox.visible(&ui) {
                stop_tracking(button);
            } else {
                run_hbox.show(&ui);
                if mot_runner_abort_handle.blocking_lock().is_none() {
                    let mot_runner = mot_runner.c();
                    *(mot_runner_abort_handle.blocking_lock()) = Some(tokio::spawn(async move { mot_runner.lock().await.run().await; }).abort_handle());
                }
                button.set_text(&ui, "Stop Tracking");
            }
        }
    });

    let mut test_hbox = HorizontalBox::new(&ui);
    test_hbox.append(&ui, test_area.c(), LayoutStrategy::Stretchy);
    test_win.set_child(&ui, test_hbox);

    test_button.on_clicked(&ui, {
        let ui = ui.c();
        let mut form_vbox = form_vbox.c();
        let mut test_win = test_win.c();
        let mot_runner = mot_runner.c();
        let mot_runner_abort_handle = mot_runner_abort_handle.c();
        move |_| {
            form_vbox.show(&ui);
            test_win.show(&ui);
            if mot_runner_abort_handle.blocking_lock().is_none() {
                let mot_runner = mot_runner.c();
                *(mot_runner_abort_handle.blocking_lock()) = Some(tokio::spawn(async move { mot_runner.lock().await.run().await; }).abort_handle());
            }
        }
    });

    main_win.show(&ui);

    ui.ui_timer(5, {
        let ui = ui.c();
        let run_area = run_area.c();
        let run_hbox = test_area.c();
        let test_area = test_area.c();
        let test_win = test_win.c();
        move || {
            if run_hbox.visible(&ui) {
                run_area.queue_redraw_all(&ui);
            }
            if test_win.visible(&ui) {
                test_area.queue_redraw_all(&ui);
            }
            true
        }
    });

    // Run the application
    let mut i = 0;
    let mut ev = ui.event_loop();
    ev.on_tick(&ui, || {
        // println!("tick {i}");
        i += 1;
    });
    ev.run(&ui);

    leptos_rt.dispose();
    Ok(())
}
