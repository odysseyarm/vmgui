mod test_procedure_view;

use std::f64::consts::PI;
use std::thread;
use std::time::Duration;

use anyhow::Result;
use iui::menus::Menu;
use iui::prelude::*;
use leptos_reactive::{create_effect, create_rw_signal, SignalGet, SignalSet, SignalWith, with, SignalWithUntracked};
use serialport::{SerialPortInfo, SerialPort};
use vision_module_gui::{device::UsbDevice, packet::Port, config_window::config_window, CloneButShorter};
use crate::test_procedure_view::TestProcedureView;

// Things to avoid doing
// * Accessing signals outside of the main thread
//     * Getting panics, setting fails silently
//     * by extension, don't access signals from tokio::spawn, use ui.spawn
// * Excessive blocking in the main thread or in callbacks
//     * Freezes the GUI
// * Creating libui uiControls dynamically
//     * They are leaky

#[derive(Clone, Debug)]
struct Device {
    name: String,
}

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
    let mut main_win = Window::new(&ui, "ATS Vision Tool", 250, 100, WindowType::NoMenubar);
    let mut config_win = config_window(&ui, tokio_handle);

    iui::layout! { &ui,
        let form = Form(padded: true) {
            (Compact, "Points collected:"): let collected_text = Label("")
        }
    }

    form.hide(&ui);

    let test_win_on_closing = {
        let ui = ui.c();
        let mut form = form.c();
        move |win:&mut Window| {
            form.hide(&ui);
            win.hide(&ui);
        }
    };

    let mut test_win = Window::new(&ui, "Aimpoint Test", 10, 10, WindowType::NoMenubar);
    test_win.set_borderless(&ui, true);
    test_win.on_closing(&ui, test_win_on_closing.c());

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

    main_win.set_child(&ui, vert_box.clone());

    config_button.on_clicked(&ui, {
        let ui = ui.c();
        move |_| {
            config_win.show(&ui);
        }
    });

    vert_box.append(&ui, form.c(), LayoutStrategy::Compact);

    let view = TestProcedureView::new(ui.c(), test_win.c(), Box::new(test_win_on_closing));
    test_win.set_child(&ui, view.area);

    test_button.on_clicked(&ui, {
        let ui = ui.c();
        let mut form = form.c();
        move |_| {
            form.show(&ui);
            test_win.show(&ui);
        }
    });

    main_win.show(&ui);

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
