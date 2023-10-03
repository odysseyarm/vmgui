use std::thread;
use std::time::Duration;

use iui::menus::Menu;
use iui::prelude::*;
use leptos_reactive::{create_effect, create_rw_signal, SignalGet, SignalSet, SignalWith};
use serialport::SerialPortInfo;

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

trait CloneButShorter: Clone {
    /// Use mainly for GUI code.
    fn c(&self) -> Self {
        self.clone()
    }
}

impl<T: Clone> CloneButShorter for T {}

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

    let mut config_win = Window::new(&ui, "Config", 10, 10, WindowType::NoMenubar);
    config_win.on_closing(&ui, {
        let ui = ui.c();
        move |config_win| {
            config_win.hide(&ui);
        }
    });

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

    main_win.set_child(&ui, vert_box);

    iui::layout! { &ui,
        let grid = LayoutGrid(padded: true) {
            (1, 0)(1, 1) Horizontal (Fill, Center) : let device_combobox = Combobox() {}
            (2, 0)(1, 1) Neither (End, Center) : let refresh_button = Button("Refresh")
            (0, 1)(1, 1) Neither (End, Center) : let bank_label = Label("Bank")
            (0, 2)(1, 1) Neither (End, Center) : let address_label = Label("Address")
            (0, 3)(1, 1) Neither (End, Center) : let data_label = Label("Data")
            (1, 1)(1, 1) Horizontal (Fill, Fill) : let bank_input = Entry()
            (1, 2)(1, 1) Horizontal (Fill, Fill) : let address_input = Entry()
            (1, 3)(1, 1) Horizontal (Fill, Fill) : let data_input = Entry()
            (0, 4)(2, 1) Horizontal (Fill, Fill) : let buttons_hbox = HorizontalBox(padded: true) {
                Stretchy: let read_button = Button("Read")
                Stretchy: let write_button = Button("Write")
            }
        }
    }

    let device_list = create_rw_signal(Vec::<SerialPortInfo>::new());
    create_effect({
        let device_combobox = device_combobox.c();
        let ui = ui.c();
        move |_| {
            let mut device_combobox = device_combobox.c();
            device_combobox.clear(&ui);
            device_list.with(|device_list| {
                for device in device_list {
                    device_combobox.append(&ui, &display_for_serial_port(&device));
                }
            });
            device_combobox.enable(&ui);
        }
    });

    let refresh_device_list = {
        let config_win = config_win.c();
        let ui = ui.c();
        move || {
            let ports = serialport::available_ports();
            let ports = match ports {
                Ok(p) => p,
                Err(e) => {
                    config_win.modal_err(&ui, "Failed to list serial ports", &e.to_string());
                    return;
                }
            };
            device_list.set(ports);
        }
    };
    refresh_device_list();
    refresh_button.on_clicked(&ui, move |_| refresh_device_list());

    let read_button_text = create_rw_signal("Read");
    let read_button_enabled = create_rw_signal(true);
    create_effect({
        let read_button = read_button.c();
        let ui = ui.c();
        move |_| {
            let mut read_button = read_button.c();
            read_button.set_text(&ui, read_button_text.get());
        }
    });
    create_effect({
        let read_button = read_button.c();
        let ui = ui.c();
        move |_| {
            let mut read_button = read_button.c();
            match read_button_enabled.get() {
                true => read_button.enable(&ui),
                false => read_button.disable(&ui),
            }
        }
    });
    // Async with tokio timers example
    read_button.on_clicked(&ui, {
        let ui = ui.c();
        move |_| {
            read_button_enabled.set(false);
            read_button_text.set("Reading.");
            ui.spawn(async move {
                // the timers happen to work using ui.spawn but it might be necessary to do
                // tokio::spawn from within ui.spawn
                tokio::time::sleep(Duration::from_millis(500)).await;
                read_button_text.set("Reading..");
                tokio::time::sleep(Duration::from_millis(500)).await;
                read_button_text.set("Reading...");
                tokio::time::sleep(Duration::from_millis(500)).await;
                read_button_enabled.set(true);
                read_button_text.set("Read");
            });
        }
    });

    config_win.set_child(&ui, grid);
    config_button.on_clicked(&ui, {
        let ui = ui.c();
        move |_| {
            config_win.show(&ui);
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

fn display_for_serial_port(port_info: &SerialPortInfo) -> String {
    let usb_port = match &port_info.port_type {
        serialport::SerialPortType::UsbPort(u) => u,
        _ => return port_info.port_name.clone(),
    };

    let mut out = String::new();
    if let Some(m) = &usb_port.manufacturer {
        out.push_str(m);
    }
    if let Some(p) = &usb_port.product {
        if !out.is_empty() {
            out.push_str(" - ");
        }
        out.push_str(p);
    }
    if !out.is_empty() {
        out.push_str(" ");
    }
    out.push_str(&format!("({:4x}:{:4x})", usb_port.vid, usb_port.pid));
    out
}
