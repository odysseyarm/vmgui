use std::f64::consts::PI;
use std::thread;
use std::time::Duration;

use anyhow::Result;
use iui::menus::Menu;
use iui::prelude::*;
use leptos_reactive::{create_effect, create_rw_signal, SignalGet, SignalSet, SignalWith, with, SignalWithUntracked};
use serialport::{SerialPortInfo, SerialPort};
use vision_module_gui::{device::UsbDevice, packet::Port};
use iui::controls::{Area, AreaDrawParams, AreaHandler, AreaKeyEvent};
use iui::draw::{Brush, FillMode, Path, SolidBrush};

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

struct TestCanvas {
    ctx: UI,
    window: Window,
}
impl AreaHandler for TestCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
        let ctx = &draw_params.context;

        let path = Path::new(ctx, FillMode::Winding);
        path.add_rectangle(ctx, 0., 0., draw_params.area_width, draw_params.area_height);
        path.end(ctx);

        let brush = Brush::Solid(SolidBrush {
            r: 0.2,
            g: 0.6,
            b: 0.8,
            a: 1.,
        });

        draw_params.context.fill(&path, &brush);

        let path = Path::new(ctx, FillMode::Winding);
        for i in 0..100 {
            let x = i as f64 / 100.;
            let y = ((x * PI * 2.).sin() + 1.) / 2.;
            path.add_rectangle(
                ctx,
                x * draw_params.area_width,
                0.,
                draw_params.area_width / 100.,
                y * draw_params.area_height,
            );
        }
        path.end(ctx);

        let brush = Brush::Solid(SolidBrush {
            r: 0.2,
            g: 0.,
            b: 0.3,
            a: 1.,
        });

        draw_params.context.fill(&path, &brush);
    }

    fn key_event(&mut self, _area: &Area, _area_key_event: &AreaKeyEvent) -> bool {
        self.window.hide(&self.ctx);
        true
    }
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

    let mut config_win = Window::new(&ui, "Config", 10, 10, WindowType::NoMenubar);
    config_win.on_closing(&ui, {
        let ui = ui.c();
        move |config_win| {
            config_win.hide(&ui);
        }
    });

    let mut test_win = Window::new(&ui, "Aimpoint Test", 10, 10, WindowType::NoMenubar);
    test_win.set_borderless(&ui, true);
    test_win.on_closing(&ui, {
        let ui = ui.c();
        move |test_win| {
            test_win.hide(&ui);
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
    let read_button_text = create_rw_signal(String::from("Read"));

    let device_list = create_rw_signal(Vec::<SerialPortInfo>::new());
    create_effect({ // update device combobox when device_list changes
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
    device_combobox.on_selected(&ui, {
        let tokio_handle = tokio_handle.clone();
        move |i| {
            let Ok(i) = usize::try_from(i) else { return };
            let path = device_list.with_untracked(|d| d[i].port_name.clone());
            let task = async move {
                let device = UsbDevice::connect(path)?;
                let nv_pid = device.product_id(Port::Nv).await?;
                let fv_pid = device.product_id(Port::Fv).await?;
                println!("nv pid: {:04x}", nv_pid);
                println!("fv pid: {:04x}", fv_pid);
                ui_ctx.queue_main(move || read_button_text.set(format!("{:04x}", nv_pid)));
                Result::<()>::Ok(())
            };
            tokio_handle.spawn(async move {
                if let Err(e) = task.await {
                    eprintln!("{e}");
                }
            });
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

    let read_button_enabled = create_rw_signal(true);
    create_effect({
        let read_button = read_button.c();
        let ui = ui.c();
        move |_| {
            let mut read_button = read_button.c();
            read_button.set_text(&ui, &read_button_text.get());
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
            read_button_text.set("Reading.".into());
            ui.spawn(async move {
                // the timers happen to work using ui.spawn but it might be necessary to do
                // tokio::spawn from within ui.spawn
                tokio::time::sleep(Duration::from_millis(500)).await;
                read_button_text.set("Reading..".into());
                tokio::time::sleep(Duration::from_millis(500)).await;
                read_button_text.set("Reading...".into());
                tokio::time::sleep(Duration::from_millis(500)).await;
                read_button_enabled.set(true);
                read_button_text.set("Read".into());
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

    let area_handler: Box<dyn AreaHandler> = Box::new(TestCanvas { ctx: ui.clone(), window: test_win.clone() });
    let area = Area::new(&ui, area_handler);

    test_win.set_child(&ui, area);

    test_button.on_clicked(&ui, {
        let ui = ui.c();
        move |_| {
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
