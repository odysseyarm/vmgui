use std::time::Duration;

use iui::{UI, prelude::{Window, WindowType, TextEntry}, controls::{Form, Button}};
use leptos_reactive::{create_rw_signal, create_effect, SignalWith, SignalWithUntracked, SignalSet, SignalGet, SignalUpdate, create_signal, with, update, ReadSignal, RwSignal};
use serialport::SerialPortInfo;
use crate::{CloneButShorter, device::UsbDevice, packet::Port};
use anyhow::Result;

pub fn config_window(ui: &UI, tokio_handle: &tokio::runtime::Handle) -> (Window, leptos_reactive::ReadSignal<Option<UsbDevice>>) {
    let ui_ctx = ui.async_context();
    let mut config_win = Window::new(&ui, "Config", 10, 10, WindowType::NoMenubar);
    config_win.on_closing(&ui, {
        let ui = ui.c();
        move |win: &mut Window| {
            win.hide(&ui);
        }
    });

    let device = create_rw_signal(None);
    let connected = move || device.with(|d| d.is_some());
    crate::layout! { &ui,
        let vbox = VerticalBox(padded: true) {
            Compact : let device_hbox = HorizontalBox(padded: true) {
                Stretchy : let device_combobox = Combobox() {}
                Compact : let refresh_button = Button("Refresh")
            }
            Compact : let tab_group = TabGroup() {} // sensor settings go in here
            Compact : let buttons_hbox = HorizontalBox(padded: true) {
                Compact : let apply_button = Button("Apply",                 enabled: connected)
                Compact : let save_button = Button("Save",                   enabled: connected)
                Compact : let reload_button = Button("Reload",               enabled: connected)
                Compact : let load_defaults_button = Button("Load defaults", enabled: connected)
            }
        }
    }
    let (wf_form, wf_settings) = SensorSettingsForm::new(&ui, device.read_only(), Port::Wf);
    let (nf_form, nf_settings) = SensorSettingsForm::new(&ui, device.read_only(), Port::Nf);
    tab_group.append(&ui, "Wide field", wf_form);
    tab_group.append(&ui, "Near field", nf_form);
    tab_group.set_margined(&ui, 0, true);
    tab_group.set_margined(&ui, 1, true);

    config_win.set_child(&ui, vbox);

    let device_list = create_rw_signal(Vec::<SerialPortInfo>::new());
    let device_combobox_on_selected = {
        let ui = ui.c();
        let config_win = config_win.c();
        move |i| {
            device.set(None);
            wf_settings.clear();
            nf_settings.clear();
            let Ok(i) = usize::try_from(i) else { return };
            let path = device_list.with_untracked(|d| d[i].port_name.clone());
            let task = async move {
                let usb_device = UsbDevice::connect(path)?;
                tokio::try_join!(
                    wf_settings.load_from_device(&usb_device),
                    nf_settings.load_from_device(&usb_device),
                )?;
                device.set(Some(usb_device.c()));
                Result::<()>::Ok(())
            };
            ui.spawn({
                let ui = ui.c();
                let config_win = config_win.c();
                async move {
                    if let Err(e) = task.await {
                        config_win.modal_err(&ui, "Failed to connect", &e.to_string());
                    }
                }
            });
        }
    };
    device_combobox.on_selected(&ui, device_combobox_on_selected.c());

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
            device_combobox_on_selected(-1);
        }
    };
    refresh_device_list();
    refresh_button.on_clicked(&ui, move |_| refresh_device_list());

    let show_todo_modal = {
        let config_win = config_win.c();
        let ui = ui.c();
        move |_: &mut _| {
            config_win.modal_msg(&ui, "Not implemented", "");
        }
    };
    apply_button.on_clicked(&ui, show_todo_modal.c());
    save_button.on_clicked(&ui, show_todo_modal.c());
    reload_button.on_clicked(&ui, show_todo_modal.c());
    load_defaults_button.on_clicked(&ui, show_todo_modal.c());

    (config_win, device.read_only())
}

#[derive(Copy, Clone)]
struct SensorSettingsForm {
    port: Port,
    pid: RwSignal<String>,
    resolution_x: RwSignal<String>,
    resolution_y: RwSignal<String>,
    sensor_gain_1: RwSignal<String>,
    sensor_gain_2: RwSignal<String>,
    sensor_exposure: RwSignal<String>,
}

impl SensorSettingsForm {
    fn new(ui: &UI, device: ReadSignal<Option<UsbDevice>>, port: Port) -> (Form, Self) {
        let connected = move || device.with(|d| d.is_some());
        let pid = create_rw_signal(String::new());
        let resolution_x = create_rw_signal(String::new());
        let resolution_y = create_rw_signal(String::new());
        let sensor_gain_1 = create_rw_signal(String::new());
        let sensor_gain_2 = create_rw_signal(String::new());
        let sensor_exposure = create_rw_signal(String::new());
        let sensor_exposure_ms = move || {
            match sensor_exposure.with(|s| u16::from_str_radix(s, 10)) {
                Ok(n) => format!("{:.4}", f64::from(n) * 200.0 / 1e6),
                Err(_) => format!("???"),
            }
        };
        crate::layout! { &ui,
            let form = Form(padded: true) {
                (Compact, "Product ID")                    : let product_id = Entry(value: pid, enabled: false)
                (Compact, "DSP area max threshold")        : let x = Entry(enabled: connected)
                (Compact, "DSP noise threshold")           : let x = Entry(enabled: connected)
                (Compact, "DSP orientation ratio")         : let x = Entry(enabled: connected)
                (Compact, "DSP orientation factor")        : let x = Entry(enabled: connected)
                (Compact, "DSP maximum number of objects") : let x = Entry(enabled: connected)
                (Compact, "Sensor gain 1")                 : let x = Entry(enabled: connected, value: sensor_gain_1)
                (Compact, "Sensor gain 2")                 : let x = Entry(enabled: connected, value: sensor_gain_2)
                (Compact, "Sensor exposure length")        : let x = HorizontalBox(padded: true) {
                    Stretchy : let e = Entry(
                        enabled: connected,
                        onchange: sensor_exposure,
                        value: sensor_exposure,
                    )
                    Compact : let x1 = LayoutGrid() {
                        (0, 0)(1, 1) Vertical (Start, Center) : let s = Label(move || format!("× 200ns = {} ms", sensor_exposure_ms()))
                    }
                }
                (Compact, "Scale resolution X")            : let x = Entry(enabled: connected, value: resolution_x)
                (Compact, "Scale resolution Y")            : let x = Entry(enabled: connected, value: resolution_y)
                (Compact, "Frame period")                  : let x = Entry(enabled: connected)
                (Compact, "Frame rate")                    : let x = Entry(enabled: connected)
            }
        }
        (
            form,
            Self {
                port,
                pid,
                resolution_x,
                resolution_y,
                sensor_gain_1,
                sensor_gain_2,
                sensor_exposure,
            }
        )
    }

    async fn load_from_device(&self, device: &UsbDevice) -> Result<()> {
        self.pid.set("Connecting...".into());
        let (pid, res_x, res_y, gain_1, gain_2, expo) = tokio::try_join!(
            device.product_id(self.port),
            device.resolution_x(self.port),
            device.resolution_y(self.port),
            device.sensor_gain_1(self.port),
            device.sensor_gain_2(self.port),
            device.sensor_exposure(self.port),
        )?;
        self.pid.set(format!("0x{pid:04x}"));
        self.resolution_x.set(res_x.to_string());
        self.resolution_y.set(res_y.to_string());
        self.sensor_gain_1.set(gain_1.to_string());
        self.sensor_gain_2.set(gain_2.to_string());
        self.sensor_exposure.set(expo.to_string());
        Ok(())
    }

    fn clear(&self) {
        self.pid.set(String::new());
        self.resolution_x.set(String::new());
        self.resolution_y.set(String::new());
        self.sensor_gain_1.set(String::new());
        self.sensor_gain_2.set(String::new());
        self.sensor_exposure.set(String::new());
    }
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
