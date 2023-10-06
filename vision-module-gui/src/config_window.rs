use std::time::Duration;

use iui::{UI, prelude::{Window, WindowType, TextEntry}};
use leptos_reactive::{create_rw_signal, create_effect, SignalWith, SignalWithUntracked, SignalSet, SignalGet, SignalUpdate, create_signal, with, update};
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

    // let read_button_enabled = create_rw_signal(true);
    // crate::layout! { &ui,
    //     let grid = LayoutGrid(padded: true) {
    //         (1, 0)(1, 1) Horizontal (Fill, Center) : let device_combobox = Combobox() {}
    //         (2, 0)(1, 1) Neither (End, Center) : let refresh_button = Button("Refresh")
    //         (0, 1)(1, 1) Neither (End, Center) : let bank_label = Label("Bank")
    //         (0, 2)(1, 1) Neither (End, Center) : let address_label = Label("Address")
    //         (0, 3)(1, 1) Neither (End, Center) : let data_label = Label("Data")
    //         (1, 1)(1, 1) Horizontal (Fill, Fill) : let bank_input = Entry()
    //         (1, 2)(1, 1) Horizontal (Fill, Fill) : let address_input = Entry()
    //         (1, 3)(1, 1) Horizontal (Fill, Fill) : let data_input = Entry()
    //         (0, 4)(2, 1) Horizontal (Fill, Fill) : let buttons_hbox = HorizontalBox(padded: true) {
    //             Stretchy: let read_button = Button("Read", enabled: read_button_enabled)
    //             Stretchy: let write_button = Button("Write")
    //         }
    //     }
    // }
    let (device_rs, device_ws) = create_signal(None);
    let wf_pid = create_rw_signal(String::new());
    let nf_pid = create_rw_signal(String::new());
    crate::layout! { &ui,
        let vbox = VerticalBox(padded: true) {
            Compact : let device_hbox = HorizontalBox(padded: true) {
                Stretchy : let device_combobox = Combobox() {}
                Compact : let refresh_button = Button("Refresh")
            }
            Compact : let tab_group = TabGroup() {
                ("Wide field", margined: true) : let form = Form(padded: true) {
                    (Compact, "Product ID") : let wide_field_product_id_entry = Entry(value: wf_pid, enabled: false)
                    (Compact, "DSP area max threshold") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                    (Compact, "DSP noise threshold") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                    (Compact, "DSP orientation ratio") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                    (Compact, "DSP orientation factor") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                    (Compact, "DSP maximum number of objects") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                    (Compact, "Sensor gain 1") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                    (Compact, "Sensor gain 2") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                    (Compact, "Sensor exposure length") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                    (Compact, "Scale resolution X") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                    (Compact, "Scale resolution Y") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                    (Compact, "Frame period") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                    (Compact, "Frame rate") : let x = Entry(enabled: move || with!(|device_rs| device_rs.is_some()))
                }
                ("Near field", margined: true) : let form = Form(padded: true) {
                    (Compact, "Product ID") : let near_field_product_id_entry = Entry(value: nf_pid, enabled: false)
                    (Compact, "DSP area max threshold") : let x = Entry()
                    (Compact, "DSP noise threshold") : let x = Entry()
                    (Compact, "DSP orientation ratio") : let x = Entry()
                    (Compact, "DSP orientation factor") : let x = Entry()
                    (Compact, "DSP maximum number of objects") : let x = Entry()
                    (Compact, "Sensor gain 1") : let x = Entry()
                    (Compact, "Sensor gain 2") : let x = Entry()
                    (Compact, "Sensor exposure length") : let x = Entry()
                    (Compact, "Scale resolution X") : let x = Entry()
                    (Compact, "Scale resolution Y") : let x = Entry()
                    (Compact, "Frame period") : let x = Entry()
                    (Compact, "Frame rate") : let x = Entry()
                }
            }
            Compact : let buttons_hbox = HorizontalBox(padded: true) {
                Compact : let save_button = Button("Apply", enabled: move || with!(|device_rs| device_rs.is_some()))
                Compact : let save_button = Button("Save", enabled: move || with!(|device_rs| device_rs.is_some()))
                Compact : let save_button = Button("Load defaults", enabled: move || with!(|device_rs| device_rs.is_some()))
            }
        }
    }
    config_win.set_child(&ui, vbox);
    let clear_inputs = move || {
        update!(|wf_pid, nf_pid| {
            wf_pid.clear();
            nf_pid.clear();
        });
    };

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
    let device_combobox_on_selected = move |i| {
        device_ws.set(None);
        clear_inputs();
        let Ok(i) = usize::try_from(i) else { return };
        let path = device_list.with_untracked(|d| d[i].port_name.clone());
        let task = async move {
            let device = UsbDevice::connect(path)?;
            let nfpid = device.product_id(Port::Nf).await?;
            let wfpid = device.product_id(Port::Wf).await?;
            nf_pid.set(format!("0x{:04x}", nfpid));
            wf_pid.set(format!("0x{:04x}", wfpid));
            device_ws.set(Some(device.c()));
            Result::<()>::Ok(())
        };
        ui_ctx.spawn(async move {
            if let Err(e) = task.await {
                eprintln!("{e}");
            }
        });
    };
    device_combobox.on_selected(&ui, device_combobox_on_selected);

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

    // create_effect({
    //     let read_button = read_button.c();
    //     let ui = ui.c();
    //     move |_| {
    //         let mut read_button = read_button.c();
    //         read_button.set_text(&ui, &read_button_text.get());
    //     }
    // });
    // create_effect({
    //     let read_button = read_button.c();
    //     let ui = ui.c();
    //     move |_| {
    //         let mut read_button = read_button.c();
    //         match read_button_enabled.get() {
    //             true => read_button.enable(&ui),
    //             false => read_button.disable(&ui),
    //         }
    //     }
    // });
    // Async with tokio timers example
    // read_button.on_clicked(&ui, {
    //     let ui = ui.c();
    //     move |_| {
    //         read_button_enabled.set(false);
    //         read_button_text.set("Reading.".into());
    //         ui.spawn(async move {
    //             // the timers happen to work using ui.spawn but it might be necessary to do
    //             // tokio::spawn from within ui.spawn
    //             tokio::time::sleep(Duration::from_millis(500)).await;
    //             read_button_text.set("Reading..".into());
    //             tokio::time::sleep(Duration::from_millis(500)).await;
    //             read_button_text.set("Reading...".into());
    //             tokio::time::sleep(Duration::from_millis(500)).await;
    //             read_button_enabled.set(true);
    //             read_button_text.set("Read".into());
    //         });
    //     }
    // });


    (config_win, device_rs.clone())
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
