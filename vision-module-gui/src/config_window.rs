use crate::{device::UsbDevice, packet::Port, CloneButShorter};
use anyhow::Result;
use iui::{
    controls::Form,
    prelude::{Window, WindowType},
    UI,
};
use leptos_reactive::{
    create_effect, create_rw_signal, ReadSignal, RwSignal, SignalGetUntracked, SignalSet,
    SignalWith, SignalWithUntracked,
};
use serialport::SerialPortInfo;

pub fn config_window(
    ui: &UI,
    _tokio_handle: &tokio::runtime::Handle,
) -> (Window, leptos_reactive::ReadSignal<Option<UsbDevice>>) {
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
                Compact : let apply_button = Button("Apply", enabled: connected)
                Compact : let save_button = Button("Save", enabled: connected)
                Compact : let reload_button = Button("Reload", enabled: connected)
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

    create_effect({
        // update device combobox when device_list changes
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
    apply_button.on_clicked(&ui, {
        let config_win = config_win.c();
        let ui = ui.c();
        let device = device.c();
        move |_| {
            let Some(device) = device.get_untracked() else {
                return;
            };
            let wf_res_x = wf_settings
                .resolution_x
                .with_untracked(|s| s.parse::<u16>());
            let wf_res_y = wf_settings
                .resolution_y
                .with_untracked(|s| s.parse::<u16>());
            let nf_res_x = nf_settings
                .resolution_x
                .with_untracked(|s| s.parse::<u16>());
            let nf_res_y = nf_settings
                .resolution_y
                .with_untracked(|s| s.parse::<u16>());
            let mut errors = vec![];
            if wf_res_x.is_err() {
                errors.push("wide field scale resolution X");
            }
            if wf_res_y.is_err() {
                errors.push("wide field scale resolution Y");
            }
            if nf_res_x.is_err() {
                errors.push("near field scale resolution X");
            }
            if nf_res_y.is_err() {
                errors.push("near field scale resolution Y");
            }
            if !errors.is_empty() {
                config_win.modal_err(&ui, "Error", &errors.join("\n"));
                return;
            }
            ui.spawn({
                let ui = ui.c();
                let config_win = config_win.c();
                async move {
                    println!(
                        "Writing resolution {wf_res_x:?} {wf_res_y:?} {nf_res_x:?} {nf_res_y:?}"
                    );
                    let r = tokio::try_join!(
                        device.set_resolution_x(Port::Wf, wf_res_x.unwrap()),
                        device.set_resolution_y(Port::Wf, wf_res_y.unwrap()),
                        device.set_resolution_x(Port::Nf, nf_res_x.unwrap()),
                        device.set_resolution_y(Port::Nf, nf_res_y.unwrap()),
                    );
                    println!("Finished writing");
                    if let Err(e) = r {
                        config_win.modal_err(&ui, "Error", &e.to_string());
                    }
                }
            });
        }
    });
    save_button.on_clicked(&ui, show_todo_modal.c());
    load_defaults_button.on_clicked(&ui, show_todo_modal.c());

    reload_button.on_clicked(&ui, {
        move |_| {
            if let Some(device) = device.get_untracked() {
                ui_ctx.spawn(async move {
                    let _ = tokio::join!(
                        nf_settings.load_from_device(&device),
                        wf_settings.load_from_device(&device),
                    );
                });
            }
        }
    });

    (config_win, device.read_only())
}

#[derive(Copy, Clone)]
struct SensorSettingsForm {
    port: Port,
    pid: RwSignal<String>,
    resolution_x: RwSignal<String>,
    resolution_y: RwSignal<String>,
    gain_1: RwSignal<String>,
    gain_2: RwSignal<String>,
    exposure_time: RwSignal<String>,
}

impl SensorSettingsForm {
    fn new(ui: &UI, device: ReadSignal<Option<UsbDevice>>, port: Port) -> (Form, Self) {
        let connected = move || device.with(|d| d.is_some());
        let pid = create_rw_signal(String::new());
        let resolution_x = create_rw_signal(String::new());
        let resolution_y = create_rw_signal(String::new());
        let gain_1 = create_rw_signal(String::new());
        let gain_2 = create_rw_signal(String::new());
        let exposure_time = create_rw_signal(String::new());
        let exposure_time_ms = move || match exposure_time.with(|s| s.parse::<u16>()) {
            Ok(n) => format!("{:.4}", f64::from(n) * 200.0 / 1e6),
            Err(_) => format!("???"),
        };
        crate::layout! { &ui,
            let form = Form(padded: true) {
                (Compact, "Product ID")               : let product_id = Entry(value: pid, enabled: false)
                (Compact, "DSP area max threshold")   : let x = Entry(enabled: connected)
                (Compact, "DSP noise threshold")      : let x = Entry(enabled: connected)
                (Compact, "DSP orientation ratio")    : let x = Entry(enabled: connected)
                (Compact, "DSP orientation factor")   : let x = Entry(enabled: connected)
                (Compact, "DSP maximum object count") : let x = Entry(enabled: connected)
                (Compact, "Gain")                     : let gain_combobox = Combobox(enabled: connected) {}
                (Compact, "Exposure time")            : let x = HorizontalBox(padded: true) {
                    Stretchy : let e = Entry(
                        enabled: connected,
                        onchange: exposure_time,
                        value: exposure_time,
                    )
                    Compact : let x1 = LayoutGrid() {
                        (0, 0)(1, 1) Vertical (Start, Center) : let s = Label(move || format!("× 200ns = {} ms", exposure_time_ms()))
                    }
                }
                (Compact, "Scale resolution X") : let x = Entry(enabled: connected, value: resolution_x, onchange: resolution_x)
                (Compact, "Scale resolution Y") : let x = Entry(enabled: connected, value: resolution_y, onchange: resolution_y)
                (Compact, "Frame period")       : let x = Entry(enabled: connected)
                (Compact, "Frame rate")         : let x = Entry(enabled: connected)
            }
        }
        for (label, _) in &GAIN_TABLE {
            gain_combobox.append(&ui, label);
        }
        match port {
            Port::Nf => gain_combobox.set_selected(&ui, 16),
            Port::Wf => gain_combobox.set_selected(&ui, 48),
        }
        (
            form,
            Self {
                port,
                pid,
                resolution_x,
                resolution_y,
                gain_1,
                gain_2,
                exposure_time,
            },
        )
    }

    async fn load_from_device(&self, device: &UsbDevice) -> Result<()> {
        self.pid.set("Connecting...".into());
        let (pid, res_x, res_y, gain_1, gain_2, expo) = tokio::try_join!(
            device.product_id(self.port),
            device.resolution_x(self.port),
            device.resolution_y(self.port),
            device.gain_1(self.port),
            device.gain_2(self.port),
            device.exposure_time(self.port),
        )?;
        self.pid.set(format!("0x{pid:04x}"));
        self.resolution_x.set(res_x.to_string());
        self.resolution_y.set(res_y.to_string());
        self.gain_1.set(gain_1.to_string());
        self.gain_2.set(gain_2.to_string());
        self.exposure_time.set(expo.to_string());
        Ok(())
    }

    fn validate(&self) {}

    fn clear(&self) {
        self.pid.set(String::new());
        self.resolution_x.set(String::new());
        self.resolution_y.set(String::new());
        self.gain_1.set(String::new());
        self.gain_2.set(String::new());
        self.exposure_time.set(String::new());
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

#[derive(Copy, Clone, Debug)]
pub struct Gain {
    b_global: u8,
    b_ggh: u8,
}

impl Gain {
    const fn new(b_global: u8, b_ggh: u8) -> Self {
        Self { b_global, b_ggh }
    }
}

// funny

const fn n_to_bstr(n: usize) -> [u8; 6] {
    [
        '0' as u8 + (n / 10000) as u8,
        '.' as u8 as u8,
        '0' as u8 + (n / 1000 % 10) as u8,
        '0' as u8 + (n / 100 % 10) as u8,
        '0' as u8 + (n / 10 % 10) as u8,
        '0' as u8 + (n % 10) as u8,
    ]
}

pub static GAIN_TABLE: [(&str, Gain); 16*3 + 1] = {
    static BACKING: [[u8; 6]; 16*3] = {
        let mut v = [[0; 6]; 16*3];
        let mut i = 0;
        while i < 16 {
            v[i] = n_to_bstr((10000 + i*10000/16) * 1);
            v[i+16] = n_to_bstr((10000 + i*10000/16) * 2);
            v[i+32] = n_to_bstr((10000 + i*10000/16) * 4);
            i += 1;
        }
        v
    };
    let mut omegalul = [("", Gain::new(0, 0)); 16*3 + 1];
    let mut i = 0;
    while i < 16*3 {
        omegalul[i].0 = unsafe { std::str::from_utf8_unchecked(&BACKING[i]) };
        omegalul[i].1 = Gain::new((i%16) as u8, [0,2,3][i/16]);
        i += 1;
    }
    omegalul[16*3] = ("8.0000", Gain::new(16, 3));
    omegalul
};

// static GAIN_TABLE: [(&'static str, Gain); 16 * 3] = [
// for i in range(16):
//     for j, k in zip([0, 2, 3], [1, 2, 4]):
//         value = (1 + i/16) * k
//         print(f'    ("{value:.4f}", Gain::new(0x{i:02x}, 0x{j:02x})),')
// ];
