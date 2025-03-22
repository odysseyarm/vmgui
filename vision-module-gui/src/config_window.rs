mod paj_sensor_settings;
mod poc_sensor_settings;

use std::{sync::Arc, time::Duration};

use crate::{mot_runner::MotRunner, CloneButShorter};
use anyhow::Result;
use ats_usb::{
    device::UsbDevice,
    packets::vm::{AccelConfig, GeneralConfig, GyroConfig, Port},
};
use iui::{
    controls::{Button, Form},
    prelude::{Window, WindowType},
    UI,
};
use leptos_reactive::{
    create_effect, create_rw_signal, ReadSignal, RwSignal, SignalGet, SignalGetUntracked,
    SignalSet, SignalWith, SignalWithUntracked,
};
use opencv_ros_camera::RosOpenCvIntrinsics;
use parking_lot::Mutex;
use serialport::SerialPortInfo;
use serialport::SerialPortType::UsbPort;

pub fn config_window(
    ui: &UI,
    simulator_addr: Option<String>,
    udp_addr: Option<String>,
    mot_runner: Arc<Mutex<MotRunner>>,
    _tokio_handle: &tokio::runtime::Handle,
) -> (
    Window,
    ReadSignal<Option<UsbDevice>>,
    ReadSignal<AccelConfig>,
) {
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

    let (general_form, general_settings) =
        GeneralSettingsForm::new(&ui, device.read_only(), mot_runner, config_win.c());

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
    let (wf_form, wf_settings) = paj_sensor_settings::PajSensorSettingsForm::new(&ui, device.read_only(), Port::Wf);
    let (nf_form, nf_settings) = paj_sensor_settings::PajSensorSettingsForm::new(&ui, device.read_only(), Port::Nf);
    let (poc_form, poc_settings) = poc_sensor_settings::PocSensorSettingsForm::new(&ui, device.read_only());
    tab_group.append(&ui, "General", general_form);
    tab_group.append(&ui, "Wide field", wf_form.c());
    tab_group.append(&ui, "Near field", nf_form.c());
    tab_group.append(&ui, "POC", poc_form.c());
    tab_group.set_margined(&ui, 0, true);
    tab_group.set_margined(&ui, 1, true);
    tab_group.set_margined(&ui, 2, true);
    tab_group.set_margined(&ui, 3, true);

    create_effect({
        let ui = ui.c();
        let nf_form = nf_form.c();
        let wf_form = wf_form.c();
        let poc_form = poc_form.c();
        move |_| {
            let mut nf_form = nf_form.c();
            let mut wf_form = wf_form.c();
            let mut poc_form = poc_form.c();
            general_settings.device_pid.with(|pid| {
                match num_traits::FromPrimitive::from_u16(*pid) {
                    Some(ats_usb::device::ProductId::PajUsb) | Some(ats_usb::device::ProductId::PajAts) => {
                        nf_form.show(&ui);
                        wf_form.show(&ui);
                        poc_form.hide(&ui);
                    }
                    Some(ats_usb::device::ProductId::PocAts) => {
                        nf_form.hide(&ui);
                        wf_form.hide(&ui);
                        poc_form.show(&ui);
                    }
                    _ => {
                        nf_form.hide(&ui);
                        wf_form.hide(&ui);
                        poc_form.hide(&ui);
                    }
                }
            });
        }
    });

    config_win.set_child(&ui, vbox);

    let device_list = create_rw_signal(Vec::<SerialPortInfo>::new());
    let device_combobox_on_selected = {
        let ui = ui.c();
        let config_win = config_win.c();
        let sim_addr = simulator_addr.c();
        let udp_addr = udp_addr.c();
        let general_settings = general_settings.c();
        move |i| {
            device.set(None);
            general_settings.clear();
            wf_settings.clear();
            nf_settings.clear();
            poc_settings.clear();
            let Ok(i) = usize::try_from(i) else { return };
            let _device = device_list.with_untracked(|d| d.get(i).cloned());
            let sim_addr = sim_addr.c();
            let udp_addr = udp_addr.c();
            let general_settings = general_settings.c();
            let task = async move {
                let usb_device = if let Some(_device) = _device {
                    match &_device.port_type {
                        UsbPort(port_info) => Ok(UsbDevice::connect_serial(
                            &_device.port_name,
                            port_info.pid == ats_usb::device::ProductId::PajAts as u16,
                        )
                        .await?),
                        _ => Err(anyhow::anyhow!("Not a USB device")),
                    }
                } else if let Some(sim_addr) = sim_addr.as_ref() {
                    Ok(UsbDevice::connect_tcp(sim_addr)?)
                } else {
                    Ok(UsbDevice::connect_hub("0.0.0.0:0", udp_addr.as_ref().unwrap()).await?)
                };
                match usb_device {
                    Ok(usb_device) => {
                        match num_traits::FromPrimitive::from_u16(general_settings.load_from_device(&usb_device, true).await?) {
                            Some(ats_usb::device::ProductId::PajAts) | Some(ats_usb::device::ProductId::PajUsb) => {
                                wf_settings.load_from_device(&usb_device).await?;
                                nf_settings.load_from_device(&usb_device).await?;
                            }
                            Some(ats_usb::device::ProductId::PocAts) => {
                                poc_settings.load_from_device(&usb_device).await?;
                            }
                            None => {}
                        }
                        device.set(Some(usb_device));
                        Result::<()>::Ok(())
                    }
                    Err(e) => Err(e),
                }
            };
            ui.spawn({
                let ui = ui.c();
                let config_win = config_win.c();
                async move {
                    if let Err(e) = task.await {
                        config_win
                            .modal_err_async(&ui, "Failed to connect", &e.to_string())
                            .await;
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
        let simulator_addr = simulator_addr.c();
        let udp_addr = udp_addr.c();
        move |_| {
            let mut device_combobox = device_combobox.c();
            device_combobox.clear(&ui);
            device_list.with(|device_list| {
                for device in device_list {
                    device_combobox.append(&ui, &display_for_serial_port(&device));
                }
            });
            if let Some(sim_addr) = &simulator_addr {
                device_combobox.append(&ui, &format!("Simulator @ {sim_addr}"));
            }
            if let Some(udp_addr) = &udp_addr {
                device_combobox.append(&ui, &format!("M4Hub @ {udp_addr}"));
            }
            device_combobox.enable(&ui);
        }
    });
    let mut refresh_device_list = {
        let config_win = config_win.c();
        let ui = ui.c();
        let simulator_addr = simulator_addr.c();
        let udp_addr = udp_addr.c();
        move || {
            let ports = serialport::available_ports();
            let ports: Vec<_> = match ports {
                Ok(p) => p,
                Err(e) => {
                    config_win.modal_err(&ui, "Failed to list serial ports", &e.to_string());
                    return;
                }
            }
            .into_iter()
            .filter(|port| {
                match &port.port_type {
                    UsbPort(port_info) => {
                        if port_info.vid == 0x1915 && (
                            port_info.pid == 0x520F
                            || port_info.pid == 0x5210
                            || port_info.pid == 0x5211
                        ) {
                            if let Some(i) = port_info.interface {
                                // interface 0: cdc acm module
                                // interface 1: cdc acm module functional subordinate interface
                                // interface 2: cdc acm dfu
                                // interface 3: cdc acm dfu subordinate interface
                                i == 0
                            } else {
                                true
                            }
                        } else {
                            false
                        }
                    }
                    _ => false,
                }
            })
            .collect();
            device_list.set(ports.c());
            if simulator_addr.is_some() {
                device_combobox.set_selected(&ui, ports.len() as i32);
                device_combobox_on_selected(ports.len() as i32);
            } else if udp_addr.is_some() {
                device_combobox.set_selected(&ui, ports.len() as i32);
                device_combobox_on_selected(ports.len() as i32);
            } else if ports.len() > 0 {
                device_combobox.set_selected(&ui, 0);
                device_combobox_on_selected(0);
            } else {
                device_combobox_on_selected(-1);
            }
        }
    };
    refresh_device_list();
    refresh_button.on_clicked(&ui, move |_| refresh_device_list());

    let apply_button_on_click = {
        let config_win = config_win.c();
        let ui = ui.c();
        let general_settings = general_settings.c();
        move |device: UsbDevice| async move {
            let mut errors = vec![];
            general_settings.validate(&mut errors);
            if !errors.is_empty() {
                let mut message = String::new();
                for msg in &errors {
                    message.push_str(&msg);
                    message.push('\n');
                }
                config_win
                    .modal_err_async(&ui, "General Validation Error", &message)
                    .await;
                return false;
            }

            wf_settings.validate(&mut errors);
            if !errors.is_empty() {
                let mut message = String::new();
                for msg in &errors {
                    message.push_str(&msg);
                    message.push('\n');
                }
                config_win
                    .modal_err_async(&ui, "Wide Field Validation Error", &message)
                    .await;
                return false;
            }

            nf_settings.validate(&mut errors);
            if !errors.is_empty() {
                let mut message = String::new();
                for msg in &errors {
                    message.push_str(&msg);
                    message.push('\n');
                }
                config_win
                    .modal_err_async(&ui, "Near Field Validation Error", &message)
                    .await;
                return false;
            }

            if let Err(e) = general_settings.apply(&device).await {
                config_win
                    .modal_err_async(&ui, "Failed to apply general settings", &e.to_string())
                    .await;
                return false;
            };
            if let Err(e) = wf_settings.apply(&device).await {
                config_win
                    .modal_err_async(&ui, "Failed to apply wide field settings", &e.to_string())
                    .await;
                return false;
            };
            if let Err(e) = nf_settings.apply(&device).await {
                config_win
                    .modal_err_async(&ui, "Failed to apply near field settings", &e.to_string())
                    .await;
                return false;
            };
            return true;
        }
    };
    apply_button.on_clicked(&ui, {
        let f = apply_button_on_click.clone();
        let device = device.c();
        let ui = ui.c();
        move |apply_button| {
            let Some(device) = device.get_untracked() else {
                return;
            };
            let f = f.clone();
            let mut apply_button = apply_button.c();
            let ui2 = ui.c();
            ui.spawn(async move {
                f(device).await;
                apply_button.set_text(&ui2, "Applied!");
                tokio::time::sleep(Duration::from_secs(3)).await;
                apply_button.set_text(&ui2, "Apply");
            });
        }
    });
    save_button.on_clicked(&ui, {
        let config_win = config_win.c();
        let ui = ui.c();
        let device = device.c();
        move |save_button| {
            let Some(device) = device.get_untracked() else {
                return;
            };
            ui.spawn({
                let ui = ui.c();
                let config_win = config_win.c();
                let apply_button_on_click = apply_button_on_click.c();
                let mut save_button = save_button.c();
                async move {
                    if !apply_button_on_click(device.clone()).await {
                        // callback should have already displayed an error modal, just return
                        return;
                    }
                    if let Err(e) = device.flash_settings().await {
                        config_win
                            .modal_err_async(
                                &ui,
                                "Failed to request flash settings",
                                &e.to_string(),
                            )
                            .await;
                    }
                    save_button.set_text(&ui, "Saved!");
                    tokio::time::sleep(Duration::from_secs(3)).await;
                    save_button.set_text(&ui, "Save");
                }
            })
        }
    });
    load_defaults_button.on_clicked(&ui, {
        let general_settings = general_settings.c();
        move |_| {
            if let Some(_device) = device.get_untracked() {
                general_settings.load_defaults();
                nf_settings.load_defaults();
                wf_settings.load_defaults();
            }
        }
    });

    reload_button.on_clicked(&ui, {
        let general_settings = general_settings.c();
        move |_| {
            if let Some(device) = device.get_untracked() {
                let general_settings = general_settings.c();
                ui_ctx.spawn(async move {
                    _ = general_settings.load_from_device(&device, false).await;
                    _ = nf_settings.load_from_device(&device).await;
                    _ = wf_settings.load_from_device(&device).await;
                });
            }
        }
    });

    (
        config_win,
        device.read_only(),
        general_settings.accel_config.read_only(),
    )
}

#[derive(Clone)]
struct GeneralSettingsForm {
    device_uuid: RwSignal<[u8; 6]>,
    device_pid: RwSignal<u16>,
    impact_threshold: RwSignal<i32>,
    accel_config: RwSignal<AccelConfig>,
    gyro_config: RwSignal<GyroConfig>,
    nf_intrinsics: RwSignal<RosOpenCvIntrinsics<f32>>,
    wf_intrinsics: RwSignal<RosOpenCvIntrinsics<f32>>,
    stereo_iso: RwSignal<nalgebra::Isometry3<f32>>,
    mot_runner: Arc<Mutex<MotRunner>>,
}

impl GeneralSettingsForm {
    fn new(
        ui: &UI,
        device: ReadSignal<Option<UsbDevice>>,
        mot_runner: Arc<Mutex<MotRunner>>,
        win: Window,
    ) -> (Form, Self) {
        let device_uuid = create_rw_signal([0; 6]);
        let device_pid = create_rw_signal(0u16);
        let connected = move || device.with(|d| d.is_some());
        let impact_threshold = create_rw_signal(0);
        let accel_config = create_rw_signal(AccelConfig::default());
        let gyro_config = create_rw_signal(GyroConfig::default());
        let nf_intrinsics =
            create_rw_signal(RosOpenCvIntrinsics::from_params(145., 0., 145., 45., 45.));
        let wf_intrinsics =
            create_rw_signal(RosOpenCvIntrinsics::from_params(34., 0., 34., 45., 45.));
        let stereo_iso = create_rw_signal(nalgebra::Isometry3::identity());
        crate::layout! { &ui,
            let form = Form(padded: true) {
                (Compact, "Device UUID") : let x = Label(move || {
                    if connected() {
                        let id = device_uuid.get();
                        format!(
                            "{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
                            id[0],
                            id[1],
                            id[2],
                            id[3],
                            id[4],
                            id[5],
                        )
                    } else {
                        "".into()
                    }
                })
                (Compact, "Impact threshold") : let x = Spinbox(enabled: connected, signal: impact_threshold)
                (Compact, "Accelerometer Config") : let x = HorizontalBox(padded: true) {
                    Compact : let upload_accel_config = Button("Upload")
                    Compact : let download_accel_config = Button("Download")
                }
                (Compact, "Gyroscope Config") : let x = HorizontalBox(padded: true) {
                    Compact : let upload_gyro_config = Button("Upload")
                    Compact : let download_gyro_config = Button("Download")
                }
                (Compact, "Nearfield Calibration") : let x = HorizontalBox(padded: true) {
                    Compact : let upload_nf_json = Button("Upload")
                    Compact : let download_nf_json = Button("Download")
                }
                (Compact, "Widefield Calibration") : let x = HorizontalBox(padded: true) {
                    Compact : let upload_wf_json = Button("Upload")
                    Compact : let download_wf_json = Button("Download")
                }
                (Compact, "Stereo Calibration") : let x = HorizontalBox(padded: true) {
                    Compact : let upload_stereo_json = Button("Upload")
                    Compact : let download_stereo_json = Button("Download")
                    Compact : let sync_stereo = Button("Sync")
                }
            }
        }
        set_calibration_upload_handlers(
            &ui,
            &mut upload_nf_json,
            &mut upload_wf_json,
            &mut upload_stereo_json,
            nf_intrinsics.c(),
            wf_intrinsics.c(),
            stereo_iso.c(),
            win.c(),
        );
        set_accel_upload_handler(&ui, &mut upload_accel_config, accel_config.c(), win.c());
        set_gyro_upload_handler(&ui, &mut upload_gyro_config, gyro_config.c(), win.c());
        set_calibration_download_handlers(
            &ui,
            &mut download_nf_json,
            &mut download_wf_json,
            &mut download_stereo_json,
            nf_intrinsics.c(),
            wf_intrinsics.c(),
            stereo_iso.c(),
            win.c(),
        );
        set_accel_download_handler(&ui, &mut download_accel_config, accel_config.c(), win.c());
        set_gyro_download_handler(&ui, &mut download_gyro_config, gyro_config.c(), win.c());

        sync_stereo.on_clicked(&ui, {
            let stereo_iso = stereo_iso.c();
            let mot_runner = mot_runner.c();
            move |_| {
                let iso = mot_runner.lock().general_config.stereo_iso;
                stereo_iso.set(iso);
            }
        });

        (
            form,
            Self {
                impact_threshold,
                accel_config,
                gyro_config,
                nf_intrinsics,
                wf_intrinsics,
                stereo_iso,
                mot_runner,
                device_uuid,
                device_pid,
            },
        )
    }

    async fn load_from_device(&self, device: &UsbDevice, first_load: bool) -> Result<u16> {
        let timeout = Duration::from_millis(5000);
        let config = retry(|| device.read_config(), timeout, 3).await.unwrap()?;
        let props = retry(|| device.read_props(), timeout, 3).await.unwrap()?;

        self.impact_threshold
            .set(i32::from(config.impact_threshold));
        self.accel_config.set(config.accel_config);
        self.gyro_config.set(config.gyro_config);
        self.nf_intrinsics.set(config.camera_model_nf.clone());
        self.wf_intrinsics.set(config.camera_model_wf.clone());
        self.stereo_iso.set(config.stereo_iso.clone());
        self.device_uuid.set(props.uuid);
        self.device_pid.set(props.product_id);

        if first_load {
            self.mot_runner.lock().general_config = config;
        }
        Ok(props.product_id)
    }

    fn validate(&self, errors: &mut Vec<String>) {
        macro_rules! validators {
            ($($display:literal $reg:ident : $ty:ty $({ $( $check:expr ),* $(,)? })? ),* $(,)?) => {
                $(
                    #[allow(unused_variables)]
                    let $reg = self.$reg.with_untracked(|s| s.parse::<$ty>());
                    match &$reg {
                        Ok(_) => (),
                        Err(e) => errors.push(format!("{}: {}", $display, e)),
                    }
                )*
                if !errors.is_empty() {
                    return
                }
                $(
                    // SAFETY: all the values are already checked for errors. Can convert to
                    // unwrap_unchecked if necessary.
                    #[allow(unused_variables)]
                    let $reg = $reg.unwrap();
                )*
                $(
                    $($(
                        let (test_result, msg) = ($check)($reg);
                        if !test_result {
                            errors.push(format!("{}: {}", $display, msg));
                        }
                    )*)?
                )*
            }
        }
        validators! {}
        if !(0..256).contains(&self.impact_threshold.get_untracked()) {
            errors.push("impact threshold: must be between 0 and 255".into());
        }
    }

    /// Make sure to call `validate()` before calling this method.
    async fn apply(&self, device: &UsbDevice) -> Result<()> {
        let config = GeneralConfig {
            impact_threshold: self.impact_threshold.get_untracked() as u8,
            accel_config: self.accel_config.get_untracked(),
            gyro_config: self.gyro_config.get_untracked(),
            camera_model_nf: self.nf_intrinsics.get_untracked(),
            camera_model_wf: self.wf_intrinsics.get_untracked(),
            stereo_iso: self.stereo_iso.get_untracked(),
        };
        device.write_config(config.clone()).await?;
        {
            let general_config = &mut self.mot_runner.lock().general_config;
            general_config.impact_threshold = config.impact_threshold;
            general_config.accel_config = config.accel_config;
            general_config.gyro_config = config.gyro_config;
            general_config.camera_model_nf = config.camera_model_nf;
            general_config.camera_model_wf = config.camera_model_wf;
            general_config.stereo_iso = config.stereo_iso;
        }
        Ok(())
    }

    fn clear(&self) {
        self.accel_config.set(AccelConfig::default());
        self.gyro_config.set(GyroConfig::default());
    }

    fn load_defaults(&self) {
        self.impact_threshold.set(5);
        self.accel_config.set(AccelConfig::default());
        self.gyro_config.set(GyroConfig::default());
        self.nf_intrinsics
            .set(RosOpenCvIntrinsics::from_params(145., 0., 145., 45., 45.));
        self.wf_intrinsics
            .set(RosOpenCvIntrinsics::from_params(34., 0., 34., 45., 45.));
        self.stereo_iso.set(nalgebra::Isometry3::identity());
    }
}

fn set_calibration_upload_handlers(
    ui: &UI,
    upload_nf: &mut Button,
    upload_wf: &mut Button,
    upload_stereo: &mut Button,
    nf_intrinsics: RwSignal<RosOpenCvIntrinsics<f32>>,
    wf_intrinsics: RwSignal<RosOpenCvIntrinsics<f32>>,
    stereo_iso: RwSignal<nalgebra::Isometry3<f32>>,
    win: Window,
) {
    upload_nf.on_clicked(&ui, {
        let ui = ui.c();
        let win = win.c();
        move |_| {
            if let Some(path) = win.open_file(&ui) {
                let Ok(()) = (|| {
                    let reader = std::fs::File::open(&path)?;
                    let intrinsics =
                        ats_common::get_intrinsics_from_opencv_camera_calibration_json(reader)?;
                    nf_intrinsics.set(intrinsics);
                    win.modal_msg(
                        &ui,
                        "Uploaded calibration",
                        "Successfully uploaded calibration",
                    );
                    Ok::<(), Box<dyn std::error::Error>>(())
                })() else {
                    win.modal_err(&ui, "Failed to upload calibration", "Failed to read file");
                    return;
                };
            } else {
                win.modal_err(&ui, "Failed to upload calibration", "No file selected");
            }
        }
    });

    upload_wf.on_clicked(&ui, {
        let ui = ui.c();
        let win = win.c();
        move |_| {
            if let Some(path) = win.open_file(&ui) {
                let Ok(()) = (|| {
                    let reader = std::fs::File::open(&path)?;
                    let intrinsics =
                        ats_common::get_intrinsics_from_opencv_camera_calibration_json(reader)?;
                    wf_intrinsics.set(intrinsics);
                    win.modal_msg(
                        &ui,
                        "Uploaded calibration",
                        "Successfully uploaded calibration",
                    );
                    Ok::<(), Box<dyn std::error::Error>>(())
                })() else {
                    win.modal_err(&ui, "Failed to upload calibration", "Failed to read file");
                    return;
                };
            } else {
                win.modal_err(&ui, "Failed to upload calibration", "No file selected");
            }
        }
    });

    upload_stereo.on_clicked(&ui, {
        let ui = ui.c();
        let win = win.c();
        move |_| {
            if let Some(path) = win.open_file(&ui) {
                let Ok(()) = (|| {
                    let reader = std::fs::File::open(&path)?;
                    let iso = ats_common::get_isometry_from_opencv_stereo_calibration_json(reader)?;
                    stereo_iso.set(iso);
                    win.modal_msg(
                        &ui,
                        "Uploaded calibration",
                        "Successfully uploaded calibration",
                    );
                    Ok::<(), Box<dyn std::error::Error>>(())
                })() else {
                    win.modal_err(&ui, "Failed to upload calibration", "Failed to read file");
                    return;
                };
            } else {
                win.modal_err(&ui, "Failed to upload calibration", "No file selected");
            }
        }
    });
}

fn set_calibration_download_handlers(
    ui: &UI,
    download_nf: &mut Button,
    download_wf: &mut Button,
    download_stereo: &mut Button,
    nf_intrinsics: RwSignal<RosOpenCvIntrinsics<f32>>,
    wf_intrinsics: RwSignal<RosOpenCvIntrinsics<f32>>,
    stereo_iso: RwSignal<nalgebra::Isometry3<f32>>,
    win: Window,
) {
    download_nf.on_clicked(&ui, {
        let ui = ui.c();
        let win = win.c();
        move |_| {
            if let Some(path) = win.save_file(&ui) {
                let Ok(()) = (|| {
                    let writer = std::fs::File::create(&path)?;
                    ats_common::write_opencv_minimal_camera_calibration_json(
                        &nf_intrinsics.get(),
                        writer,
                    )?;
                    win.modal_msg(
                        &ui,
                        "Downloaded calibration",
                        "Successfully downloaded calibration",
                    );
                    Ok::<(), Box<dyn std::error::Error>>(())
                })() else {
                    win.modal_err(
                        &ui,
                        "Failed to download calibration",
                        "Failed to write file",
                    );
                    return;
                };
            } else {
                win.modal_err(&ui, "Failed to download calibration", "No file selected");
            }
        }
    });

    download_wf.on_clicked(&ui, {
        let ui = ui.c();
        let win = win.c();
        move |_| {
            if let Some(path) = win.save_file(&ui) {
                let Ok(()) = (|| {
                    let writer = std::fs::File::create(&path)?;
                    ats_common::write_opencv_minimal_camera_calibration_json(
                        &wf_intrinsics.get(),
                        writer,
                    )?;
                    win.modal_msg(
                        &ui,
                        "Downloaded calibration",
                        "Successfully downloaded calibration",
                    );
                    Ok::<(), Box<dyn std::error::Error>>(())
                })() else {
                    win.modal_err(
                        &ui,
                        "Failed to download calibration",
                        "Failed to write file",
                    );
                    return;
                };
            } else {
                win.modal_err(&ui, "Failed to download calibration", "No file selected");
            }
        }
    });

    download_stereo.on_clicked(&ui, {
        let ui = ui.c();
        let win = win.c();
        move |_| {
            if let Some(path) = win.save_file(&ui) {
                let Ok(()) = (|| {
                    let writer = std::fs::File::create(&path)?;
                    ats_common::write_opencv_minimal_stereo_calibration_json(
                        &stereo_iso.get(),
                        writer,
                    )?;
                    win.modal_msg(
                        &ui,
                        "Downloaded calibration",
                        "Successfully downloaded calibration",
                    );
                    Ok::<(), Box<dyn std::error::Error>>(())
                })() else {
                    win.modal_err(
                        &ui,
                        "Failed to download calibration",
                        "Failed to write file",
                    );
                    return;
                };
            } else {
                win.modal_err(&ui, "Failed to download calibration", "No file selected");
            }
        }
    });
}

fn set_accel_upload_handler(
    ui: &UI,
    upload_accel: &mut Button,
    accel_config_signal: RwSignal<AccelConfig>,
    win: Window,
) {
    upload_accel.on_clicked(&ui, {
        let ui = ui.c();
        let win = win.c();
        move |_| {
            if let Some(path) = win.open_file(&ui) {
                let Ok(()) = (|| {
                    let reader = std::fs::File::open(&path)?;
                    let accel_config: AccelConfig = serde_json::from_reader(reader)?;
                    accel_config_signal.set(accel_config);
                    win.modal_msg(
                        &ui,
                        "Uploaded configuration",
                        "Successfully uploaded configuration",
                    );
                    Ok::<(), Box<dyn std::error::Error>>(())
                })() else {
                    win.modal_err(&ui, "Failed to upload configuration", "Failed to read file");
                    return;
                };
            } else {
                win.modal_err(&ui, "Failed to upload configuration", "No file selected");
            }
        }
    });
}

fn set_accel_download_handler(
    ui: &UI,
    download_accel: &mut Button,
    accel_config_signal: RwSignal<AccelConfig>,
    win: Window,
) {
    download_accel.on_clicked(&ui, {
        let ui = ui.c();
        let win = win.c();
        move |_| {
            if let Some(path) = win.save_file(&ui) {
                let Ok(()) = (|| {
                    let writer = std::fs::File::create(&path)?;
                    serde_json::to_writer(writer, &accel_config_signal.get())?;
                    win.modal_msg(
                        &ui,
                        "Downloaded configuration",
                        "Successfully downloaded configuration",
                    );
                    Ok::<(), Box<dyn std::error::Error>>(())
                })() else {
                    win.modal_err(
                        &ui,
                        "Failed to download configuration",
                        "Failed to write file",
                    );
                    return;
                };
            } else {
                win.modal_err(&ui, "Failed to download configuration", "No file selected");
            }
        }
    });
}

fn set_gyro_upload_handler(
    ui: &UI,
    upload_gyro: &mut Button,
    gyro_config_signal: RwSignal<GyroConfig>,
    win: Window,
) {
    upload_gyro.on_clicked(&ui, {
        let ui = ui.c();
        let win = win.c();
        move |_| {
            if let Some(path) = win.open_file(&ui) {
                let Ok(()) = (|| {
                    let reader = std::fs::File::open(&path)?;
                    let gyro_config: GyroConfig = serde_json::from_reader(reader)?;
                    gyro_config_signal.set(gyro_config);
                    win.modal_msg(
                        &ui,
                        "Uploaded configuration",
                        "Successfully uploaded configuration",
                    );
                    Ok::<(), Box<dyn std::error::Error>>(())
                })() else {
                    win.modal_err(&ui, "Failed to upload configuration", "Failed to read file");
                    return;
                };
            } else {
                win.modal_err(&ui, "Failed to upload configuration", "No file selected");
            }
        }
    });
}

fn set_gyro_download_handler(
    ui: &UI,
    download_gyro: &mut Button,
    gyro_config_signal: RwSignal<GyroConfig>,
    win: Window,
) {
    download_gyro.on_clicked(&ui, {
        let ui = ui.c();
        let win = win.c();
        move |_| {
            if let Some(path) = win.save_file(&ui) {
                let Ok(()) = (|| {
                    let writer = std::fs::File::create(&path)?;
                    serde_json::to_writer(writer, &gyro_config_signal.get())?;
                    win.modal_msg(
                        &ui,
                        "Downloaded configuration",
                        "Successfully downloaded configuration",
                    );
                    Ok::<(), Box<dyn std::error::Error>>(())
                })() else {
                    win.modal_err(
                        &ui,
                        "Failed to download configuration",
                        "Failed to write file",
                    );
                    return;
                };
            } else {
                win.modal_err(&ui, "Failed to download configuration", "No file selected");
            }
        }
    });
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
    out = out.replace('\x00', "");
    out
}

/// Retry an asynchronous operation up to `limit` times.
async fn retry<F, G>(mut op: F, timeout: Duration, limit: usize) -> Option<G::Output>
where
    F: FnMut() -> G,
    G: std::future::Future,
{
    for _ in 0..limit {
        match tokio::time::timeout(timeout, op()).await {
            Ok(r) => return Some(r),
            Err(_) => (),
        }
    }
    None
}
