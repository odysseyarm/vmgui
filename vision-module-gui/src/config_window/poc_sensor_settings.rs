use std::time::Duration;

use anyhow::Result;
use ats_usb::{device::UsbDevice, packets::vm::Port};
use iui::{controls::Form, UI};
use leptos_reactive::{create_rw_signal, ReadSignal, RwSignal, SignalGetUntracked, SignalSet, SignalUpdate, SignalWith, SignalWithUntracked};

use super::retry;

#[derive(Copy, Clone)]
pub struct PocSensorSettingsForm {
    cid: RwSignal<String>,
    // resolution_x: RwSignal<String>,
    // resolution_y: RwSignal<String>,
    exposure_us: RwSignal<i32>,
    // frame_period: RwSignal<String>,
    // brightness_threshold: RwSignal<String>,
    // noise_threshold: RwSignal<String>,
    // area_threshold_min: RwSignal<String>,
    // area_threshold_max: RwSignal<String>,
    // max_object_cnt: RwSignal<String>,

    // operation_mode: RwSignal<i32>,
    // frame_subtraction: RwSignal<i32>,
    gain: RwSignal<i32>,
}

impl PocSensorSettingsForm {
    pub fn new(ui: &UI, device: ReadSignal<Option<UsbDevice>>) -> (Form, Self) {
        let connected = move || device.with(|d| d.is_some());
        let cid = create_rw_signal(String::new());
        // let resolution_x = create_rw_signal(String::new());
        // let resolution_y = create_rw_signal(String::new());
        let exposure_us = create_rw_signal(0);
        // let frame_period = create_rw_signal(String::new());
        // let brightness_threshold = create_rw_signal(String::new());
        // let noise_threshold = create_rw_signal(String::new());
        // let area_threshold_min = create_rw_signal(String::new());
        // let area_threshold_max = create_rw_signal(String::new());
        // let max_object_cnt = create_rw_signal(String::new());
        // let operation_mode = create_rw_signal(0);
        // let frame_subtraction = create_rw_signal(0);
        let gain = create_rw_signal(0);

        // let frame_period_ms = move || match frame_period.with(|s| s.parse::<u32>()) {
        //     Ok(n) => format!("{:.4}", f64::from(n) / 1e4),
        //     Err(_) => format!("???"),
        // };
        // let fps = move || match frame_period.with(|s| s.parse::<u32>()) {
        //     Ok(n) => format!("{:.2}", 1e7 / f64::from(n)),
        //     Err(_) => format!("???"),
        // };
        crate::layout! { &ui,
            let form = Form(padded: true) {
                (Compact, "Chip ID")               : let chip_id = Entry(value: cid, enabled: false)
                // (Compact, "DSP brightness threshold") : let x = Entry(enabled: connected, signal: brightness_threshold)
                // (Compact, "DSP noise threshold")      : let x = Entry(enabled: connected, signal: noise_threshold)
                // (Compact, "DSP area threshold min")   : let x = Entry(enabled: connected, signal: area_threshold_min)
                // (Compact, "DSP area threshold max")   : let x = Entry(enabled: connected, signal: area_threshold_max)
                // (Compact, "DSP maximum object count") : let x = Entry(enabled: connected, signal: max_object_cnt)
                // (Compact, "DSP operation mode")       : let x = Combobox(enabled: connected, signal: operation_mode) { "Normal", "Tracking" }
                (Compact, "Exposure time (us)")          : let x = Spinbox(enabled: connected, signal: exposure_us)
                // (Compact, "Frame period") : let x = HorizontalBox(padded: true) {
                //     Stretchy : let e = Entry(
                //         enabled: connected,
                //         signal: frame_period,
                //     )
                //     Compact : let fps_label = LayoutGrid() {
                //         (0, 0)(1, 1) Vertical (Start, Center) : let s = Label(move || format!("{} ms ({} fps)", frame_period_ms(), fps()))
                //     }
                // }
                // (Compact, "Frame subtraction")  : let x = Combobox(enabled: connected, signal: frame_subtraction) { "Off", "On" }
                (Compact, "Gain")               : let gain_combobox = Spinbox(enabled: connected, signal: gain)
                // (Compact, "Scale resolution X") : let x = Entry(enabled: connected, signal: resolution_x)
                // (Compact, "Scale resolution Y") : let x = Entry(enabled: connected, signal: resolution_y)
            }
        }
        (
            form,
            Self {
                cid,
                // resolution_x,
                // resolution_y,
                exposure_us,
                // frame_period,
                // brightness_threshold,
                // noise_threshold,
                // area_threshold_min,
                // area_threshold_max,
                // max_object_cnt,
                // operation_mode,
                // frame_subtraction,
                gain,
            },
        )
    }

    pub async fn load_from_device(&self, device: &UsbDevice) -> Result<()> {
        self.cid.set("Connecting...".into());
        let timeout = Duration::from_millis(2000);
        let cid = retry(|| device.pag_chip_id(Port::Nf), timeout, 3)
            .await
            .unwrap()?;
        // let res_x = retry(|| device.resolution_x(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        // let res_y = retry(|| device.resolution_y(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        let expo = retry(|| device.poc_exposure(Port::Nf), timeout, 3)
            .await
            .unwrap()?;
        let _led_always_on = expo & (1 << 7) != 0;
        // let frame_period = retry(|| device.frame_period(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        // let brightness_threshold = retry(|| device.brightness_threshold(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        // let noise_threshold = retry(|| device.noise_threshold(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        // let area_threshold_min = retry(|| device.area_threshold_min(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        // let area_threshold_max = device.area_threshold_max(self.port).await?;
        // let max_object_cnt = retry(|| device.max_object_cnt(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        // let operation_mode = retry(|| device.operation_mode(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        // let frame_subtraction = retry(|| device.frame_subtraction(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        let gain = retry(|| device.poc_gain(Port::Nf), timeout, 3)
            .await
            .unwrap()?;

        self.cid.set(format!("0x{cid:04x}"));
        // self.resolution_x.set(res_x.to_string());
        // self.resolution_y.set(res_y.to_string());
        self.exposure_us.set(i32::from(u16::from(expo & !(1 << 7)) * 100));
        // self.frame_period.set(frame_period.to_string());
        // self.brightness_threshold
        //     .set(brightness_threshold.to_string());
        // self.noise_threshold.set(noise_threshold.to_string());
        // self.area_threshold_min.set(area_threshold_min.to_string());
        // self.area_threshold_max.set(area_threshold_max.to_string());
        // self.max_object_cnt.set(max_object_cnt.to_string());

        // self.operation_mode.set(i32::from(operation_mode));
        // self.frame_subtraction.set(i32::from(frame_subtraction));
        self.gain.set(i32::from(gain));
        Ok(())
    }

    pub fn validate(&self, errors: &mut Vec<String>) {
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
        validators! {
            // "exposure time" exposure_time: u16 {
            //     |x| (x >= 100, "must be >= 20 µs"),
            //     |x| ((200..=i64::from(frame_period) - 27000).contains(&(i64::from(x)*2)), "must be between 20 µs and frame period − 2.7 ms"),
            // },
            // "frame period" frame_period: u32 { |x| (x >= 49780, "must be >= 4.978 ms") },
            // "brightness threshold" brightness_threshold: u8,
            // "noise threshold" noise_threshold: u8,
            // "area threshold min" area_threshold_min: u8,
            // "area threshold max" area_threshold_max: u16 { |x| (x < (1 << 14), "must be < 16384") },
            // "max object count" max_object_cnt: u8 { |x| ((1..=16).contains(&x), "must be between 1 and 16") },
            // "scale resolution X" resolution_x: u16 { |x| ((1..=4095).contains(&x), "must be between 1 and 4095") },
            // "scale resolution Y" resolution_y: u16 { |x| ((1..=4095).contains(&x), "must be between 1 and 4095") },
        }
    }

    /// Make sure to call `validate()` before calling this method.
    pub async fn apply(&self, device: &UsbDevice) -> Result<()> {
        let gain = u8::try_from(self.gain.get_untracked()).unwrap();

        let exposure_us = self.exposure_us.get_untracked();
        assert!(exposure_us % 100 == 0);
        assert!(exposure_us <= 12700);
        // TODO
        let led_always_on = true;
        let exposure = (exposure_us / 100) as u8 | ((led_always_on as u8) << 7);

        tokio::try_join!(
        //     device.set_resolution_x(
        //         self.port,
        //         self.resolution_x.with_untracked(|v| v.parse().unwrap())
        //     ),
        //     device.set_resolution_y(
        //         self.port,
        //         self.resolution_y.with_untracked(|v| v.parse().unwrap())
        //     ),
            device.set_poc_gain(Port::Nf, gain),
            device.set_poc_exposure(
                Port::Nf,
                exposure,
            ),
        //     device.set_brightness_threshold(
        //         self.port,
        //         self.brightness_threshold
        //             .with_untracked(|v| v.parse().unwrap())
        //     ),
        //     device.set_noise_threshold(
        //         self.port,
        //         self.noise_threshold.with_untracked(|v| v.parse().unwrap())
        //     ),
        //     device.set_area_threshold_max(
        //         self.port,
        //         self.area_threshold_max
        //             .with_untracked(|v| v.parse().unwrap())
        //     ),
        //     device.set_area_threshold_min(
        //         self.port,
        //         self.area_threshold_min
        //             .with_untracked(|v| v.parse().unwrap())
        //     ),
        //     device.set_operation_mode(
        //         self.port,
        //         u8::try_from(self.operation_mode.get_untracked()).unwrap()
        //     ),
        //     device.set_max_object_cnt(
        //         self.port,
        //         self.max_object_cnt.with_untracked(|v| v.parse().unwrap())
        //     ),
        //     device.set_frame_subtraction(
        //         self.port,
        //         u8::try_from(self.frame_subtraction.get_untracked()).unwrap()
        //     ),
        //     device.set_frame_period(
        //         self.port,
        //         self.frame_period.with_untracked(|v| v.parse().unwrap())
        //     ),
        )?;
        // tokio::try_join!(
        //     device.set_bank1_sync_updated(self.port, 1),
        //     device.set_bank0_sync_updated(self.port, 1),
        // )?;
        Ok(())
    }

    pub fn clear(&self) {
        self.cid.update(String::clear);
        // self.resolution_x.update(String::clear);
        // self.resolution_y.update(String::clear);
        self.exposure_us.set(0);
        // self.frame_period.update(String::clear);
        // self.brightness_threshold.update(String::clear);
        // self.noise_threshold.update(String::clear);
        // self.area_threshold_min.update(String::clear);
        // self.area_threshold_max.update(String::clear);
        // self.max_object_cnt.update(String::clear);

        // self.operation_mode.set(0);
        // self.frame_subtraction.set(0);
        self.gain.set(0);
    }

    pub fn load_defaults(&self) {
        // self.resolution_x.update(|s| s.replace_range(.., "4095"));
        // self.resolution_y.update(|s| s.replace_range(.., "4095"));
        // self.frame_period.update(|s| s.replace_range(.., "49780"));
        // self.brightness_threshold
        //     .update(|s| s.replace_range(.., "110"));
        // self.noise_threshold.update(|s| s.replace_range(.., "15"));
        // self.area_threshold_max
        //     .update(|s| s.replace_range(.., "9605"));
        // self.max_object_cnt.update(|s| s.replace_range(.., "16"));

        // self.operation_mode.set(0);
        // self.frame_subtraction.set(0);

        self.gain.set(2);
        // match self.port {
        //     Port::Nf => {
        //         self.area_threshold_min
        //             .update(|s| s.replace_range(.., "10"));
        //         self.exposure_time.update(|s| s.replace_range(.., "8192"));
        //     }
        //     Port::Wf => {
        //         self.area_threshold_min.update(|s| s.replace_range(.., "5"));
        //         self.exposure_time.update(|s| s.replace_range(.., "11365"));
        //     }
        // }
    }
}
