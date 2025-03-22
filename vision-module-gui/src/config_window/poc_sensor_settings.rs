use std::time::Duration;

use anyhow::Result;
use ats_usb::{device::UsbDevice, packets::vm::Port};
use iui::{controls::Form, UI};
use leptos_reactive::{create_rw_signal, ReadSignal, RwSignal, SignalGetUntracked, SignalSet, SignalUpdate, SignalWith, SignalWithUntracked};

use super::retry;

#[derive(Copy, Clone)]
pub struct PocSensorSettingsForm {
    pid: RwSignal<String>,
    // resolution_x: RwSignal<String>,
    // resolution_y: RwSignal<String>,
    // exposure_time: RwSignal<String>,
    // frame_period: RwSignal<String>,
    // brightness_threshold: RwSignal<String>,
    // noise_threshold: RwSignal<String>,
    // area_threshold_min: RwSignal<String>,
    // area_threshold_max: RwSignal<String>,
    // max_object_cnt: RwSignal<String>,

    // operation_mode: RwSignal<i32>,
    // frame_subtraction: RwSignal<i32>,
    // gain: RwSignal<i32>,
}

impl PocSensorSettingsForm {
    pub fn new(ui: &UI, device: ReadSignal<Option<UsbDevice>>) -> (Form, Self) {
        let connected = move || device.with(|d| d.is_some());
        let pid = create_rw_signal(String::new());
        // let resolution_x = create_rw_signal(String::new());
        // let resolution_y = create_rw_signal(String::new());
        // let exposure_time = create_rw_signal(String::new());
        // let frame_period = create_rw_signal(String::new());
        // let brightness_threshold = create_rw_signal(String::new());
        // let noise_threshold = create_rw_signal(String::new());
        // let area_threshold_min = create_rw_signal(String::new());
        // let area_threshold_max = create_rw_signal(String::new());
        // let max_object_cnt = create_rw_signal(String::new());
        // let operation_mode = create_rw_signal(0);
        // let frame_subtraction = create_rw_signal(0);
        // let gain = create_rw_signal(0);

        // let exposure_time_ms = move || match exposure_time.with(|s| s.parse::<u16>()) {
        //     Ok(n) => format!("{:.4}", f64::from(n) * 200.0 / 1e6),
        //     Err(_) => format!("???"),
        // };
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
                (Compact, "Product ID")               : let product_id = Entry(value: pid, enabled: false)
                // (Compact, "DSP brightness threshold") : let x = Entry(enabled: connected, signal: brightness_threshold)
                // (Compact, "DSP noise threshold")      : let x = Entry(enabled: connected, signal: noise_threshold)
                // (Compact, "DSP area threshold min")   : let x = Entry(enabled: connected, signal: area_threshold_min)
                // (Compact, "DSP area threshold max")   : let x = Entry(enabled: connected, signal: area_threshold_max)
                // (Compact, "DSP maximum object count") : let x = Entry(enabled: connected, signal: max_object_cnt)
                // (Compact, "DSP operation mode")       : let x = Combobox(enabled: connected, signal: operation_mode) { "Normal", "Tracking" }
                // (Compact, "Exposure time") : let x = HorizontalBox(padded: true) {
                //     Stretchy : let e = Entry(
                //         enabled: connected,
                //         signal: exposure_time,
                //     )
                //     Compact : let expo_label = LayoutGrid() {
                //         (0, 0)(1, 1) Vertical (Start, Center) : let s = Label(move || format!("× 200ns = {} ms", exposure_time_ms()))
                //     }
                // }
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
                // (Compact, "Gain")               : let gain_combobox = Combobox(enabled: connected, signal: gain) {}
                // (Compact, "Scale resolution X") : let x = Entry(enabled: connected, signal: resolution_x)
                // (Compact, "Scale resolution Y") : let x = Entry(enabled: connected, signal: resolution_y)
            }
        }
        // for (label, _) in &GAIN_TABLE {
        //     gain_combobox.append(&ui, label);
        // }
        (
            form,
            Self {
                pid,
                // resolution_x,
                // resolution_y,
                // exposure_time,
                // frame_period,
                // brightness_threshold,
                // noise_threshold,
                // area_threshold_min,
                // area_threshold_max,
                // max_object_cnt,
                // operation_mode,
                // frame_subtraction,
                // gain,
            },
        )
    }

    pub async fn load_from_device(&self, device: &UsbDevice) -> Result<()> {
        self.pid.set("Connecting...".into());
        let timeout = Duration::from_millis(2000);
        let pid = retry(|| device.product_id(Port::Nf), timeout, 3)
            .await
            .unwrap()?;
        // let res_x = retry(|| device.resolution_x(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        // let res_y = retry(|| device.resolution_y(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        // let expo = retry(|| device.exposure_time(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
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
        // let gain_1 = retry(|| device.gain_1(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;
        // let gain_2 = retry(|| device.gain_2(self.port), timeout, 3)
        //     .await
        //     .unwrap()?;

        self.pid.set(format!("0x{pid:04x}"));
        // self.resolution_x.set(res_x.to_string());
        // self.resolution_y.set(res_y.to_string());
        // self.exposure_time.set(expo.to_string());
        // self.frame_period.set(frame_period.to_string());
        // self.brightness_threshold
        //     .set(brightness_threshold.to_string());
        // self.noise_threshold.set(noise_threshold.to_string());
        // self.area_threshold_min.set(area_threshold_min.to_string());
        // self.area_threshold_max.set(area_threshold_max.to_string());
        // self.max_object_cnt.set(max_object_cnt.to_string());

        // self.operation_mode.set(i32::from(operation_mode));
        // self.frame_subtraction.set(i32::from(frame_subtraction));
        // self.gain
        //     .set(i32::from(Gain::index_from_reg(gain_1, gain_2)));
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
        // let gain = usize::try_from(self.gain.get_untracked()).unwrap();
        // let gain = GAIN_TABLE[gain].1;

        // tokio::try_join!(
        //     device.set_resolution_x(
        //         self.port,
        //         self.resolution_x.with_untracked(|v| v.parse().unwrap())
        //     ),
        //     device.set_resolution_y(
        //         self.port,
        //         self.resolution_y.with_untracked(|v| v.parse().unwrap())
        //     ),
        //     device.set_gain_1(self.port, gain.b_global),
        //     device.set_gain_2(self.port, gain.b_ggh),
        //     device.set_exposure_time(
        //         self.port,
        //         self.exposure_time.with_untracked(|v| v.parse().unwrap())
        //     ),
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
        // )?;
        // tokio::try_join!(
        //     device.set_bank1_sync_updated(self.port, 1),
        //     device.set_bank0_sync_updated(self.port, 1),
        // )?;
        Ok(())
    }

    pub fn clear(&self) {
        self.pid.update(String::clear);
        // self.resolution_x.update(String::clear);
        // self.resolution_y.update(String::clear);
        // self.exposure_time.update(String::clear);
        // self.frame_period.update(String::clear);
        // self.brightness_threshold.update(String::clear);
        // self.noise_threshold.update(String::clear);
        // self.area_threshold_min.update(String::clear);
        // self.area_threshold_max.update(String::clear);
        // self.max_object_cnt.update(String::clear);

        // self.operation_mode.set(0);
        // self.frame_subtraction.set(0);
        // self.gain.set(0);
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

        // match self.port {
        //     Port::Nf => {
        //         self.area_threshold_min
        //             .update(|s| s.replace_range(.., "10"));
        //         self.exposure_time.update(|s| s.replace_range(.., "8192"));
        //         self.gain.set(i32::from(Gain::index_from_reg(16, 0)));
        //     }
        //     Port::Wf => {
        //         self.area_threshold_min.update(|s| s.replace_range(.., "5"));
        //         self.exposure_time.update(|s| s.replace_range(.., "11365"));
        //         self.gain.set(i32::from(Gain::index_from_reg(8, 3)));
        //     }
        // }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Gain {
    b_ggh: u8,
    b_global: u8,
}

impl Gain {
    const fn new(b_global: u8, b_ggh: u8) -> Self {
        Self { b_global, b_ggh }
    }
}

impl Gain {
    fn index_from_reg(b_global: u8, mut b_ggh: u8) -> i32 {
        // change b_ggh from 0,2,3 to 0,1,2
        if b_ggh > 0 {
            b_ggh -= 1;
        }
        let b_ggh = i32::from(b_ggh);
        let b_global = i32::from(b_global);
        b_ggh * 16 + b_global
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

// static GAIN_TABLE: [(&'static str, Gain); 16 * 3] = [
// for j, k in zip([0, 2, 3], [1, 2, 4]):
//     for i in range(16):
//         value = (1 + i/16) * k
//         print(f'    ("{value:.4f}", Gain::new(0x{i:02x}, 0x{j:02x})),')
// ];

pub static GAIN_TABLE: [(&str, Gain); 16 * 3 + 1] = {
    const BACKING: [[u8; 6]; 16 * 3] = {
        let mut v = [[0; 6]; 16 * 3];
        let mut i = 0;
        while i < 16 {
            v[i] = n_to_bstr((10000 + i * 10000 / 16) * 1);
            v[i + 16] = n_to_bstr((10000 + i * 10000 / 16) * 2);
            v[i + 32] = n_to_bstr((10000 + i * 10000 / 16) * 4);
            i += 1;
        }
        v
    };
    let mut omegalul = [("", Gain::new(0, 0)); 16 * 3 + 1];
    let mut i = 0;
    while i < 16 * 3 {
        omegalul[i].0 = unsafe { std::str::from_utf8_unchecked(&BACKING[i]) };
        omegalul[i].1 = Gain::new((i % 16) as u8, [0, 2, 3][i / 16]);
        i += 1;
    }
    omegalul[16 * 3] = ("8.0000", Gain::new(16, 3));
    omegalul
};
