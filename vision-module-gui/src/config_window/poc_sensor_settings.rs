use std::time::Duration;

use anyhow::Result;
use ats_usb::{device::UsbDevice, packets::vm::Port};
use iui::{controls::Form, UI};
use leptos_reactive::{
    create_rw_signal, ReadSignal, RwSignal, SignalGetUntracked, SignalSet, SignalUpdate, SignalWith,
};

use super::retry;

#[derive(Copy, Clone)]
pub struct PocSensorSettingsForm {
    cid: RwSignal<String>,
    fps: RwSignal<i32>,
    exposure_us: RwSignal<i32>,
    gain: RwSignal<i32>,
    area_threshold_min: RwSignal<i32>,
    area_threshold_max: RwSignal<i32>,
    light_threshold: RwSignal<i32>,
}

impl PocSensorSettingsForm {
    pub fn new(ui: &UI, device: ReadSignal<Option<UsbDevice>>) -> (Form, Self) {
        let connected = move || device.with(|d| d.is_some());
        let cid = create_rw_signal(String::new());
        let fps = create_rw_signal(0);
        let exposure_us = create_rw_signal(0);
        let area_threshold_min = create_rw_signal(0);
        let area_threshold_max = create_rw_signal(0);
        let light_threshold = create_rw_signal(0);
        let gain = create_rw_signal(0);

        crate::layout! { &ui,
            let form = Form(padded: true) {
                (Compact, "Chip ID")               : let chip_id = Entry(value: cid, enabled: false)
                (Compact, "FPS")                   : let x = Spinbox(enabled: connected, signal: fps)
                (Compact, "Exposure time (us)")    : let x = Spinbox(enabled: connected, signal: exposure_us)
                (Compact, "Gain")                  : let gain_combobox = Spinbox(enabled: connected, signal: gain)
                (Compact, "Light threshold")       : let x = Spinbox(enabled: connected, signal: light_threshold)
                (Compact, "Area threshold min")    : let x = Spinbox(enabled: connected, signal: area_threshold_min)
                (Compact, "Area threshold max")    : let x = Spinbox(enabled: connected, signal: area_threshold_max)
                (Compact, "Can I habe da img pls") : let img_mode_btn = Button("👍", enabled: connected)
            }
        }

        let ui_ctx = ui.async_context();

        img_mode_btn.on_clicked(&ui, move |_| {
            let timeout = Duration::from_millis(2000);
            if let Some(device) = device.get_untracked() {
                ui_ctx.spawn(async move {
                    _ = retry(
                        || device.write_mode(ats_usb::packets::vm::Mode::Image),
                        timeout,
                        3,
                    )
                    .await
                    .unwrap();
                });
            }
        });

        (
            form,
            Self {
                cid,
                fps,
                exposure_us,
                gain,
                area_threshold_min,
                area_threshold_max,
                light_threshold,
            },
        )
    }

    pub async fn load_from_device(&self, device: &UsbDevice) -> Result<()> {
        self.cid.set("Connecting...".into());
        let timeout = Duration::from_millis(2000);
        let cid = retry(|| device.pag_chip_id(Port::Nf), timeout, 3)
            .await
            .unwrap()?;
        let fps = retry(|| device.pag_fps(Port::Nf), timeout, 3)
            .await
            .unwrap()?;
        let expo = retry(|| device.pag_exposure(Port::Nf), timeout, 3)
            .await
            .unwrap()?;
        let _led_always_on = expo & (1 << 7) != 0;
        let area_threshold_min = retry(|| device.pag_area_lower(Port::Nf), timeout, 3)
            .await
            .unwrap()?;
        let area_threshold_max = retry(|| device.pag_area_upper(Port::Nf), timeout, 3)
            .await
            .unwrap()?;
        let light_threshold = retry(|| device.pag_light_threshold(Port::Nf), timeout, 3)
            .await
            .unwrap()?;
        let gain = retry(|| device.pag_gain(Port::Nf), timeout, 3)
            .await
            .unwrap()?;

        self.cid.set(format!("0x{cid:04x}"));
        self.fps.set(i32::from(fps));
        self.exposure_us
            .set(i32::from(u16::from(expo & !(1 << 7)) * 100));
        self.gain.set(i32::from(gain));

        self.area_threshold_min
            .set(i32::from(u16::from(area_threshold_min)));
        self.area_threshold_max
            .set(i32::from(u16::from(area_threshold_max)));
        self.light_threshold.set(i32::from(light_threshold));
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
        let fps = u8::try_from(self.fps.get_untracked()).unwrap();

        let gain = u8::try_from(self.gain.get_untracked()).unwrap();

        let exposure_us = self.exposure_us.get_untracked();
        assert!(exposure_us % 100 == 0);
        assert!(exposure_us <= 12700);
        // TODO
        let led_always_on = true;
        let exposure = (exposure_us / 100) as u8 | ((led_always_on as u8) << 7);

        let area_threshold_min = u16::try_from(self.area_threshold_min.get_untracked()).unwrap();
        let area_threshold_max = u16::try_from(self.area_threshold_max.get_untracked()).unwrap();
        let light_threshold = u8::try_from(self.light_threshold.get_untracked()).unwrap();

        tokio::try_join!(
            device.set_pag_fps(Port::Nf, fps),
            device.set_pag_gain(Port::Nf, gain),
            device.set_pag_exposure(Port::Nf, exposure,),
            device.set_pag_area_lower(Port::Nf, area_threshold_min),
            device.set_pag_area_upper(Port::Nf, area_threshold_max),
            device.set_pag_light_threshold(Port::Nf, light_threshold),
        )?;
        Ok(())
    }

    pub fn clear(&self) {
        self.cid.update(String::clear);
        self.fps.set(0);
        self.exposure_us.set(0);
        self.gain.set(0);

        self.area_threshold_min.set(0);
        self.area_threshold_max.set(0);
        self.light_threshold.set(0);
    }

    pub fn load_defaults(&self) {
        // self.frame_period.update(|s| s.replace_range(.., "49780"));
        // self.brightness_threshold
        //     .update(|s| s.replace_range(.., "110"));
        // self.noise_threshold.update(|s| s.replace_range(.., "15"));
        // self.area_threshold_max
        //     .update(|s| s.replace_range(.., "9605"));
        // self.max_object_cnt.update(|s| s.replace_range(.., "16"));

        // self.operation_mode.set(0);
        // self.frame_subtraction.set(0);

        self.fps.set(180);
        self.exposure_us.set(2000);
        self.gain.set(2);
        self.area_threshold_min.set(10);
        self.area_threshold_max.set(u16::MAX.into());
        self.light_threshold.set(120);
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
