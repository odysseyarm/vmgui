use std::time::Duration;

use anyhow::Result;
use ats_usb::{device::VmDevice, packets::vm::Port};
use iui::{controls::Form, UI};
use leptos_reactive::{
    create_rw_signal, ReadSignal, RwSignal, SignalGet, SignalGetUntracked, SignalSet, SignalUpdate,
    SignalWith,
};

use super::retry;

#[derive(Copy, Clone)]
pub struct PagSensorSettingsForm {
    cid: RwSignal<String>,
    fps: RwSignal<i32>,
    exposure_us: RwSignal<i32>,
    gain: RwSignal<i32>,
    area_threshold_min: RwSignal<i32>,
    area_threshold_max: RwSignal<i32>,
    light_threshold: RwSignal<i32>,
    // Circle detection parameters (PAG7665QN)
    circle_r_min: RwSignal<i32>,
    circle_r_max: RwSignal<i32>,
    circle_k_min: RwSignal<i32>,
    circle_k_max: RwSignal<i32>,
}

impl PagSensorSettingsForm {
    pub fn new(ui: &UI, device: ReadSignal<Option<VmDevice>>) -> (Form, Self) {
        let connected = move || device.with(|d| d.is_some());
        let cid = create_rw_signal(String::new());
        let fps = create_rw_signal(0);
        let exposure_us = create_rw_signal(0);
        let area_threshold_min = create_rw_signal(0);
        let area_threshold_max = create_rw_signal(0);
        let light_threshold = create_rw_signal(0);
        let gain = create_rw_signal(0);
        // Circle detection parameters
        let circle_r_min = create_rw_signal(0);
        let circle_r_max = create_rw_signal(0);
        let circle_k_min = create_rw_signal(0);
        let circle_k_max = create_rw_signal(0);

        crate::layout! { &ui,
            let form = Form(padded: true) {
                (Compact, "Chip ID")               : let chip_id = Entry(value: cid, enabled: false)
                (Compact, "FPS")                   : let x = Spinbox(enabled: connected, signal: fps)
                (Compact, "Exposure time (us)")    : let x = Spinbox(enabled: connected, signal: exposure_us)
                (Compact, "Gain")                  : let gain_combobox = Spinbox(enabled: connected, signal: gain)
                (Compact, "Light threshold")       : let x = Spinbox(enabled: connected, signal: light_threshold)
                (Compact, "Area threshold min")    : let x = Spinbox(enabled: connected, signal: area_threshold_min)
                (Compact, "Area threshold max")    : let x = Spinbox(enabled: connected, signal: area_threshold_max)
                (Compact, "Circle R min")          : let x = HorizontalBox(padded: true) {
                    Stretchy : let e = Spinbox(enabled: connected, signal: circle_r_min)
                    Compact : let l = Label(move || format!("= {:.3}", circle_r_min.get() as f32 / 32.0))
                }
                (Compact, "Circle R max")          : let x = HorizontalBox(padded: true) {
                    Stretchy : let e = Spinbox(enabled: connected, signal: circle_r_max)
                    Compact : let l = Label(move || format!("= {:.3}", circle_r_max.get() as f32 / 32.0))
                }
                (Compact, "Circle K min")          : let x = HorizontalBox(padded: true) {
                    Stretchy : let e = Spinbox(enabled: connected, signal: circle_k_min)
                    Compact : let l = Label(move || format!("= {:.3}", circle_k_min.get() as f32 / 32.0))
                }
                (Compact, "Circle K max")          : let x = HorizontalBox(padded: true) {
                    Stretchy : let e = Spinbox(enabled: connected, signal: circle_k_max)
                    Compact : let l = Label(move || format!("= {:.3}", circle_k_max.get() as f32 / 32.0))
                }
                (Compact, "Image mode")            : let img_mode_btn = Button("Capture", enabled: connected)
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
                circle_r_min,
                circle_r_max,
                circle_k_min,
                circle_k_max,
            },
        )
    }

    pub async fn load_from_device(&self, device: &VmDevice) -> Result<()> {
        self.cid.set("Connecting...".into());
        let timeout = Duration::from_millis(2000);

        // Check product ID to determine PAG variant (PAG7665QN vs PAG7661QN)
        let product_id = retry(
            || device.read_prop(ats_usb::packets::vm::PropKind::ProductId),
            timeout,
            3,
        )
        .await
        .and_then(|r| r.ok())
        .and_then(|p| match p {
            ats_usb::packets::vm::Props::ProductId(id) => ats_usb::device::ProductId::from_u16(id),
            _ => None,
        });
        let is_pag7665 = product_id == Some(ats_usb::device::ProductId::AtsPro);

        let cid = retry(|| device.pag_chip_id(Port::Nf), timeout, 3)
            .await
            .unwrap()?;
        let fps = retry(|| device.pag_fps(Port::Nf), timeout, 3)
            .await
            .unwrap()?;

        // Use correct register addresses based on PAG variant
        let (expo, area_threshold_min, area_threshold_max, light_threshold, gain) = if is_pag7665 {
            let expo = retry(|| device.pag7665_exposure(Port::Nf), timeout, 3)
                .await
                .unwrap()?;
            let area_threshold_min = retry(|| device.pag7665_area_lower(Port::Nf), timeout, 3)
                .await
                .unwrap()?;
            let area_threshold_max = retry(|| device.pag7665_area_upper(Port::Nf), timeout, 3)
                .await
                .unwrap()?;
            let light_threshold = retry(|| device.pag7665_light_threshold(Port::Nf), timeout, 3)
                .await
                .unwrap()?;
            let gain = retry(|| device.pag7665_gain(Port::Nf), timeout, 3)
                .await
                .unwrap()?;
            (
                expo,
                area_threshold_min,
                area_threshold_max,
                light_threshold,
                gain,
            )
        } else {
            let expo = retry(|| device.pag_exposure(Port::Nf), timeout, 3)
                .await
                .unwrap()?;
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
            (
                expo,
                area_threshold_min,
                area_threshold_max,
                light_threshold,
                gain,
            )
        };
        let _led_always_on = expo & (1 << 7) != 0;

        // Circle detection parameters (PAG7665QN)
        let circle_r_min = retry(|| device.pag_circle_r_min(Port::Nf), timeout, 3)
            .await
            .unwrap_or(Ok(0))?;
        let circle_r_max = retry(|| device.pag_circle_r_max(Port::Nf), timeout, 3)
            .await
            .unwrap_or(Ok(255))?;
        let circle_k_min = retry(|| device.pag_circle_k_min(Port::Nf), timeout, 3)
            .await
            .unwrap_or(Ok(0))?;
        let circle_k_max = retry(|| device.pag_circle_k_max(Port::Nf), timeout, 3)
            .await
            .unwrap_or(Ok(255))?;

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

        self.circle_r_min.set(i32::from(circle_r_min));
        self.circle_r_max.set(i32::from(circle_r_max));
        self.circle_k_min.set(i32::from(circle_k_min));
        self.circle_k_max.set(i32::from(circle_k_max));
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
    pub async fn apply(&self, device: &VmDevice) -> Result<()> {
        // Check product ID to determine PAG variant (PAG7665QN vs PAG7661QN)
        let product_id = device
            .read_prop(ats_usb::packets::vm::PropKind::ProductId)
            .await
            .ok()
            .and_then(|p| match p {
                ats_usb::packets::vm::Props::ProductId(id) => {
                    ats_usb::device::ProductId::from_u16(id)
                }
                _ => None,
            });
        let is_pag7665 = product_id == Some(ats_usb::device::ProductId::AtsPro);

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

        let circle_r_min = u8::try_from(self.circle_r_min.get_untracked()).unwrap();
        let circle_r_max = u8::try_from(self.circle_r_max.get_untracked()).unwrap();
        let circle_k_min = u8::try_from(self.circle_k_min.get_untracked()).unwrap();
        let circle_k_max = u8::try_from(self.circle_k_max.get_untracked()).unwrap();

        // Use correct register addresses based on PAG variant
        if is_pag7665 {
            tokio::try_join!(
                device.set_pag_fps(Port::Nf, fps),
                device.set_pag7665_gain(Port::Nf, gain),
                device.set_pag7665_exposure(Port::Nf, exposure),
                device.set_pag7665_area_lower(Port::Nf, area_threshold_min),
                device.set_pag7665_area_upper(Port::Nf, area_threshold_max),
                device.set_pag7665_light_threshold(Port::Nf, light_threshold),
                device.set_pag_circle_r_min(Port::Nf, circle_r_min),
                device.set_pag_circle_r_max(Port::Nf, circle_r_max),
                device.set_pag_circle_k_min(Port::Nf, circle_k_min),
                device.set_pag_circle_k_max(Port::Nf, circle_k_max),
            )?;
        } else {
            tokio::try_join!(
                device.set_pag_fps(Port::Nf, fps),
                device.set_pag_gain(Port::Nf, gain),
                device.set_pag_exposure(Port::Nf, exposure),
                device.set_pag_area_lower(Port::Nf, area_threshold_min),
                device.set_pag_area_upper(Port::Nf, area_threshold_max),
                device.set_pag_light_threshold(Port::Nf, light_threshold),
                device.set_pag_circle_r_min(Port::Nf, circle_r_min),
                device.set_pag_circle_r_max(Port::Nf, circle_r_max),
                device.set_pag_circle_k_min(Port::Nf, circle_k_min),
                device.set_pag_circle_k_max(Port::Nf, circle_k_max),
            )?;
        }
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

        self.circle_r_min.set(0);
        self.circle_r_max.set(0);
        self.circle_k_min.set(0);
        self.circle_k_max.set(0);
    }

    pub fn load_defaults(&self) {
        self.fps.set(180);
        self.exposure_us.set(2000);
        self.gain.set(2);
        self.area_threshold_min.set(10);
        self.area_threshold_max.set(u16::MAX.into());
        self.light_threshold.set(120);

        // Circle detection defaults (from PAG7665QN datasheet)
        // Q3.5 format: value / 32.0 = float
        self.circle_r_min.set(0x1A); // 26 → 0.8125
        self.circle_r_max.set(0x26); // 38 → 1.1875
        self.circle_k_min.set(0x17); // 23 → 0.71875
        self.circle_k_max.set(0x1D); // 29 → 0.90625
    }
}
