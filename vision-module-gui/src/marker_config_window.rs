extern crate directories;

mod calibrate;

use std::fs;
use std::fs::OpenOptions;
use std::io::Read;
use std::sync::Arc;
use directories::ProjectDirs;
use nalgebra::Matrix2x4;
use tracing::error;

use ats_usb::{packet::MarkerPattern};
use crate::CloneButShorter;
use anyhow::Result;
use iui::{
    controls::Form,
    prelude::{Window, WindowType},
    UI,
};
use leptos_reactive::{create_effect, create_rw_signal, Memo, RwSignal, SignalGet, SignalGetUntracked, SignalSet};
use serde::{Deserialize, Serialize};
use parking_lot::Mutex;
use crate::mot_runner::MotRunner;

pub fn marker_config_window(
    ui: &UI,
    marker_offset_calibrating: RwSignal<bool>,
    // marker_pattern_memo: Memo<MarkerPattern>,
    mot_runner: Arc<Mutex<MotRunner>>,
) -> Window {
    let ui_ctx = ui.async_context();
    let mut config_win = Window::new(&ui, "Marker Config", 10, 10, WindowType::NoMenubar);
    config_win.on_closing(&ui, {
        let ui = ui.c();
        move |win: &mut Window| {
            win.hide(&ui);
        }
    });

    crate::layout! { &ui,
        let vbox = VerticalBox(padded: true) {
            Compact : let tab_group = TabGroup() {} // sensor settings go in here
            Compact : let buttons_hbox = HorizontalBox(padded: true) {
                Compact : let apply_button = Button("Apply")
                Compact : let save_button = Button("Save")
                Compact : let reload_button = Button("Reload")
                Compact : let load_defaults_button = Button("Load defaults")
            }
        }
    }
    let (marker_settings_form, mut marker_settings) = MarkersSettingsForm::new(&ui);
    tab_group.append(&ui, "Marker Settings", marker_settings_form.c());
    tab_group.append(
        &ui,
        "Calibrate",
        calibrate::create(ui, marker_offset_calibrating, mot_runner.c(), marker_settings, config_win.c()),
    );
    tab_group.set_margined(&ui, 0, true);
    tab_group.set_margined(&ui, 1, true);

    config_win.set_child(&ui, vbox);

    let apply_button_on_click = {
        let mot_runner = mot_runner.c();
        move || {
            let mot_runner = mot_runner.c();
            let runner = &mut mot_runner.lock();
            marker_settings.apply(&mut runner.markers_settings);
        }
    };
    apply_button.on_clicked(&ui, {
        let apply_button_on_click = apply_button_on_click.c();
        move |_| apply_button_on_click()
    });
    save_button.on_clicked(&ui, {
        move |_save_button| {
        }
    });
    load_defaults_button.on_clicked(&ui, move |_| {
        // marker_settings.load_defaults(marker_pattern_memo.get_untracked());
        marker_settings.load_defaults(MarkerPattern::Rectangle);
    });
    // When the applied marker pattern changes, reload the defaults
    // create_effect(move |_| {
    //     marker_settings.load_defaults(marker_pattern_memo.get());
    //     apply_button_on_click();
    // });

    reload_button.on_clicked(&ui, {
        move |_| {
            ui_ctx.spawn(async move {
                let _ = tokio::join!(
                    marker_settings.load_from_file(),
                );
            });
        }
    });

    config_win
}

// -2047,-2047 represents the top left corner of the view
#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
pub struct Position {
    pub x: i32,
    pub y: i32,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
pub struct Marker {
    pub position: Position,
}

// the markers are positioned in a cross pattern around the center of the view
#[derive(Serialize, Deserialize, Debug)]
pub struct View {
    pub marker_top: Marker,
    pub marker_right: Marker,
    pub marker_bottom: Marker,
    pub marker_left: Marker,
}

// todo support multiple views
#[derive(Copy, Clone)]
struct MarkersSettingsForm {
    marker_top: (RwSignal<i32>, RwSignal<i32>),
    marker_right: (RwSignal<i32>, RwSignal<i32>),
    marker_bottom: (RwSignal<i32>, RwSignal<i32>),
    marker_left: (RwSignal<i32>, RwSignal<i32>),
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MarkersSettings {
    pub views: Vec<View>,
}

impl MarkersSettingsForm {
    fn new(ui: &UI) -> (Form, Self) {
        let marker_top = (create_rw_signal(0), create_rw_signal(-2047));
        let marker_right = (create_rw_signal(2047), create_rw_signal(0));
        let marker_bottom = (create_rw_signal(0), create_rw_signal(2047));
        let marker_left = (create_rw_signal(-2047), create_rw_signal(0));

        crate::layout! { &ui,
            let form = Form(padded: true) {
                (Compact, "Offset (top.x)") : let x = Spinbox(signal: marker_top.0)
                (Compact, "Offset (top.y)") : let x = Spinbox(signal: marker_top.1)
                (Compact, "Offset (right.x)") : let x = Spinbox(signal: marker_right.0)
                (Compact, "Offset (right.y)") : let x = Spinbox(signal: marker_right.1)
                (Compact, "Offset (bottom.x)") : let x = Spinbox(signal: marker_bottom.0)
                (Compact, "Offset (bottom.y)") : let x = Spinbox(signal: marker_bottom.1)
                (Compact, "Offset (left.x)") : let x = Spinbox(signal: marker_left.0)
                (Compact, "Offset (left.y)") : let x = Spinbox(signal: marker_left.1)
            }
        }
        (
            form,
            Self {
                marker_top,
                marker_right,
                marker_bottom,
                marker_left,
            },
        )
    }

    async fn load_from_file(&mut self) -> Result<()> {
        if let Some(proj_dirs) = ProjectDirs::from("com", "odysseyarm",  "odyssey") {
            fs::create_dir_all(proj_dirs.config_dir()).expect("Unable to create directories");
            let views_config_path = proj_dirs.config_dir().join("markers.toml");
            if views_config_path.exists() {
                let mut data = String::new();
                OpenOptions::new()
                    .read(true)
                    .open(views_config_path)
                    .unwrap()
                    .read_to_string(&mut data)
                    .expect("Unable to read to string");
                match toml::from_str::<MarkersSettings>(data.as_str()) {
                    Ok(markers_settings) => {
                        // todo don't only use the first one
                        if markers_settings.views.len() > 0 {
                            let view = &markers_settings.views[0];
                            self.marker_top.0.set(view.marker_top.position.x);
                            self.marker_top.1.set(view.marker_top.position.y);
                            self.marker_right.0.set(view.marker_right.position.x);
                            self.marker_right.1.set(view.marker_right.position.y);
                            self.marker_bottom.0.set(view.marker_bottom.position.x);
                            self.marker_bottom.1.set(view.marker_bottom.position.y);
                            self.marker_left.0.set(view.marker_left.position.x);
                            self.marker_left.1.set(view.marker_left.position.y);
                        }
                    },
                    Err(e) => error!("{}", e),
                };
            } else {
                self.marker_top.0.set(0);
                self.marker_top.1.set(-2047);
                self.marker_right.0.set(2047);
                self.marker_right.1.set(0);
                self.marker_bottom.0.set(0);
                self.marker_bottom.1.set(2047);
                self.marker_left.0.set(-2047);
                self.marker_left.1.set(0);
            }
        }
        Ok(())
    }

    fn _validate(&self, _errors: &mut Vec<String>) {
        macro_rules! _validators {
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
    }

    /// Make sure to call `validate()` before calling this method.
    fn apply(&self, markers_settings: &mut MarkersSettings) {
        markers_settings.views[0].marker_top.position.x = self.marker_top.0.get_untracked();
        markers_settings.views[0].marker_top.position.y = self.marker_top.1.get_untracked();
        markers_settings.views[0].marker_bottom.position.x = self.marker_bottom.0.get_untracked();
        markers_settings.views[0].marker_bottom.position.y = self.marker_bottom.1.get_untracked();
        markers_settings.views[0].marker_left.position.x = self.marker_left.0.get_untracked();
        markers_settings.views[0].marker_left.position.y = self.marker_left.1.get_untracked();
        markers_settings.views[0].marker_right.position.x = self.marker_right.0.get_untracked();
        markers_settings.views[0].marker_right.position.y = self.marker_right.1.get_untracked();
    }

    fn load_defaults(&self, marker_pattern: MarkerPattern) {
        let [bottom, left, top, right] = marker_pattern.marker_positions();
        let m = Matrix2x4::from_columns(&[
            bottom.coords,
            left.coords,
            top.coords,
            right.coords,
        ]);
        let m: Matrix2x4<f64> = m.add_scalar(-0.5) * 4094.0;
        let m = m.map(|v| v.round() as i32);
        self.marker_bottom.0.set(m.column(0).x);
        self.marker_bottom.1.set(m.column(0).y);
        self.marker_left.0.set(m.column(1).x);
        self.marker_left.1.set(m.column(1).y);
        self.marker_top.0.set(m.column(2).x);
        self.marker_top.1.set(m.column(2).y);
        self.marker_right.0.set(m.column(3).x);
        self.marker_right.1.set(m.column(3).y);
    }
}

impl Default for MarkersSettings {
    fn default() -> Self {
        Self {
            views: vec![View {
                marker_top: Marker { position: Position { x: 0, y: -2047 } },
                marker_right: Marker { position: Position { x: 2047, y: 0 } },
                marker_bottom: Marker { position: Position { x: 0, y: 2047 } },
                marker_left: Marker { position: Position { x: -2047, y: 0 } },
            }]
        }
    }
}
