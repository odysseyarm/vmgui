//! Marker offset calibration tab

use iui::{controls::VerticalBox, UI};
use leptos_reactive::{create_effect, Effect, RwSignal, SignalGet, SignalGetUntracked, SignalSet, SignalUpdate, SignalWith};
use nalgebra::Point2;

use crate::CloneButShorter;

pub fn create(ui: &UI, calibrating: RwSignal<bool>) -> VerticalBox {
    let samples = RwSignal::<Vec<Sample>>::new(vec![]);
    crate::layout! { &ui,
        let vbox = VerticalBox(padded: true) {
            Compact : let hbox = HorizontalBox(padded: true) {
                Compact : let start_button = Button(
                    move || if calibrating.get() {
                        "Cancel calibration"
                    } else {
                        "Start calibration"
                    }
                )
            }
            Compact : let hbox = HorizontalBox(padded: true) {
                Compact : let text = Label(move || samples.with(|s| format!("Samples: {}/4", s.len())))
            }
            Compact : let hbox = HorizontalBox(padded: true) {
                Compact : let collect_button = Button("Collect", enabled: calibrating)
            }
        }
    }

    // Enable/disable the text
    create_effect({
        let (ui, text) = (ui.c(), text.c());
        move |_| {
            let mut text = text.c();
            if calibrating.get() {
                text.enable(&ui);
            } else {
                text.disable(&ui);
            }
        }
    });

    start_button.on_clicked(ui, {
        move |_| {
            if calibrating.get_untracked() {
                // Cancelled
                samples.update(Vec::clear);
                calibrating.set(false);
            } else {
                calibrating.set(true);
            }
        }
    });
    vbox
}

struct Sample {
    reported_aimpoint: Point2<f64>,
    true_aimpoint: Point2<f64>,
}
