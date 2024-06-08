use std::error::Error;

use iui::{controls::{Area, AreaDrawParams, AreaHandler, Window, WindowType}, draw::plotters::PlottersBackend, UI};
use plotters::{backend::DrawingBackend, chart::ChartBuilder, coord::{CoordTranslate, Shift}, drawing::{DrawingArea, IntoDrawingArea}, element::Rectangle, series::LineSeries, style::{Color, HSLColor, BLUE, GREEN, RED, WHITE}};

use crate::CloneButShorter;

pub fn plots_window(ui: &UI) -> Window {
    let mut window = Window::new(ui, "le plot", 640, 480, WindowType::NoMenubar);
    crate::layout! { ui,
        let vbox = VerticalBox(padded: false) {
            Stretchy : let area = Area(Box::new(MainCanvas))
        }
    }
    window.set_child(&ui, vbox);

    window.on_closing(&ui, {
        let ui = ui.c();
        move |win| {
            win.hide(&ui);
        }
    });

    ui.ui_timer(16, {
        let window = window.c();
        let ui = ui.c();
        let area = area.c();
        move || {
            if window.visible(&ui) {
                area.queue_redraw_all(&ui);
            }
            true
        }
    });
    window.visible(ui);
    window
}

struct MainCanvas;

impl AreaHandler for MainCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
        let mut root = PlottersBackend::new(
            draw_params,
            (
                draw_params.area_width as u32,
                draw_params.area_height as u32,
            ),
        )
        .into_drawing_area();
        root.fill(&WHITE);
        let subplots = root.split_evenly((1, 2));

        gyro_chart(&subplots[0]);
        accel_chart(&subplots[1]);
    }
}

fn gyro_chart<DB: DrawingBackend>(
    area: &DrawingArea<DB, Shift>,
) {
    let series = ats_cv::telemetry::imu_data();
    let mut chart = ChartBuilder::on(area)
        .caption("Angular Velocity", ("sans-serif", 12))
        .build_cartesian_2d(0..series.size, -20.0..20.0)
        .unwrap();

    let data = series.values.lock().unwrap();

    let x = data.iter().map(|p| p.1.1.x).enumerate();
    let y = data.iter().map(|p| p.1.1.y).enumerate();
    let z = data.iter().map(|p| p.1.1.z).enumerate();
    chart.draw_series(LineSeries::new(x, &RED)).unwrap();
    chart.draw_series(LineSeries::new(y, &GREEN)).unwrap();
    chart.draw_series(LineSeries::new(z, &BLUE)).unwrap();
}

fn accel_chart<DB: DrawingBackend>(
    area: &DrawingArea<DB, Shift>,
) {
    let series = ats_cv::telemetry::imu_data();
    let mut chart = ChartBuilder::on(area)
        .caption("Acceleration", ("sans-serif", 12))
        .build_cartesian_2d(0..series.size, -30.0..30.0)
        .unwrap();

    let data = series.values.lock().unwrap();

    let x = data.iter().map(|p| p.1.0.x).enumerate();
    let y = data.iter().map(|p| p.1.0.y).enumerate();
    let z = data.iter().map(|p| p.1.0.z).enumerate();
    chart.draw_series(LineSeries::new(x, &RED)).unwrap();
    chart.draw_series(LineSeries::new(y, &GREEN)).unwrap();
    chart.draw_series(LineSeries::new(z, &BLUE)).unwrap();
}
