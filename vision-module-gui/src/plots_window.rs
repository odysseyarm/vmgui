use std::{f64::consts::PI, ops::Range};

use ats_cv::telemetry::Series;
use iui::{
    controls::{Area, AreaDrawParams, AreaHandler, Window, WindowType},
    draw::plotters::PlottersBackend,
    UI,
};
use nalgebra::Vector3;
use plotters::{
    backend::DrawingBackend,
    chart::{ChartBuilder, LabelAreaPosition},
    coord::Shift,
    drawing::{DrawingArea, IntoDrawingArea},
    series::LineSeries,
    style::{BLUE, GREEN, RED, WHITE},
};

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
        let root = PlottersBackend::new(
            draw_params,
            (
                draw_params.area_width as u32,
                draw_params.area_height as u32,
            ),
        )
        .into_drawing_area();
        root.fill(&WHITE).unwrap();
        let subplots = root.split_evenly((6, 2));

        gyro_chart(&subplots[0]);
        accel_chart(&subplots[1]);
        eskf_mismatch_chart(&subplots[2]);
        apparent_accel_chart(&subplots[3]);
        accel_bias_chart(&subplots[4]);
        local_gravity_chart(&subplots[5]);
        velocity_uncertainty_chart(&subplots[6]);
        velocity_chart(&subplots[7]);
        // pnp_position_chart(&subplots[8]);
        // pnp_orientation_chart(&subplots[9]);
        position_uncertainty_chart(&subplots[8]);
        position_chart(&subplots[9]);
        accel_bias_uncertainty_chart(&subplots[10]);
        orientation_uncertainty_chart(&subplots[11]);
    }
}

fn gyro_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::imu_data();
    vec3_f64_chart(
        area,
        "IMU Angular Velocity",
        series.values.lock().unwrap().iter().map(|p| p.1.1),
        series.size,
        -5.0..5.0,
    );
}

fn accel_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::imu_data();
    vec3_f64_chart(
        area,
        "IMU Acceleration",
        series.values.lock().unwrap().iter().map(|p| p.1.0),
        series.size,
        -10.0..10.0,
    );
}

fn pnp_position_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::pnp_solutions();
    vec3_f64_chart(
        area,
        "PnP Position",
        series.values.lock().unwrap().iter().map(|p| p.1.inverse_transform_point(&[0., 0., 0.].into()).coords),
        series.size,
        -5.0..1.0,
    );
}

fn pnp_orientation_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::pnp_solutions();
    vec3_f64_chart(
        area,
        "PnP Orientation",
        series.values.lock().unwrap().iter().map(|p| {
            let (x, y, z) = p.1.rotation.inverse().euler_angles();
            Vector3::new(x, y, z)
        }),
        series.size,
        -3.15..3.15,
    );
}

fn rot_bias_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    vec3_f32_chart(
        area,
        "ESKF Rot. Bias",
        ats_cv::telemetry::eskf_rot_bias(),
        -1.0..1.0,
    );
}

fn accel_bias_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    vec3_f32_chart(
        area,
        "ESKF Accel. Bias",
        ats_cv::telemetry::eskf_accel_bias(),
        -5.0..5.0,
    );
}

fn gravity_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    vec3_f32_chart(
        area,
        "ESKF Gravity",
        ats_cv::telemetry::eskf_gravity(),
        -10.0..10.0,
    );
}

fn local_gravity_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    vec3_f32_chart(
        area,
        "ESKF Local Gravity",
        ats_cv::telemetry::eskf_local_gravity(),
        -10.0..10.0,
    );
}

fn apparent_accel_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    vec3_f32_chart(
        area,
        "ESKF Apparent Accel.",
        ats_cv::telemetry::eskf_apparent_accel(),
        -10.0..10.0,
    );
}

fn velocity_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    vec3_f32_chart(
        area,
        "ESKF Velocity",
        ats_cv::telemetry::eskf_velocity(),
        -1.0..1.0,
    );
}

fn accel_bias_uncertainty_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    vec3_f32_chart(
        area,
        "ESKF Accel. Bias Uncertainty",
        ats_cv::telemetry::eskf_accel_bias_uncertainty(),
        0.0..1.0,
    );
}

fn rot_bias_uncertainty_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    vec3_f32_chart(
        area,
        "ESKF Rot. Bias Uncertainty",
        ats_cv::telemetry::eskf_rot_bias_uncertainty(),
        0.0..1.0,
    );
}

fn velocity_uncertainty_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    vec3_f32_chart(
        area,
        "ESKF Velocity Uncertainty",
        ats_cv::telemetry::eskf_velocity_uncertainty(),
        0.0..1.0,
    );
}

fn orientation_uncertainty_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::eskf_orientation_uncertainty();
    vec3_f64_chart(
        area,
        "ESKF Orientation Uncertainty",
        series
            .values
            .lock()
            .unwrap()
            .iter()
            .map(|p| p.1.cast() * 180.0 / PI),
        series.size,
        0.0..10.0,
    );
}

fn eskf_mismatch_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::eskf_mismatch_count();
    scalar_f64_chart(
        area,
        "ESKF/P3P Mismatch Count",
        series
            .values
            .lock()
            .unwrap()
            .iter()
            .map(|p| f64::from(p.1)),
        series.size,
        0.0..10.0,
    );
}

fn position_uncertainty_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    vec3_f32_chart(
        area,
        "ESKF Position Uncertainty",
        ats_cv::telemetry::eskf_position_uncertainty(),
        0.0..1.0,
    );
}

fn position_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    vec3_f32_chart(
        area,
        "ESKF Position",
        ats_cv::telemetry::eskf_position(),
        0.0..5.0,
    );
}

fn vec3_f32_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>, caption: &str, series: &Series<Vector3<f32>>, default_range: Range<f32>) {
    let data = series.values.lock().unwrap();
    let min = data
        .iter()
        .flat_map(|p| p.1.as_slice())
        .copied()
        .min_by(f32::total_cmp)
        .unwrap_or(default_range.start)
        .min(default_range.start);
    let max = data
        .iter().
        flat_map(|p| p.1.as_slice())
        .copied()
        .max_by(f32::total_cmp)
        .unwrap_or(default_range.end)
        .max(default_range.end);

    let mut chart = ChartBuilder::on(area)
        .caption(caption, ("sans-serif", 12))
        .margin(20)
        // .set_left_and_bottom_label_area_size(30)
        .set_label_area_size(LabelAreaPosition::Left, 30)
        .build_cartesian_2d(0..series.size, min..max)
        .unwrap();
    chart
        .configure_mesh()
        .disable_x_mesh()
        .max_light_lines(1)
        .draw()
        .unwrap();

    let x = data.iter().map(|p| p.1.x).enumerate();
    let y = data.iter().map(|p| p.1.y).enumerate();
    let z = data.iter().map(|p| p.1.z).enumerate();
    chart.draw_series(LineSeries::new(x, &RED)).unwrap();
    chart.draw_series(LineSeries::new(y, &GREEN)).unwrap();
    chart.draw_series(LineSeries::new(z, &BLUE)).unwrap();
}

fn vec3_f64_chart<DB: DrawingBackend>(
    area: &DrawingArea<DB, Shift>,
    caption: &str,
    data: impl IntoIterator<Item = Vector3<f64>> + Clone,
    series_size: usize,
    default_range: Range<f64>,
) {
    let min = data
        .clone()
        .into_iter()
        .flat_map(|p| [p.x, p.y, p.z])
        .min_by(f64::total_cmp)
        .unwrap_or(default_range.start)
        .min(default_range.start);
    let max = data
        .clone()
        .into_iter()
        .flat_map(|p| [p.x, p.y, p.z])
        .max_by(f64::total_cmp)
        .unwrap_or(default_range.end)
        .max(default_range.end);

    let mut chart = ChartBuilder::on(area)
        .caption(caption, ("sans-serif", 12))
        .margin(20)
        // .set_left_and_bottom_label_area_size(30)
        .set_label_area_size(LabelAreaPosition::Left, 30)
        .build_cartesian_2d(0..series_size, min..max)
        .unwrap();
    chart
        .configure_mesh()
        .disable_x_mesh()
        .max_light_lines(1)
        .draw()
        .unwrap();

    let x = data.clone().into_iter().map(|p| p.x).enumerate();
    let y = data.clone().into_iter().map(|p| p.y).enumerate();
    let z = data.clone().into_iter().map(|p| p.z).enumerate();
    chart.draw_series(LineSeries::new(x, &RED)).unwrap();
    chart.draw_series(LineSeries::new(y, &GREEN)).unwrap();
    chart.draw_series(LineSeries::new(z, &BLUE)).unwrap();
}

fn scalar_f64_chart<DB: DrawingBackend>(
    area: &DrawingArea<DB, Shift>,
    caption: &str,
    data: impl IntoIterator<Item = f64> + Clone,
    series_size: usize,
    default_range: Range<f64>,
) {
    let min = data
        .clone()
        .into_iter()
        .min_by(f64::total_cmp)
        .unwrap_or(default_range.start)
        .min(default_range.start);
    let max = data
        .clone()
        .into_iter()
        .max_by(f64::total_cmp)
        .unwrap_or(default_range.end)
        .max(default_range.end);

    let mut chart = ChartBuilder::on(area)
        .caption(caption, ("sans-serif", 12))
        .margin(20)
        // .set_left_and_bottom_label_area_size(30)
        .set_label_area_size(LabelAreaPosition::Left, 30)
        .build_cartesian_2d(0..series_size, min..max)
        .unwrap();
    chart
        .configure_mesh()
        .disable_x_mesh()
        .max_light_lines(1)
        .draw()
        .unwrap();

    chart.draw_series(LineSeries::new(data.into_iter().enumerate(), &RED)).unwrap();
}
