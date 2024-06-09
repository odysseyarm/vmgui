use iui::{
    controls::{Area, AreaDrawParams, AreaHandler, Window, WindowType},
    draw::plotters::PlottersBackend,
    UI,
};
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
        let subplots = root.split_evenly((3, 2));

        gyro_chart(&subplots[0]);
        accel_chart(&subplots[1]);
        pnp_position_chart(&subplots[2]);
        pnp_orientation_chart(&subplots[3]);
        accel_bias_chart(&subplots[4]);
        rot_bias_chart(&subplots[5]);
    }
}

fn gyro_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::imu_data();
    let mut chart = ChartBuilder::on(area)
        .caption("IMU Angular Velocity", ("sans-serif", 12))
        .margin(20)
        .set_left_and_bottom_label_area_size(30)
        .build_cartesian_2d(0..series.size, -35.0..35.0)
        .unwrap();
    chart
        .configure_mesh()
        .disable_x_mesh()
        .max_light_lines(1)
        .draw()
        .unwrap();

    let data = series.values.lock().unwrap();

    let x = data.iter().map(|p| p.1 .1.x).enumerate();
    let y = data.iter().map(|p| p.1 .1.y).enumerate();
    let z = data.iter().map(|p| p.1 .1.z).enumerate();
    chart.draw_series(LineSeries::new(x, &RED)).unwrap();
    chart.draw_series(LineSeries::new(y, &GREEN)).unwrap();
    chart.draw_series(LineSeries::new(z, &BLUE)).unwrap();
}

fn accel_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::imu_data();
    let mut chart = ChartBuilder::on(area)
        .caption("IMU Acceleration", ("sans-serif", 12))
        .margin(20)
        .set_left_and_bottom_label_area_size(30)
        .build_cartesian_2d(0..series.size, -30.0..30.0)
        .unwrap();
    chart
        .configure_mesh()
        .disable_x_mesh()
        .max_light_lines(1)
        .draw()
        .unwrap();

    let data = series.values.lock().unwrap();

    let x = data.iter().map(|p| p.1 .0.x).enumerate();
    let y = data.iter().map(|p| p.1 .0.y).enumerate();
    let z = data.iter().map(|p| p.1 .0.z).enumerate();
    chart.draw_series(LineSeries::new(x, &RED)).unwrap();
    chart.draw_series(LineSeries::new(y, &GREEN)).unwrap();
    chart.draw_series(LineSeries::new(z, &BLUE)).unwrap();
}

fn pnp_position_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::pnp_solutions();
    let mut chart = ChartBuilder::on(area)
        .caption("PnP Position", ("sans-serif", 12))
        .margin(20)
        .set_left_and_bottom_label_area_size(30)
        .build_cartesian_2d(0..series.size, -10.0..10.0)
        .unwrap();
    chart
        .configure_mesh()
        .disable_x_mesh()
        .max_light_lines(1)
        .draw()
        .unwrap();

    let data = series.values.lock().unwrap();

    let x = data.iter().map(|p| p.1.translation.x).enumerate();
    let y = data.iter().map(|p| p.1.translation.y).enumerate();
    let z = data.iter().map(|p| p.1.translation.z).enumerate();
    chart.draw_series(LineSeries::new(x, &RED)).unwrap();
    chart.draw_series(LineSeries::new(y, &GREEN)).unwrap();
    chart.draw_series(LineSeries::new(z, &BLUE)).unwrap();
}

fn pnp_orientation_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::pnp_solutions();
    let mut chart = ChartBuilder::on(area)
        .caption("PnP Orientation", ("sans-serif", 12))
        .margin(20)
        .set_left_and_bottom_label_area_size(30)
        .build_cartesian_2d(0..series.size, -3.15..3.15)
        .unwrap();
    chart
        .configure_mesh()
        .disable_x_mesh()
        .max_light_lines(1)
        .draw()
        .unwrap();

    let data = series.values.lock().unwrap();

    let x = data
        .iter()
        .map(|p| p.1.rotation.euler_angles().0)
        .enumerate();
    let y = data
        .iter()
        .map(|p| p.1.rotation.euler_angles().1)
        .enumerate();
    let z = data
        .iter()
        .map(|p| p.1.rotation.euler_angles().2)
        .enumerate();
    chart.draw_series(LineSeries::new(x, &RED)).unwrap();
    chart.draw_series(LineSeries::new(y, &GREEN)).unwrap();
    chart.draw_series(LineSeries::new(z, &BLUE)).unwrap();
}

fn rot_bias_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::eskf_rot_bias();
    let mut chart = ChartBuilder::on(area)
        .caption("ESKF Rot. Bias", ("sans-serif", 12))
        .margin(20)
        .set_left_and_bottom_label_area_size(30)
        .build_cartesian_2d(0..series.size, -1.0f32..1.0f32)
        .unwrap();
    chart
        .configure_mesh()
        .disable_x_mesh()
        .max_light_lines(1)
        .draw()
        .unwrap();

    let data = series.values.lock().unwrap();

    let x = data.iter().map(|p| p.1.x).enumerate();
    let y = data.iter().map(|p| p.1.y).enumerate();
    let z = data.iter().map(|p| p.1.z).enumerate();
    chart.draw_series(LineSeries::new(x, &RED)).unwrap();
    chart.draw_series(LineSeries::new(y, &GREEN)).unwrap();
    chart.draw_series(LineSeries::new(z, &BLUE)).unwrap();
}

fn accel_bias_chart<DB: DrawingBackend>(area: &DrawingArea<DB, Shift>) {
    let series = ats_cv::telemetry::eskf_accel_bias();
    let mut chart = ChartBuilder::on(area)
        .caption("ESKF Accel. Bias", ("sans-serif", 12))
        .margin(20)
        .set_left_and_bottom_label_area_size(30)
        .build_cartesian_2d(0..series.size, -5.0f32..5.0f32)
        .unwrap();
    chart
        .configure_mesh()
        .disable_x_mesh()
        .max_light_lines(1)
        .draw()
        .unwrap();

    let data = series.values.lock().unwrap();

    let x = data.iter().map(|p| p.1.x).enumerate();
    let y = data.iter().map(|p| p.1.y).enumerate();
    let z = data.iter().map(|p| p.1.z).enumerate();
    chart.draw_series(LineSeries::new(x, &RED)).unwrap();
    chart.draw_series(LineSeries::new(y, &GREEN)).unwrap();
    chart.draw_series(LineSeries::new(z, &BLUE)).unwrap();
}
