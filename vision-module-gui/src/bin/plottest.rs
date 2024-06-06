use std::{error::Error, time::Instant};

use iui::{
    controls::{Area, AreaDrawParams, AreaHandler, Window, WindowType},
    draw::{plotters::PlottersBackend, Brush, FillMode, Path, SolidBrush},
    UI,
};
use plotters::{
    backend::DrawingBackend,
    chart::ChartBuilder,
    coord::Shift,
    drawing::{DrawingArea, IntoDrawingArea},
    element::{Circle, EmptyElement, Rectangle, Text},
    series::{LineSeries, PointSeries, SurfaceSeries},
    style::{colors::colormaps::VulcanoHSL, Color, HSLColor, IntoFont, BLACK, RED, WHITE},
};

fn main() {
    let ui = UI::init().expect("Couldn't initialize UI library");
    let mut main_win = Window::new(&ui, "le plot", 640, 480, WindowType::NoMenubar);
    vision_module_gui::layout! { &ui,
        let vbox = VerticalBox(padded: false) {
            Stretchy : let area = Area(Box::new(MainCanvas {
                start: Instant::now(),
            }))
        }
    }
    main_win.set_child(&ui, vbox);
    main_win.show(&ui);
    ui.ui_timer(16, {
        let ui = ui.clone();
        let area = area.clone();
        move || {
            area.queue_redraw_all(&ui);
            true
        }
    });

    let mut ev = ui.event_loop();
    ev.run(&ui);
}

struct MainCanvas {
    start: Instant,
}

impl AreaHandler for MainCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
        let ctx = &draw_params.context;
        let bg = Path::new(ctx, FillMode::Winding);
        bg.add_rectangle(
            ctx,
            0.0,
            0.0,
            draw_params.area_width,
            draw_params.area_height,
        );
        bg.end(ctx);
        ctx.fill(
            &bg,
            &Brush::Solid(SolidBrush {
                r: 1.,
                g: 1.,
                b: 1.,
                a: 1.,
            }),
        );

        let mut root = PlottersBackend::new(
            draw_params,
            (
                draw_params.area_width as u32,
                draw_params.area_height as u32,
            ),
        )
        .into_drawing_area();

        // example_chart(root);
        // matshow(&mut root).unwrap();
        gaussian_2d(
            &mut root,
            (self.start.elapsed().as_secs_f64() * 1000.0 / 20.0) as i32 % 157,
        )
        .unwrap();
    }
}

fn example_chart<DB: DrawingBackend>(root: DrawingArea<DB, Shift>) {
    root.fill(&WHITE).unwrap();
    let root = root.margin(50, 50, 50, 50);
    // After this point, we should be able to construct a chart context
    let mut chart = ChartBuilder::on(&root)
        // Set the caption of the chart
        .caption("This is our first plot", ("sans-serif", 30).into_font())
        // Set the size of the label region
        .x_label_area_size(20)
        .y_label_area_size(40)
        // Finally attach a coordinate on the drawing area and make a chart context
        .build_cartesian_2d(0f32..10f32, 0f32..10f32)
        .expect("failed to build chart");

    // Then we can draw a mesh
    chart
        .configure_mesh()
        // We can customize the maximum number of labels allowed for each axis
        .x_labels(5)
        .y_labels(5)
        // We can also change the format of the label text
        .y_label_formatter(&|x| format!("{:.3}", x))
        .draw()
        .expect("failed to draw chart");

    // And we can draw something in the drawing area
    chart
        .draw_series(LineSeries::new(
            vec![(0.0, 0.0), (5.0, 5.0), (8.0, 7.0)],
            &RED,
        ))
        .unwrap();
    // Similarly, we can draw point series
    chart
        .draw_series(PointSeries::of_element(
            vec![(0.0, 0.0), (5.0, 5.0), (8.0, 7.0)],
            5,
            &RED,
            &|c, s, st| {
                return EmptyElement::at(c)    // We want to construct a composed element on-the-fly
            + Circle::new((0,0),s,st.filled()) // At this point, the new pixel coordinate is established
            + Text::new(format!("{:?}", c), (10, 0), ("sans-serif", 10).into_font());
            },
        ))
        .unwrap();
}

fn matshow<DB: DrawingBackend>(
    root: &mut DrawingArea<DB, Shift>,
) -> Result<(), Box<dyn Error + '_>> {
    let mut chart = ChartBuilder::on(&root)
        .caption("Matshow Example", ("sans-serif", 80))
        .margin(5)
        .top_x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(0i32..15i32, 15i32..0i32)?;

    chart
        .configure_mesh()
        .x_labels(15)
        .y_labels(15)
        .max_light_lines(4)
        .x_label_offset(35)
        .y_label_offset(25)
        .disable_x_mesh()
        .disable_y_mesh()
        .label_style(("sans-serif", 20))
        .draw()?;

    let mut matrix = [[0; 15]; 15];

    for i in 0..15 {
        matrix[i][i] = i + 4;
    }

    chart.draw_series(
        matrix
            .iter()
            .zip(0..)
            .flat_map(|(l, y)| l.iter().zip(0..).map(move |(v, x)| (x, y, v)))
            .map(|(x, y, v)| {
                Rectangle::new(
                    [(x, y), (x + 1, y + 1)],
                    HSLColor(
                        240.0 / 360.0 - 240.0 / 360.0 * (*v as f64 / 20.0),
                        0.7,
                        0.1 + 0.4 * *v as f64 / 20.0,
                    )
                    .filled(),
                )
            }),
    )?;
    Ok(())
}

fn gaussian_2d<DB: DrawingBackend>(
    root: &mut DrawingArea<DB, Shift>,
    pitch: i32,
) -> Result<(), Box<dyn Error + '_>> {
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption("2D Gaussian PDF", ("sans-serif", 20))
        .build_cartesian_3d(-3.0..3.0, 0.0..6.0, -3.0..3.0)?;
    chart.with_projection(|mut p| {
        p.pitch = 1.57 - (1.57 - pitch as f64 / 50.0).abs();
        p.scale = 0.7;
        p.into_matrix() // build the projection matrix
    });

    chart
        .configure_axes()
        .light_grid_style(BLACK.mix(0.15))
        .max_light_lines(3)
        .draw()?;

    chart.draw_series(
        SurfaceSeries::xoz(
            (-15..=15).map(|x| x as f64 / 5.0),
            (-15..=15).map(|x| x as f64 / 5.0),
            pdf,
        )
        .style_func(&|&v| (VulcanoHSL::get_color(v / 5.0)).into()),
    )?;

    root.present()?;

    fn pdf(x: f64, y: f64) -> f64 {
        const SDX: f64 = 0.1;
        const SDY: f64 = 0.1;
        const A: f64 = 5.0;
        let x = x / 10.0;
        let y = y / 10.0;
        A * (-x * x / 2.0 / SDX / SDX - y * y / 2.0 / SDY / SDY).exp()
    }

    Ok(())
}
