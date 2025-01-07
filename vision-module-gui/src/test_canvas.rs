use crate::custom_shapes::{draw_crosshair, draw_grid, draw_text};
use crate::mot_runner::MotRunner;
use iui::controls::{Area, AreaDrawParams, AreaHandler, AreaKeyEvent, Modifiers, Window};
use iui::draw::{Brush, FillMode, Path, SolidBrush, StrokeParams};
use iui::UI;
use nalgebra::{Isometry3, Point2, Translation3, Vector2, Vector3};
use parking_lot::Mutex;
use std::sync::Arc;
use tracing::debug;

pub struct TestCanvas {
    pub ctx: UI,
    pub window: Window,
    pub on_closing: Box<dyn FnMut(&mut Window)>,
    pub runner: Arc<Mutex<MotRunner>>,
    pub last_draw_width: Option<f64>,
    pub last_draw_height: Option<f64>,
}

impl AreaHandler for TestCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
        self.last_draw_width = Some(draw_params.area_width);
        self.last_draw_height = Some(draw_params.area_height);
        let ctx = &draw_params.context;

        let background = Path::new(ctx, FillMode::Winding);
        background.add_rectangle(ctx, 0., 0., draw_params.area_width, draw_params.area_height);
        background.end(ctx);

        ctx.fill(
            &background,
            &Brush::Solid(SolidBrush {
                r: 0.5,
                g: 0.5,
                b: 0.5,
                a: 1.,
            }),
        );

        let fv_ch_path = Path::new(ctx, FillMode::Winding);
        let runner = self.runner.lock();
        let state = &runner.state;
        {
            let aimpoint = state.fv_aimpoint;
            draw_crosshair(
                &ctx,
                &fv_ch_path,
                aimpoint.x as f64 * draw_params.area_width,
                aimpoint.y as f64 * draw_params.area_height,
                30.,
            );
        }
        fv_ch_path.end(ctx);
        draw_text(
            &ctx,
            20.0,
            20.0,
            &format!("distance = {:.4}", runner.state.distance),
        );
        draw_text(
            &ctx,
            20.0,
            40.0,
            &format!("screen_id = {}", runner.state.fv_state.screen_id),
        );

        let grid_path = Path::new(ctx, FillMode::Winding);

        // todo lol... i know
        let transform = ats_cv::get_perspective_transform(
            Point2::new(draw_params.area_width / 2.0, draw_params.area_height as f64), // bottom
            Point2::new(0.0, draw_params.area_height / 2.0),                           // left
            Point2::new(draw_params.area_width / 2.0, 0.0),                            // top
            Point2::new(draw_params.area_width as f64, draw_params.area_height / 2.0), // right
            Point2::new(0.5, 1.),                                                      // bottom
            Point2::new(0., 0.5),                                                      // left
            Point2::new(0.5, 0.),                                                      // top
            Point2::new(1., 0.5),                                                      // right
        );
        if let Some(transform) = transform.and_then(|t| t.try_inverse()) {
            draw_grid(ctx, &grid_path, 10, 10, transform);
        }
        grid_path.end(ctx);

        let center_target_path = Path::new(ctx, FillMode::Winding);
        draw_crosshair(
            &ctx,
            &center_target_path,
            draw_params.area_width / 2.0,
            draw_params.area_height / 2.0,
            25.,
        );
        center_target_path.new_figure_with_arc(
            &ctx,
            draw_params.area_width / 2.0,
            draw_params.area_height / 2.0,
            25.,
            0.,
            2. * std::f64::consts::PI,
            false,
        );
        center_target_path.end(ctx);

        let stroke = StrokeParams {
            cap: 0,  // Bevel
            join: 0, // Flat
            thickness: 10.,
            miter_limit: 0.,
            dashes: vec![],
            dash_phase: 0.,
        };

        let brush = Brush::Solid(SolidBrush {
            r: 0.,
            g: 1.,
            b: 0.,
            a: 1.,
        });

        ctx.stroke(&fv_ch_path, &brush, &stroke);

        let _stroke = StrokeParams {
            cap: 0,  // Bevel
            join: 0, // Flat
            thickness: 5.,
            miter_limit: 0.,
            dashes: vec![],
            dash_phase: 0.,
        };

        // Grid
        let brush = Brush::Solid(SolidBrush {
            r: 0.5,
            g: 0.,
            b: 0.,
            a: 1.,
        });
        let stroke = StrokeParams {
            cap: 0,  // Bevel
            join: 0, // Flat
            thickness: 1.,
            miter_limit: 0.,
            dashes: vec![],
            dash_phase: 0.,
        };
        ctx.stroke(&grid_path, &brush, &stroke);

        // Center target
        let brush = Brush::Solid(SolidBrush {
            r: 0.,
            g: 0.,
            b: 1.,
            a: 1.,
        });
        let stroke = StrokeParams {
            cap: 0,  // Bevel
            join: 0, // Flat
            thickness: 1.,
            miter_limit: 0.,
            dashes: vec![],
            dash_phase: 0.,
        };
        ctx.stroke(&center_target_path, &brush, &stroke);
    }

    fn key_event(&mut self, _area: &Area, area_key_event: &AreaKeyEvent) -> bool {
        debug!("{:?}", area_key_event);
        if area_key_event.up {
            match area_key_event.ext_key {
                _ => match area_key_event.key {
                    b'z' => {
                        let t = Translation3::new(0., 0.0381, 0.);
                        let quat = {
                            let screen_calibrations = self.runner.lock().screen_calibrations.clone();
                            let fv_state = &self.runner.lock().state.fv_state;
                            ats_cv::helpers::calculate_zero_offset_quat(t, Point2::new(0.5, 0.5), &screen_calibrations, fv_state)
                        };
                        if let Some(quat) = quat {
                            self.runner.lock().state.fv_zero_offset = Isometry3::from_parts(
                                t,
                                quat,
                            );
                        } else {
                            tracing::warn!("Failed to calculate zero offset quat");
                        }
                    },
                    _ => (),
                }
            }
            return true;
        }
        match area_key_event.ext_key as _ {
            ui_sys::uiExtKeyEscape => (self.on_closing)(&mut self.window),
            _ => match area_key_event.key {
                b'q' => (self.on_closing)(&mut self.window),
                // Backspace
                8 => self.runner.lock().state.fv_zero_offset = Default::default(),
                _ => (),
            },
        }
        true
    }
}
