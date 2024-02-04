use std::sync::Arc;
use tokio::sync::Mutex;
use iui::controls::{Area, AreaDrawParams, AreaHandler, AreaKeyEvent, Window};
use iui::draw::{Brush, FillMode, Path, SolidBrush, StrokeParams};
use iui::UI;
use crate::custom_shapes::draw_crosshair;
use crate::mot_runner::MotRunner;
use crate::MotState;

pub struct TestCanvas {
    pub ctx: UI,
    pub window: Window,
    pub on_closing: Box<dyn FnMut(&mut Window)>,
    pub state: Arc<Mutex<MotRunner>>,
    pub offset_x: f64,
    pub offset_y: f64,
}

impl AreaHandler for TestCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
        let ctx = &draw_params.context;

        // todo foveated is not ready
        // let fv_ch_path = Path::new(ctx, FillMode::Winding);
        let nf_ch_path = Path::new(ctx, FillMode::Winding);
        let wf_ch_path = Path::new(ctx, FillMode::Winding);
        let runner = self.state.blocking_lock();
        let state = &runner.state;
        // if let Some(aim_point) = state.fv_aim_point {
        //     draw_crosshair(&ctx, &fv_ch_path, aim_point.x*draw_params.area_width, aim_point.y*draw_params.area_height, 30.);
        // }
        // fv_ch_path.end(ctx);
        ctx.draw_text(20.0, 20.0, format!("offset = ({:.4}, {:.4})", self.offset_x, self.offset_y).as_str());
        if let Some(aim_point) = state.nf_aim_point {
            let x = aim_point.x*draw_params.area_width + self.offset_x;
            let y = aim_point.y*draw_params.area_height + self.offset_y;
            draw_crosshair(&ctx, &nf_ch_path, x, y, 15.);
            ctx.draw_text(x+20.0, y+20.0, format!("({:.4}, {:.4})", x, y).as_str());
        }
        nf_ch_path.end(ctx);
        if let Some(aim_point) = state.wf_aim_point {
            draw_crosshair(&ctx, &wf_ch_path, aim_point.x*draw_params.area_width, aim_point.y*draw_params.area_height, 15.);
        }
        wf_ch_path.end(ctx);

        // let stroke = StrokeParams {
        //     cap: 0, // Bevel
        //     join: 0, // Flat
        //     thickness: 10.,
        //     miter_limit: 0.,
        //     dashes: vec![],
        //     dash_phase: 0.,
        // };

        // let brush = Brush::Solid(SolidBrush {
        //     r: 0.,
        //     g: 1.,
        //     b: 0.,
        //     a: 1.,
        // });

        // ctx.stroke(&fv_ch_path, &brush, &stroke);

        let stroke = StrokeParams {
            cap: 0, // Bevel
            join: 0, // Flat
            thickness: 5.,
            miter_limit: 0.,
            dashes: vec![],
            dash_phase: 0.,
        };

        let brush = Brush::Solid(SolidBrush {
            r: 1.,
            g: 0.,
            b: 0.,
            a: 1.,
        });

        ctx.stroke(&nf_ch_path, &brush, &stroke);

        let brush = Brush::Solid(SolidBrush {
            r: 0.,
            g: 0.,
            b: 1.,
            a: 1.,
        });

        ctx.stroke(&wf_ch_path, &brush, &stroke);
    }

    fn key_event(&mut self, _area: &Area, area_key_event: &AreaKeyEvent) -> bool {
        println!("{:?}", area_key_event);
        #[cfg(target_os = "windows")]
        let evt_key = area_key_event.ext_key as i32;
        #[cfg(not(target_os = "windows"))]
        let evt_key = area_key_event.ext_key;
        match evt_key {
            ui_sys::uiExtKeyUp => self.offset_y -= 1.0,
            ui_sys::uiExtKeyDown => self.offset_y += 1.0,
            ui_sys::uiExtKeyLeft => self.offset_x -= 1.0,
            ui_sys::uiExtKeyRight => self.offset_x += 1.0,
            _ => match area_key_event.key {
                b'w' => self.offset_y -= 100.0,
                b's' => self.offset_y += 100.0,
                b'a' => self.offset_x -= 100.0,
                b'd' => self.offset_x += 100.0,
                _ => (self.on_closing)(&mut self.window),
            }
        }
        true
    }
}
