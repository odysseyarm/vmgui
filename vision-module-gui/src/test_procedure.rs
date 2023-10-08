use std::f64::consts::PI;
use std::ops::DerefMut;
use std::sync::Arc;
use std::time::Duration;
use leptos_reactive::{ReadSignal, SignalGetUntracked};
use tokio::sync::Mutex;
use tokio::time::sleep;
use iui::controls::{Area, AreaDrawParams, AreaHandler, AreaKeyEvent, Window};
use iui::draw::{Brush, FillMode, Path, SolidBrush};
use iui::UI;
use vision_module_gui::CloneButShorter;
use vision_module_gui::device::UsbDevice;
use vision_module_gui::packet::MotData;

pub struct TestProcedureView {
    pub state: Arc<Mutex<TestCanvasState>>,
    pub device: Option<UsbDevice>,
}

impl TestProcedureView {
    pub async fn run(&self) {
        loop {
            if self.device.is_none() {
                return;
            }
            let device = self.device.c().unwrap();
            let (nf_data, wf_data) = device.get_frame().await.expect("Problem getting frame from device");
            let mut state = self.state.lock().await;
            let state = state.deref_mut();
            state.nf_data = Some(nf_data);
            state.wf_data = Some(wf_data);
            println!("{:?}", state.wf_data);
            sleep(Duration::from_millis(5)).await;
        }
    }
}

#[derive(Default)]
pub struct TestCanvasState {
    pub nf_data: Option<MotData>,
    pub wf_data: Option<MotData>,
}

pub struct TestCanvas {
    pub ctx: UI,
    pub window: Window,
    pub on_closing: Box<dyn FnMut(&mut Window)>,
    pub state: Arc<Mutex<TestCanvasState>>,
}

impl AreaHandler for TestCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
        let ctx = &draw_params.context;

        let path = Path::new(ctx, FillMode::Winding);
        path.add_rectangle(ctx, 0., 0., draw_params.area_width, draw_params.area_height);
        path.end(ctx);

        let brush = Brush::Solid(SolidBrush {
            r: 0.2,
            g: 0.6,
            b: 0.8,
            a: 1.,
        });

        draw_params.context.fill(&path, &brush);

        let path = Path::new(ctx, FillMode::Winding);
        for i in 0..100 {
            let x = i as f64 / 100.;
            let y = ((x * PI * 2.).sin() + 1.) / 2.;
            path.add_rectangle(
                ctx,
                x * draw_params.area_width,
                0.,
                draw_params.area_width / 100.,
                y * draw_params.area_height,
            );
        }
        path.end(ctx);

        let brush = Brush::Solid(SolidBrush {
            r: 0.2,
            g: 0.,
            b: 0.3,
            a: 1.,
        });

        draw_params.context.fill(&path, &brush);
    }

    fn key_event(&mut self, _area: &Area, _area_key_event: &AreaKeyEvent) -> bool {
        (self.on_closing)(&mut self.window);
        true
    }
}
