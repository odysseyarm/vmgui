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
            state.nf_data = Some(nf_data);
            state.wf_data = Some(wf_data);
            if state.wf_data.unwrap()[0].area > 0 {
                println!("{:?}", state.wf_data.unwrap()[0]);
            }
            sleep(Duration::from_millis(5)).await;
        }
    }
}

#[derive(Default)]
pub struct TestCanvasState {
    pub nf_data: Option<[MotData; 16]>,
    pub wf_data: Option<[MotData; 16]>,
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

        let nf_path = Path::new(ctx, FillMode::Winding);
        let wf_path = Path::new(ctx, FillMode::Winding);
        let mut state = self.state.blocking_lock();
        if state.nf_data.is_some() {
            for mot_data in state.nf_data.expect("Nf data is None") {
                if mot_data.area == 0 {
                    break;
                }
                // todo don't use hardcoded 4096x4096 res assumption
                let x = mot_data.cx as f64 / 4096.;
                let y = mot_data.cy as f64 / 4096.;
                nf_path.add_rectangle(
                    ctx,
                    x * draw_params.area_width,
                    y * draw_params.area_height,
                    draw_params.area_width / 100.,
                    draw_params.area_width / 100.,
                );
            }
        }
        nf_path.end(ctx);
        if state.wf_data.is_some() {
            for mot_data in state.wf_data.expect("Wf data is None") {
                if mot_data.area == 0 {
                    break;
                }
                // todo don't use hardcoded 4096x4096 res assumption
                let x = mot_data.cx as f64 / 4096.;
                let y = mot_data.cy as f64 / 4096.;
                wf_path.add_rectangle(
                    ctx,
                    x * draw_params.area_width,
                    y * draw_params.area_height,
                    draw_params.area_width / 100.,
                    draw_params.area_width / 100.,
                );
            }
        }
        wf_path.end(ctx);

        let brush = Brush::Solid(SolidBrush {
            r: 1.,
            g: 0.,
            b: 0.,
            a: 1.,
        });

        draw_params.context.fill(&nf_path, &brush);

        let brush = Brush::Solid(SolidBrush {
            r: 0.,
            g: 0.,
            b: 1.,
            a: 1.,
        });

        draw_params.context.fill(&wf_path, &brush);
    }

    fn key_event(&mut self, _area: &Area, _area_key_event: &AreaKeyEvent) -> bool {
        (self.on_closing)(&mut self.window);
        true
    }
}
