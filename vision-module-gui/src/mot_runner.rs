use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio::time::sleep;
use crate::CloneButShorter;
use crate::device::UsbDevice;
use crate::test_procedure::MotState;

pub struct MotRunner {
    pub state: Arc<Mutex<MotState>>,
    pub device: Option<UsbDevice>,
}

impl MotRunner {
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
