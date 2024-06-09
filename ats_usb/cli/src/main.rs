use std::time::Instant;

use nalgebra::Vector3;
use futures::StreamExt;

#[tokio::main]
async fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = &args[1];
    let device = ats_usb::device::UsbDevice::connect_serial(path, false)
        .await
        .unwrap();
    let mut s = device.stream_accel().await.unwrap();

    let mut accel_sum = Vector3::zeros();
    let mut gyro_sum = Vector3::zeros();
    let mut count = 0.0;

    tokio::select! {
        _ = tokio::signal::ctrl_c() => {},
        _ = async {
            let mut prev = None;
            let mut odr_average = None;
            loop {
                let v = s.next().await.unwrap();
                let dt = match prev {
                    None => {
                        prev = Some(Instant::now());
                        None
                    },
                    Some(p) => {
                        let e = p.elapsed().as_secs_f64();
                        prev = Some(Instant::now());
                        Some(e)
                    }
                };
                if let Some(dt) = dt {
                    match odr_average {
                        None => odr_average = Some(1. / dt),
                        Some(a) => {
                            odr_average = Some(0.7 / 0.8 * a + 0.1 / 0.8 * (1.0 / dt));
                        }
                    }
                }

                println!("accel = {:8.4?}, ||accel|| = {:7.4}, gyro = {:8.4?}, ODR = {:7.3?}", v.accel, v.accel.magnitude(), v.gyro, odr_average);
                count += 1.0;
                accel_sum += v.accel;
                gyro_sum += v.gyro;
            }
        } => {},
    }

    println!();
    println!("Mean accel: {:.8?}", accel_sum / count);
    println!(
        "Mean accel magnitude: {:.8?}",
        (accel_sum / count).magnitude()
    );
    println!("Mean gyro: {:.8?}", gyro_sum / count);
    println!("Samples: {count}");
}
