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
            loop {
                let mut v = s.next().await.unwrap();
                println!("accel = {:8.4?}, ||accel|| = {:7.4}, gyro = {:8.4?}", v.accel, v.accel.magnitude(), v.gyro);
                count += 1.0;
                accel_sum += v.accel;
                gyro_sum += v.gyro;
            }
        }=> {},
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
