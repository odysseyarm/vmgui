use std::time::Instant;

use nalgebra::Vector3;
use futures::StreamExt;
use serialport::SerialPortType;

#[tokio::main]
async fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = match args.get(1) {
        Some(p) => p.clone(),
        None => {
            let mut ports = get_serialport_devices();
            if ports.len() == 0 {
                eprintln!("No devices connected");
                return;
            } else if ports.len() == 1 {
                ports.remove(0)
            } else {
                eprintln!("Multiple devices detected, please specify one: {:?}", ports);
                return;
            }
        }
    };
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
            let mut prev_timestamp = None;
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
                let elapsed = if let Some(prev_timestamp) = prev_timestamp {
                    v.timestamp - prev_timestamp
                } else {
                    0
                };
                prev_timestamp = Some(v.timestamp);

                println!("accel = {:8.4?}, ||accel|| = {:7.4}, gyro = {:8.4?}, ts = {:9}, elapsed = {:7}, ODR = {:7.3?}", v.accel, v.accel.magnitude(), v.gyro, v.timestamp, elapsed, odr_average);
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

fn get_serialport_devices() -> Vec<String> {
    let ports = serialport::available_ports();
    match ports {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to list serial ports: {}", e);
            return vec![];
        }
    }.into_iter().filter(|port| {
        match &port.port_type {
            SerialPortType::UsbPort(port_info) => {
                if port_info.vid == 0x1915 && port_info.pid == 0x520F || port_info.pid == 0x5210 {
                    if let Some(i) = port_info.interface {
                        // interface 0: cdc acm module
                        // interface 1: cdc acm module functional subordinate interface
                        // interface 2: cdc acm dfu
                        // interface 3: cdc acm dfu subordinate interface
                        i == 0
                    } else {
                        true
                    }
                } else {
                    false
                }
            },
            _ => false,
        }
    })
    .map(|port| port.port_name)
        .collect()
}
