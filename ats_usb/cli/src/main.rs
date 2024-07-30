use std::time::Instant;
use nalgebra::Vector3;
use futures::StreamExt;
use serialport::SerialPortType;
use serde_json::json;
use clap::{Arg, Parser};
use serde::{Serialize, Deserialize};
use argmin::solver::neldermead::NelderMead;
use argmin::core::{Executor, CostFunction};
use argmin::core::observers::ObserverMode;
use argmin_observer_slog::SlogLogger;
use argmin::core::Error;

#[derive(Serialize, Deserialize)]
struct Calibration {
    b_x: f64,
    b_y: f64,
    b_z: f64,
    s_x: f64,
    s_y: f64,
    s_z: f64,
}

struct CostFn {
    measurements: Vec<Vector3<f64>>,
    g: f64,
}

impl CostFunction for CostFn {
    type Param = nalgebra::SVector<f64, 6>;
    type Output = f64;

    fn cost(&self, p: &Self::Param) -> Result<Self::Output, argmin::core::Error> {
        let b_x = p[0];
        let b_y = p[1];
        let b_z = p[2];
        let s_x = p[3];
        let s_y = p[4];
        let s_z = p[5];

        let mut cost = 0.0;

        for m in &self.measurements {
            let ax = s_x * m.x + b_x;
            let ay = s_y * m.y + b_y;
            let az = s_z * m.z + b_z;

            // Check for potential overflow
            if ax.is_nan() || ay.is_nan() || az.is_nan() || ax.is_infinite() || ay.is_infinite() || az.is_infinite() {
                return Err(argmin::core::Error::msg("NaN or infinite value encountered in cost calculation"));
            }

            cost += (ax * ax + ay * ay + az * az - self.g * self.g).powi(2);

            // Overflow check
            if cost.is_nan() || cost.is_infinite() {
                return Err(argmin::core::Error::msg("NaN or infinite cost encountered"));
            }
        }

        Ok(cost)
    }
}

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Serial port path
    #[arg(short, long)]
    path: Option<String>,

    /// Output file for accelerometer bias and scale calculation
    #[arg(short)]
    a: Option<String>,

    /// Number of samples to use for accelerometer bias and scale calculation
    #[arg(short, default_value_t = 100)]
    n: usize,

    /// Gravity value to use for calculations
    #[arg(short, default_value_t = 9.80665)]
    g: f64,
}

#[tokio::main]
async fn main() {
    // Parse command-line arguments
    let args = Args::parse();

    // Get serial port path
    let path = args.path.or_else(|| {
        let mut ports = get_serialport_devices();
        if ports.len() == 0 {
            eprintln!("No devices connected");
            None
        } else if ports.len() == 1 {
            Some(ports.remove(0))
        } else {
            eprintln!("Multiple devices detected, please specify one: {:?}", ports);
            None
        }
    });

    let path = match path {
        Some(p) => p,
        None => return,
    };

    println!("Connecting to device at {}", path);

    // Get accelerometer bias and scale parameters
    let ac_path = args.a;
    let num_samples = args.n;
    let gravity = args.g;

    let device = ats_usb::device::UsbDevice::connect_serial(path, false)
        .await
        .unwrap();

    println!("Connected to device");

    let mut s = device.stream_accel().await.unwrap();

    if let Some(ac_path) = ac_path {
        // Perform bias and scale calculation
        calculate_and_save_bias_scale(&mut s, &ac_path, num_samples, gravity).await;
    } else {
        // Normal streaming
        normal_streaming(&mut s).await;
    }
}

async fn calculate_and_save_bias_scale(s: &mut (impl futures::Stream<Item = ats_usb::packet::AccelReport> + Unpin), ac_path: &str, num_samples: usize, g: f64) {
    let mut measurements = Vec::new();

    // Collect samples
    for _ in 0..num_samples {
        if let Some(v) = s.next().await {
            measurements.push(v.accel.cast::<f64>());
        }
    }

    // Validate measurements
    measurements.retain(|m| !m.x.is_nan() && !m.y.is_nan() && !m.z.is_nan());

    // Define cost function
    let cost_function = CostFn { measurements, g };

    // Initial parameter guess: [b_x, b_y, b_z, s_x, s_y, s_z]
    let init_param = nalgebra::Vector6::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    // add .2 to each parameter to avoid division by zero or whatever
    let keklol = nalgebra::Vector6::new(init_param[0] + 0.2, init_param[1] + 0.2, init_param[2] + 0.2, init_param[3] + 0.2, init_param[4] + 0.2, init_param[5] + 0.2);

    // Perform optimization with constraints and detailed logging
    let solver = NelderMead::new(vec![init_param.clone(), keklol.clone()]);
    let result = Executor::new(cost_function, solver)
        .configure(|state| {
            state
                .param(init_param)
                .max_iters(1000)
        })
        .add_observer(SlogLogger::term(), ObserverMode::Always)
        .run();

    match result {
        Ok(result) => {
            let solution = result.state.best_param.unwrap();

            // Extract solution
            let calibration = Calibration {
                b_x: solution[0],
                b_y: solution[1],
                b_z: solution[2],
                s_x: solution[3],
                s_y: solution[4],
                s_z: solution[5],
            };

            // Save to JSON
            let data = json!(calibration);

            std::fs::write(ac_path, serde_json::to_string_pretty(&data).unwrap()).expect("Unable to write file");

            println!("Bias and scale saved to {}", ac_path);
        }
        Err(e) => {
            eprintln!("Optimization failed: {}", e);
        }
    }
}

async fn normal_streaming(s: &mut (impl futures::Stream<Item = ats_usb::packet::AccelReport> + Unpin)) {
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

                // let bias = nalgebra::matrix![
                //     -0.0265612400362450;
                //     0.139760313286514;
                //     -0.00784890045575691;
                // ];
                // let scale = nalgebra::matrix![
                //     0.998286469439129;
                //     0.998740566223973;
                //     0.980932089902715;
                // ];
                // v.accel = v.accel.component_mul(&scale) + bias;

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
                if port_info.vid == 0x1915 && (port_info.pid == 0x520F || port_info.pid == 0x5210) {
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
