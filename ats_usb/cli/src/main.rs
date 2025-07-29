use argmin::core::observers::ObserverMode;
use argmin::core::Error;
use argmin::core::{CostFunction, Executor, Gradient};
use argmin_observer_slog::SlogLogger;
use clap::Parser;
use futures::StreamExt;
use nalgebra::{SVector, Vector3};
use serde::{Deserialize, Serialize};
use serde_json::json;
use serialport::SerialPortType;
use std::time::Instant;
use tokio::io::{AsyncBufReadExt, BufReader};

#[derive(Serialize, Deserialize)]
struct AccelCalibration {
    b_x: f64,
    b_y: f64,
    b_z: f64,
    s_x: f64,
    s_y: f64,
    s_z: f64,
}

#[derive(Serialize, Deserialize)]
struct GyroCalibration {
    b_x: f64,
    b_y: f64,
    b_z: f64,
}

struct CostFn {
    measurements: Vec<Vector3<f64>>,
    g: f64,
}

impl CostFunction for CostFn {
    type Param = SVector<f64, 6>;
    type Output = f64;

    fn cost(&self, p: &Self::Param) -> Result<Self::Output, Error> {
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

            cost += (ax * ax + ay * ay + az * az - self.g * self.g).powi(2);
        }

        Ok(cost)
    }
}

impl Gradient for CostFn {
    type Param = SVector<f64, 6>;
    type Gradient = SVector<f64, 6>;

    // Calculate the gradient of the cost function
    fn gradient(&self, p: &Self::Param) -> Result<Self::Gradient, Error> {
        let b_x = p[0];
        let b_y = p[1];
        let b_z = p[2];
        let s_x = p[3];
        let s_y = p[4];
        let s_z = p[5];

        let mut grad = SVector::<f64, 6>::zeros();

        for m in &self.measurements {
            let ax = s_x * m.x + b_x;
            let ay = s_y * m.y + b_y;
            let az = s_z * m.z + b_z;

            let common_term = 4.0 * (ax * ax + ay * ay + az * az - self.g * self.g);

            grad[0] += common_term * ax;
            grad[1] += common_term * ay;
            grad[2] += common_term * az;
            grad[3] += common_term * ax * m.x;
            grad[4] += common_term * ay * m.y;
            grad[5] += common_term * az * m.z;
        }

        Ok(grad)
    }
}

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Serial port path
    #[arg(short, long)]
    path: Option<String>,

    /// Output file for accelerometer bias and scale calculation
    #[arg(long)]
    accel: Option<String>,

    /// Output file for gyroscope bias calculation
    #[arg(long)]
    gyro: Option<String>,

    /// Number of samples to use for bias and scale calculation
    #[arg(short, default_value_t = 400)]
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

    // Get accelerometer and gyroscope calibration parameters
    let accel_path = args.accel;
    let gyro_path = args.gyro;
    let num_samples = args.n;
    let gravity = args.g;

    let mut device = ats_usb::device::UsbDevice::connect_serial(path, false)
        .await
        .unwrap();

    println!("Connected to device");

    if let Some(gyro_path) = gyro_path {
        // Perform gyroscope bias calculation
        calculate_and_save_gyro_bias(&mut device, &gyro_path, num_samples).await;
    } else if let Some(accel_path) = accel_path {
        // Perform bias and scale calculation for accelerometer
        calculate_and_save_bias_scale(&mut device, &accel_path, num_samples, gravity).await;
    } else {
        // Normal streaming
        let mut s = device.stream_accel().await.unwrap();
        normal_streaming(&mut s).await;
    }
}

async fn calculate_and_save_bias_scale(
    device: &mut ats_usb::device::UsbDevice,
    accel_path: &str,
    num_samples: usize,
    g: f64,
) {
    let orientations = [
        "Place the device with the top side facing up.",
        "Place the device with the bottom side facing up.",
        "Place the device with the front side facing up.",
        "Place the device with the back side facing up.",
        "Place the device with the left side facing up.",
        "Place the device with the right side facing up.",
    ];

    let mut measurements = Vec::new();

    for orientation in &orientations {
        println!("{}", orientation);
        println!("Press Enter when ready to start collecting samples for this orientation.");
        let mut reader = BufReader::new(tokio::io::stdin());
        let mut input = String::new();
        reader.read_line(&mut input).await.unwrap();

        let mut s = device.stream_accel().await.unwrap();

        for _ in 0..num_samples {
            if let Some(v) = s.next().await {
                measurements.push(v.accel.cast::<f64>());
            }
        }

        println!("Collected samples for this orientation.");
    }

    // Validate measurements
    measurements.retain(|m| !m.x.is_nan() && !m.y.is_nan() && !m.z.is_nan());

    // Define cost function
    let cost_function = CostFn { measurements, g };

    // Initial parameter guess: [b_x, b_y, b_z, s_x, s_y, s_z]
    let init_param = nalgebra::Vector6::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    // Perform optimization with constraints and detailed logging
    let solver = argmin::solver::gradientdescent::SteepestDescent::new(
        argmin::solver::linesearch::MoreThuenteLineSearch::new(),
    );
    let result = Executor::new(cost_function, solver)
        .configure(|state| state.param(init_param).max_iters(1000))
        .add_observer(SlogLogger::term(), ObserverMode::Always)
        .run();

    match result {
        Ok(result) => {
            let solution = result.state.best_param.unwrap();

            // Extract solution
            let calibration = AccelCalibration {
                b_x: solution[0],
                b_y: solution[1],
                b_z: solution[2],
                s_x: solution[3],
                s_y: solution[4],
                s_z: solution[5],
            };

            // Save to JSON
            let data = json!(calibration);

            std::fs::write(accel_path, serde_json::to_string_pretty(&data).unwrap())
                .expect("Unable to write file");

            println!("Bias and scale saved to {}", accel_path);
        }
        Err(e) => {
            eprintln!("Optimization failed: {}", e);
        }
    }
}

async fn calculate_and_save_gyro_bias(
    device: &mut ats_usb::device::UsbDevice,
    gyro_path: &str,
    num_samples: usize,
) {
    let mut gyro_sum = Vector3::zeros();

    println!("Please make sure the device is held firmly in place.");
    println!("Press Enter when ready to start collecting samples for gyroscope calibration.");
    let mut reader = BufReader::new(tokio::io::stdin());
    let mut input = String::new();
    reader.read_line(&mut input).await.unwrap();

    let mut s = device.stream_accel().await.unwrap();

    for _ in 0..num_samples {
        if let Some(v) = s.next().await {
            gyro_sum += v.gyro.cast::<f64>();
        }
    }

    let gyro_bias = gyro_sum / (num_samples as f64);

    // Save gyro bias to JSON
    let calibration = GyroCalibration {
        b_x: gyro_bias.x,
        b_y: gyro_bias.y,
        b_z: gyro_bias.z,
    };

    let gyro_data = json!(calibration);

    std::fs::write(gyro_path, serde_json::to_string_pretty(&gyro_data).unwrap())
        .expect("Unable to write file");

    println!("Gyroscope bias saved to {}", gyro_path);
}

async fn normal_streaming(
    s: &mut (impl futures::Stream<Item = ats_usb::packets::vm::AccelReport> + Unpin),
) {
    let mut accel_sum = Vector3::zeros();
    let mut gyro_sum = Vector3::zeros();
    let mut count = 0.0;

    tokio::select! {
        _ = tokio::signal::ctrl_c() => {},
        _ = async {
            let mut prev = None;
            let mut sample_interval_average = None;
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
                    match sample_interval_average {
                        None => sample_interval_average = Some(dt),
                        Some(a) => {
                            sample_interval_average = Some(0.7 / 0.8 * a + 0.1 / 0.8 * dt);
                        }
                    }
                }
                let elapsed = if let Some(prev_timestamp) = prev_timestamp {
                    v.timestamp - prev_timestamp
                } else {
                    0
                };
                prev_timestamp = Some(v.timestamp);

                println!("accel = {:8.4?}, ||accel|| = {:7.4}, gyro = {:8.4?}, ts = {:9}, elapsed = {:7}, ODR = {:7.3?}", v.accel, v.accel.magnitude(), v.gyro, v.timestamp, elapsed, sample_interval_average.map(|s| 1.0 / s));
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
    }
    .into_iter()
    .filter(|port| {
        match &port.port_type {
            SerialPortType::UsbPort(port_info) => {
                if port_info.vid == 0x1915
                    && (port_info.pid == 0x520F
                        || port_info.pid == 0x5210
                        || port_info.pid == 0x5211)
                {
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
            }
            _ => false,
        }
    })
    .map(|port| port.port_name)
    .collect()
}
