//! Calibration functions



use argmin::core::{CostFunction, Error, Executor, Gradient, State};

use ats_usb::device::VmDevice;

use futures::StreamExt;

use nalgebra::Vector3;

use serde::{Deserialize, Serialize};

use serde_json::json;

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

    type Param = Vec<f64>;

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



            let diff = ax * ax + ay * ay + az * az - self.g * self.g;

            cost += diff * diff;

        }



        Ok(cost)

    }

}



impl Gradient for CostFn {

    type Param = Vec<f64>;

    type Gradient = Vec<f64>;



    fn gradient(&self, p: &Self::Param) -> Result<Self::Gradient, Error> {

        let b_x = p[0];

        let b_y = p[1];

        let b_z = p[2];

        let s_x = p[3];

        let s_y = p[4];

        let s_z = p[5];



        let mut grad = vec![0.0; 6];



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



pub async fn cmd_accel_calib(

    device: &mut VmDevice,

    num_samples: usize,

    gravity: f64,

    output_path: &str,

) -> Result<(), String> {

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

        reader

            .read_line(&mut input)

            .await

            .map_err(|e| format!("Failed to read input: {}", e))?;



        let mut s = device

            .stream_accel()

            .await

            .map_err(|e| format!("Failed to start accel stream: {}", e))?;



        for _ in 0..num_samples {

            if let Some(v) = s.next().await {

                measurements.push(v.accel.cast::<f64>());

            }

        }



        println!("Collected samples for this orientation.");

    }



    // Validate measurements

    measurements.retain(|m| !m.x.is_nan() && !m.y.is_nan() && !m.z.is_nan());



    println!(

        "Running optimization with {} measurements...",

        measurements.len()

    );



    // Define cost function

    let cost_function = CostFn {

        measurements,

        g: gravity,

    };



    // Initial parameter guess: [b_x, b_y, b_z, s_x, s_y, s_z]

    let init_param = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];



    // Build simplex around the initial guess for Nelder-Mead (n + 1 vertices)

    let mut simplex = Vec::with_capacity(init_param.len() + 1);

    simplex.push(init_param.clone());

    for i in 0..init_param.len() {

        let mut v = init_param.clone();

        v[i] += 0.05;

        simplex.push(v);

    }



    let solver = argmin::solver::neldermead::NelderMead::new(simplex);



    let executor = Executor::new(cost_function, solver).configure(|state| {

        state.param(init_param.clone()).max_iters(1000)

    });



    let result = executor.run();



    match result {

        Ok(result) => {

            let state = result.state;

            let solution = state

                .get_best_param()

                .or_else(|| state.get_param())

                .cloned()

                .ok_or_else(|| "Optimization returned no solution".to_string())?;



            println!("Optimization complete!");

            println!("  Final cost: {}", state.get_cost());

            println!("  Iterations: {}", state.get_iter());



            let calibration = AccelCalibration {

                b_x: solution[0],

                b_y: solution[1],

                b_z: solution[2],

                s_x: solution[3],

                s_y: solution[4],

                s_z: solution[5],

            };



            let data = json!(calibration);

            std::fs::write(output_path, serde_json::to_string_pretty(&data).unwrap())

                .map_err(|e| format!("Unable to write file: {}", e))?;



            println!("Bias and scale saved to {}", output_path);

            Ok(())

        }

        Err(e) => Err(format!("Optimization failed: {}", e)),

    }

}



pub async fn cmd_gyro_calib(

    device: &mut VmDevice,

    num_samples: usize,

    output_path: &str,

) -> Result<(), String> {

    let mut gyro_sum = Vector3::<f64>::zeros();



    println!("Please make sure the device is held firmly in place.");

    println!("Press Enter when ready to start collecting samples for gyroscope calibration.");

    let mut reader = BufReader::new(tokio::io::stdin());

    let mut input = String::new();

    reader

        .read_line(&mut input)

        .await

        .map_err(|e| format!("Failed to read input: {}", e))?;



    let mut s = device

        .stream_accel()

        .await

        .map_err(|e| format!("Failed to start accel stream: {}", e))?;



    for _ in 0..num_samples {

        if let Some(v) = s.next().await {

            gyro_sum += v.gyro.cast::<f64>();

        }

    }



    let gyro_bias = gyro_sum / (num_samples as f64);



    let calibration = GyroCalibration {

        b_x: gyro_bias.x,

        b_y: gyro_bias.y,

        b_z: gyro_bias.z,

    };



    let gyro_data = json!(calibration);

    std::fs::write(

        output_path,

        serde_json::to_string_pretty(&gyro_data).unwrap(),

    )

    .map_err(|e| format!("Unable to write file: {}", e))?;



    println!("Gyroscope bias saved to {}", output_path);

    Ok(())

}



pub async fn cmd_stream(device: &mut VmDevice) -> Result<(), String> {

    let mut s = device

        .stream_accel()

        .await

        .map_err(|e| format!("Failed to start accel stream: {}", e))?;



    let mut accel_sum = Vector3::<f64>::zeros();

    let mut gyro_sum = Vector3::<f64>::zeros();

    let mut count = 0.0;



    println!("Streaming data... Press Ctrl+C to stop.");



    tokio::select! {

        _ = tokio::signal::ctrl_c() => {},

        _ = async {

            let mut prev_timestamp = None;

            loop {

                let v = s.next().await.unwrap();



                let elapsed = if let Some(prev_timestamp) = prev_timestamp {

                    v.timestamp - prev_timestamp

                } else {

                    0

                };

                prev_timestamp = Some(v.timestamp);



                println!(

                    "accel = {:8.4?}, ||accel|| = {:7.4}, gyro = {:8.4?}, ts = {:9}, elapsed = {:7}",

                    v.accel, v.accel.magnitude(), v.gyro, v.timestamp, elapsed

                );

                count += 1.0;

                accel_sum += v.accel.cast::<f64>();

                gyro_sum += v.gyro.cast::<f64>();

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



    Ok(())

}

