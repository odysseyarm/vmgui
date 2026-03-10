//! Unified CLI tool for managing ATS devices (mux and direct device connections)

use std::process::ExitCode;

use clap::{Parser, Subcommand};

mod mux;
mod device;
mod calibration;
mod bond;

#[derive(Parser)]
#[command(name = "ats-cli")]
#[command(about = "CLI tool for managing ATS mux and devices", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Mux (dongle) commands
    Mux {
        /// Index of the mux to use (use 'mux list-devices' to see available muxes)
        #[arg(short, long)]
        device: Option<usize>,

        #[command(subcommand)]
        command: mux::MuxCommands,
    },
    /// Direct device commands (for lite/vm connected via USB)
    Device {
        /// Index of the device to use (use 'device list-devices' to see available devices)
        #[arg(short, long)]
        device: Option<usize>,

        #[command(subcommand)]
        command: device::DeviceCommands,
    },
    /// Create a manual bond between a dongle and a device
    Bond(bond::BondArgs),
}

#[tokio::main]
async fn main() -> ExitCode {
    let cli = Cli::parse();

    let result = match cli.command {
        Commands::Mux { device, command } => mux::handle_command(device, command).await,
        Commands::Device { device, command } => device::handle_command(device, command).await,
        Commands::Bond(args) => bond::handle_bond(args).await,
    };

    match result {
        Ok(_) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Error: {e}");
            ExitCode::FAILURE
        }
    }
}
