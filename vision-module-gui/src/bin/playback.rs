use std::{io::{Read, Write}, net::{TcpListener, TcpStream}, sync::{Arc, Mutex}, time::Duration};

use iui::{
    controls::{
        Window,
        WindowType,
    },
    UI,
};
use opencv_ros_camera::RosOpenCvIntrinsics;
use tracing::{error, info};
use ats_usb::{device::encode_slip_frame, packet::{GeneralConfig, Packet, PacketData, ReadRegisterResponse}};

// Positive x is right
// Positive y is up
// Right hand rule (z is out from the screen)

fn main() {
    let mut port = 4444u16;

    let leptos_rt = leptos_reactive::create_runtime();

    if let Some(port_arg) = std::env::args().nth(1) {
        port = port_arg.parse().unwrap();
    }
    let ui = UI::init().expect("Couldn't initialize UI library");
    let mut main_win = Window::new(
        &ui,
        "Recording Playback",
        200,
        200,
        WindowType::NoMenubar,
    );
    let state = Arc::new(Mutex::new(State::new()));
    vision_module_gui::layout! { &ui,
        let vbox = VerticalBox(padded: false) {
            Compact : let upload_btn = Button("Upload")
        }
    }

    upload_btn.on_clicked(&ui, {
        let state = state.clone();
        let main_win = main_win.clone();
        let ui = ui.clone();
        move |_| {
            let mut state = state.lock().unwrap();
            state.packets.lock().unwrap().clear();

            if let Some(path) = main_win.open_file(&ui) {
                let (general_config, packets) = ats_playback::read_file(&path).unwrap();
                state.general_config = general_config;
                state.packets.lock().unwrap().clear();
                state.packets.lock().unwrap().extend(packets);
            }
        }
    });

    main_win.set_child(&ui, vbox);
    main_win.show(&ui);

    std::thread::Builder::new()
        .name("listener".into())
        .spawn(move || listener_thread(port, state))
        .unwrap();

    ui.main();

    leptos_rt.dispose();
}

fn listener_thread(port: u16, state: Arc<Mutex<State>>) {
    let listener = TcpListener::bind(("127.0.0.1", port)).unwrap();
    loop {
        match listener.accept() {
            Ok((sock, addr)) => {
                info!("Connection from {}", addr);
                let sock2 = sock.try_clone().unwrap();
                let state_clone = state.clone();
                std::thread::Builder::new()
                    .name("serve".into())
                    .spawn(move || {
                        socket_serve_thread(sock, state_clone);
                    })
                    .unwrap();
                let state_clone = state.clone();
                std::thread::Builder::new()
                    .name("stream".into())
                    .spawn(move || {
                        socket_stream_thread(sock2, state_clone);
                    })
                    .unwrap();
            }
            Err(e) => error!("couldn't get client: {e:?}"),
        }
    }
}

// Services the requests
fn socket_serve_thread(mut sock: TcpStream, state: Arc<Mutex<State>>) {
    let mut buf = vec![0; 1024];
    loop {
        buf.resize(1024, 0);
        sock.read_exact(&mut buf[..3]).unwrap();
        assert_eq!(buf[0], 0xff);
        let len = u16::from_le_bytes([buf[1], buf[2]]);
        let len = usize::from(len) * 2;
        sock.read_exact(&mut buf[3..][..len - 2]).unwrap();
        let pkt = Packet::parse(&mut &buf[1..][..len]).unwrap();

        // hold the state lock as a hack to prevent the
        // stream thread from writing at the same time
        let mut state = state.lock().unwrap();
        let response = match pkt.data {
            PacketData::WriteRegister(_) => None,  // lmao no thanks
            PacketData::ReadRegister(r) => Some(PacketData::ReadRegisterResponse(ReadRegisterResponse { bank: r.bank, address: r.address, data: 0 })),
            PacketData::ReadRegisterResponse(_) => unreachable!(),
            PacketData::ObjectReportRequest(_) => todo!(),
            PacketData::ObjectReport(_) => unreachable!(),
            PacketData::StreamUpdate(s) => {
                if s.mask & 0b0100 != 0 {
                    state.stream_accel = if s.active { Some(pkt.id) } else { None };
                }
                if s.mask & 0b0010 != 0 {
                    state.stream_combined_markers = if s.active { Some(pkt.id) } else { None };
                }
                if s.mask & 0b0001 != 0 {
                    state.stream_object_report = if s.active { Some(pkt.id) } else { None };
                }
                None
            }
            PacketData::FlashSettings() => None,
            PacketData::CombinedMarkersReport(_) => unreachable!(),
            PacketData::ImpactReport(_) => unreachable!(),
            PacketData::AccelReport(_) => unreachable!(),
            PacketData::WriteConfig(_) => None,
            PacketData::ReadConfig() => Some(PacketData::ReadConfigResponse(state.general_config.clone())),
            PacketData::ReadConfigResponse(_) => unreachable!(),
        };

        if let Some(data) = response {
            buf.clear();
            Packet { id: pkt.id, data }.serialize(&mut buf);
            encode_slip_frame(&mut buf);
            sock.write_all(&buf).unwrap();
        }
    }
}

// Services the streams
fn socket_stream_thread(mut sock: TcpStream, state: Arc<Mutex<State>>) {
    let mut prev_timestamp = None;
    let mut buf = vec![0; 1024];
    let mut packet_index = 0;
    loop {
        if state.lock().unwrap().packets.lock().unwrap().len() == 0 {
            std::thread::sleep(Duration::from_millis(100));
            continue;
        }

        let (timestamp, mut pkt) = state.lock().unwrap().packets.lock().unwrap()[packet_index].clone();

        if let Some(prev_timestamp) = prev_timestamp {
            let elapsed = timestamp - prev_timestamp;
            if elapsed > 0 {
                std::thread::sleep(Duration::from_millis(elapsed as u64*2));
            }
        }

        let state = state.lock().unwrap();

        if let Some(id) = match pkt.data {
            PacketData::AccelReport(_) => {
                state.stream_accel
            }
            PacketData::CombinedMarkersReport(_) => {
                state.stream_combined_markers
            }
            PacketData::ObjectReport(_) => {
                state.stream_object_report
            }
            _ => None,
        } {
            pkt.id = id;
            buf.clear();
            pkt.serialize(&mut buf);
            encode_slip_frame(&mut buf);
            sock.write_all(&buf).unwrap();
        }

        prev_timestamp = Some(timestamp);
        packet_index = (packet_index + 1) % state.packets.lock().unwrap().len();
    }
}

struct State {
    stream_accel: Option<u8>,
    stream_combined_markers: Option<u8>,
    stream_object_report: Option<u8>,
    packets: Arc<Mutex<Vec<(i128, Packet)>>>,
    general_config: GeneralConfig,
}

impl State {
    fn new() -> Self {
        Self {
            stream_accel: None,
            stream_combined_markers: None,
            stream_object_report: None,
            packets: Arc::new(Mutex::new(vec![])),
            general_config: GeneralConfig {
                impact_threshold: 0,
                camera_model_nf: RosOpenCvIntrinsics::from_params(
                    49.0 * nf_focal_length() as f32,
                    0.,
                    49.0 * nf_focal_length() as f32,
                    49.,
                    49.,
                ),
                camera_model_wf: RosOpenCvIntrinsics::from_params(
                    49.0 * wf_focal_length() as f32,
                    0.,
                    49.0 * wf_focal_length() as f32,
                    49.,
                    49.,
                ),
                stereo_iso: nalgebra::Isometry3::identity(),
                accel_odr: 100,
                uuid: [42, 69, 3, 7, 9, 13],
            },
        }
    }
}

fn nf_focal_length() -> f64 {
    let fov_deg = 38.3;
    let fov = fov_deg / 180.0 * std::f64::consts::PI;
    1. / (fov / 2.).tan()
}

fn wf_focal_length() -> f64 {
    let fov_deg = 111.3;
    let fov = fov_deg / 180.0 * std::f64::consts::PI;
    1. / (fov / 2.).tan()
}
