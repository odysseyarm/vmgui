use std::{io::{Read, Write}, net::{TcpListener, TcpStream}, ops::Not, path::PathBuf, sync::{Arc, Mutex}, time::Duration};

use crossbeam::channel::{Receiver, TryRecvError};
use iui::{
    controls::{
        Window,
        WindowType,
    },
    UI,
};
use leptos_reactive::{Effect, RwSignal, SignalGet as _, SignalGetUntracked, SignalSet as _};
use opencv_ros_camera::RosOpenCvIntrinsics;
use tracing::{error, info};
use ats_usb::{device::encode_slip_frame, packet::{GeneralConfig, Packet, PacketData, PacketType, Props, ReadRegisterResponse, StreamUpdateAction}};

// Positive x is right
// Positive y is up
// Right hand rule (z is out from the screen)

fn main() {
    let port = 4444u16;

    let leptos_rt = leptos_reactive::create_runtime();

    let replay_file = std::env::args_os().nth(1).map(PathBuf::from);
    let ui = UI::init().expect("Couldn't initialize UI library");
    let mut main_win = Window::new(
        &ui,
        "Recording Playback",
        200,
        200,
        WindowType::NoMenubar,
    );
    let state = Arc::new(Mutex::new(State::new()));
    let stream_ctrl = crossbeam::channel::bounded(2);
    let stream_state = RwSignal::new(StreamState::Pause);
    Effect::new(move |_| { let _ = stream_ctrl.0.send(stream_state.get()); });


    vision_module_gui::layout! { &ui,
        let vbox = VerticalBox(padded: true) {
            Compact : let upload_btn = Button("Upload")
            Compact : let play_btn = Button(move || (!stream_state.get()).as_str())
        }
    }
    play_btn.hide(&ui);

    let open_file = |ui: &UI, path, state: &mut State, upload_btn: &mut iui::controls::Button, play_btn: &mut iui::controls::Button| {
        let (general_config, packets) = ats_playback::read_file(&path).unwrap();
        state.general_config = general_config;
        state.packets.lock().unwrap().clear();
        state.packets.lock().unwrap().extend(packets);
        upload_btn.hide(ui);
        play_btn.show(ui);
    };
    upload_btn.on_clicked(&ui, {
        let state = state.clone();
        let main_win = main_win.clone();
        let ui = ui.clone();
        let mut play_btn = play_btn.clone();
        move |btn| {
            let mut state = state.lock().unwrap();
            state.packets.lock().unwrap().clear();

            if let Some(path) = main_win.open_file(&ui) {
                open_file(&ui, path, &mut state, btn, &mut play_btn);
            }
        }
    });
    if let Some(replay_file) = replay_file {
        open_file(&ui, replay_file, &mut state.lock().unwrap(), &mut upload_btn, &mut play_btn);
    }

    play_btn.on_clicked(&ui, move |_| {
        stream_state.set(!stream_state.get_untracked());
    });

    main_win.set_child(&ui, vbox);
    main_win.show(&ui);

    let ui_ctx = ui.async_context();

    std::thread::Builder::new()
        .name("listener".into())
        .spawn(move || listener_thread(port, state, ui_ctx, stream_state, stream_ctrl.1))
        .unwrap();

    ui.main();

    leptos_rt.dispose();
}

fn listener_thread(port: u16, state: Arc<Mutex<State>>, ui_ctx: iui::concurrent::Context, stream_state: RwSignal<StreamState>, stream_ctrl: Receiver<StreamState>) {
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
                        socket_serve_thread(sock, state_clone, ui_ctx, stream_state);
                    })
                    .unwrap();
                let state_clone = state.clone();
                std::thread::Builder::new()
                    .name("stream".into())
                    .spawn(move || {
                        socket_stream_thread(sock2, state_clone, stream_ctrl);
                    })
                    .unwrap();
                break;
            }
            Err(e) => error!("couldn't get client: {e:?}"),
        }
    }
}

// Services the requests
fn socket_serve_thread(mut sock: TcpStream, state: Arc<Mutex<State>>, ui_ctx: iui::concurrent::Context, stream_state: RwSignal<StreamState>) {
    let mut buf = vec![0; 1024];
    let mut first_stream_enable = true;
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
                if s.action == StreamUpdateAction::DisableAll {
                    state.stream_impact = None;
                    state.stream_accel = None;
                    state.stream_object_report = None;
                    state.stream_combined_markers = None;
                } else {
                    match s.packet_id {
                        PacketType::ImpactReport => {
                            match s.action {
                                StreamUpdateAction::Enable => state.stream_impact = Some(pkt.id),
                                StreamUpdateAction::Disable => state.stream_impact = None,
                                StreamUpdateAction::DisableAll => { unreachable!() },
                            }
                        },
                        PacketType::AccelReport => {
                            match s.action {
                                StreamUpdateAction::Enable => state.stream_accel = Some(pkt.id),
                                StreamUpdateAction::Disable => state.stream_accel = None,
                                StreamUpdateAction::DisableAll => { unreachable!() },
                            }
                        },
                        PacketType::ObjectReport => {
                            match s.action {
                                StreamUpdateAction::Enable => state.stream_object_report = Some(pkt.id),
                                StreamUpdateAction::Disable => state.stream_object_report = None,
                                StreamUpdateAction::DisableAll => { unreachable!() },
                            }
                        },
                        PacketType::CombinedMarkersReport => {
                            match s.action {
                                StreamUpdateAction::Enable => state.stream_combined_markers = Some(pkt.id),
                                StreamUpdateAction::Disable => state.stream_combined_markers = None,
                                StreamUpdateAction::DisableAll => { unreachable!() },
                            }
                        },
                        _ => {},
                    }
                }
                if state.stream_combined_markers.is_some() && first_stream_enable {
                    ui_ctx.queue_main(move || stream_state.set(StreamState::Play));
                    first_stream_enable = false;
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
            PacketData::ReadProps() => Some(PacketData::ReadPropsResponse(state.props.clone())),
            PacketData::ReadPropsResponse(_) => unreachable!(),
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
fn socket_stream_thread(mut sock: TcpStream, state: Arc<Mutex<State>>, ctrl: Receiver<StreamState>) {
    let mut prev_timestamp = None;
    let mut buf = vec![0; 1024];
    let mut packet_index = 0;
    loop {
        match ctrl.try_recv() {
            Ok(StreamState::Pause) => {
                eprintln!("pause");
                loop {
                    match ctrl.recv() {
                        Ok(StreamState::Pause) => (),
                        Ok(StreamState::Play) => {
                            eprintln!("play");
                            break;
                        }
                        Err(_) => return,
                    }
                }
            },
            Ok(StreamState::Play) => (),
            Err(TryRecvError::Empty) => (),
            Err(TryRecvError::Disconnected) => return,
        }
        if state.lock().unwrap().packets.lock().unwrap().len() == 0 {
            std::thread::sleep(Duration::from_millis(100));
            continue;
        }

        let (timestamp, mut pkt) = state.lock().unwrap().packets.lock().unwrap()[packet_index].clone();

        if let Some(prev_timestamp) = prev_timestamp {
            let elapsed = timestamp - prev_timestamp;
            if elapsed > 0 {
                std::thread::sleep(Duration::from_millis(elapsed as u64));
            }
        }

        let state = state.lock().unwrap();

        if let Some(id) = match pkt.data {
            PacketData::ImpactReport(_) => {
                state.stream_impact
            }
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
        packet_index = packet_index + 1;
        if packet_index >= state.packets.lock().unwrap().len() {
            packet_index = 0;
            println!("looping");
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum StreamState {
    Play,
    Pause,
}

impl StreamState {
    fn as_str(self) -> &'static str {
        match self {
            StreamState::Play => "Play",
            StreamState::Pause => "Pause",
        }
    }
}

impl Not for StreamState {
    type Output = Self;

    fn not(self) -> Self::Output {
        match self {
            StreamState::Play => StreamState::Pause,
            StreamState::Pause => StreamState::Play,
        }
    }
}

struct State {
    stream_impact: Option<u8>,
    stream_accel: Option<u8>,
    stream_combined_markers: Option<u8>,
    stream_object_report: Option<u8>,
    packets: Arc<Mutex<Vec<(u128, Packet)>>>,
    general_config: GeneralConfig,
    props: Props,
}

impl State {
    fn new() -> Self {
        Self {
            stream_impact: None,
            stream_accel: None,
            stream_combined_markers: None,
            stream_object_report: None,
            packets: Arc::new(Mutex::new(vec![])),
            general_config: GeneralConfig {
                impact_threshold: 0,
                accel_config: Default::default(),
                gyro_config: Default::default(),
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
            },
            props: Props {
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
