use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream, UdpSocket},
    ops::Not,
    path::PathBuf,
    sync::{Arc, Mutex},
    time::Duration,
};

use ats_usb::{
    device::encode_slip_frame,
    packet::{
        GeneralConfig, Packet, PacketData, PacketType, Props, ReadRegisterResponse,
        StreamUpdateAction,
    }, udp_stream::UdpStream,
};
use crossbeam::channel::{Receiver, TryRecvError};
use iui::{
    controls::{Window, WindowType},
    UI,
};
use leptos_reactive::{Effect, RwSignal, SignalGet as _, SignalGetUntracked, SignalSet as _};
use opencv_ros_camera::RosOpenCvIntrinsics;
use tracing::{error, info, Level};
use tracing_subscriber::EnvFilter;

// Positive x is right
// Positive y is up
// Right hand rule (z is out from the screen)

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::builder()
                .with_env_var("RUST_LOG")
                .with_default_directive(Level::INFO.into())
                .from_env_lossy(),
        )
        .init();

    let port = 4444u16;

    let leptos_rt = leptos_reactive::create_runtime();

    let replay_file = std::env::args_os().nth(1).map(PathBuf::from);
    let ui = UI::init().expect("Couldn't initialize UI library");
    let mut main_win = Window::new(&ui, "Recording Playback", 200, 200, WindowType::NoMenubar);
    let state = Arc::new(Mutex::new(State::new()));
    let stream_ctrl = crossbeam::channel::bounded(2);
    let stream_state = RwSignal::new(StreamState::Pause);
    Effect::new(move |_| {
        let _ = stream_ctrl.0.send(stream_state.get());
    });

    vision_module_gui::layout! { &ui,
        let vbox = VerticalBox(padded: true) {
            Compact : let upload_btn = Button("Upload")
            Compact : let play_btn = Button(move || (!stream_state.get()).as_str())
        }
    }
    play_btn.hide(&ui);

    let open_file = |ui: &UI,
                     path,
                     state: &mut State,
                     upload_btn: &mut iui::controls::Button,
                     play_btn: &mut iui::controls::Button| {
        let (general_config, packets) = ats_playback::read_file(&path).unwrap();
        state.general_config = general_config;
        state.packets.clear();
        state.packets.extend(packets);
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
            state.packets.clear();

            if let Some(path) = main_win.open_file(&ui) {
                open_file(&ui, path, &mut state, btn, &mut play_btn);
            }
        }
    });
    if let Some(replay_file) = replay_file {
        open_file(
            &ui,
            replay_file,
            &mut state.lock().unwrap(),
            &mut upload_btn,
            &mut play_btn,
        );
    }

    play_btn.on_clicked(&ui, move |_| {
        stream_state.set(!stream_state.get_untracked());
    });

    main_win.set_child(&ui, vbox);
    main_win.show(&ui);

    let ui_ctx = ui.async_context();

    let state_clone = state.clone();
    std::thread::Builder::new()
        .name("stream".into())
        .spawn(move || {
            socket_stream_thread(state_clone, stream_ctrl.1);
        })
        .unwrap();
    let state_clone = state.clone();
    std::thread::Builder::new()
        .name("listener-tcp".into())
        .spawn(move || listener_thread_tcp(port, state_clone, ui_ctx, stream_state))
        .unwrap();
    std::thread::Builder::new()
        .name("listener-udp".into())
        .spawn(move || listener_thread_udp(state, ui_ctx, stream_state))
        .unwrap();

    ui.main();

    leptos_rt.dispose();
}

fn listener_thread_tcp(
    port: u16,
    state: Arc<Mutex<State>>,
    ui_ctx: iui::concurrent::Context,
    stream_state: RwSignal<StreamState>,
) {
    let listener = TcpListener::bind(("127.0.0.1", port)).unwrap();
    loop {
        match listener.accept() {
            Ok((sock, addr)) => {
                info!("Connection from {}", addr);
                let sock2 = sock.try_clone().unwrap();
                let conn = Arc::new(Mutex::new(Connection::new(sock2)));
                state.lock().unwrap().connections.push(conn.clone());
                std::thread::Builder::new()
                    .name("serve-tcp".into())
                    .spawn(move || {
                        socket_serve_thread(conn, sock, state, ui_ctx, stream_state);
                    })
                    .unwrap();
                break;
            }
            Err(e) => error!("couldn't get client: {e:?}"),
        }
    }
}

fn listener_thread_udp(
    state: Arc<Mutex<State>>,
    ui_ctx: iui::concurrent::Context,
    stream_state: RwSignal<StreamState>,
) {
    let socket = UdpSocket::bind(("127.31.33.7", 23456)).unwrap();
    let sock = UdpSocket::bind(("127.0.0.1", 0)).unwrap();
    info!("Local UDP address is {}", sock.local_addr().unwrap());
    let broadcast_src_addr;
    let mut buf = [0; 1472];
    loop {
        match socket.recv_from(&mut buf) {
            Ok((n, addr)) if buf[..n] == [255, 3] => {
                info!("UDP broadcast ping from {}", addr);
                sock.send_to(&[1, 1], addr).unwrap();
                broadcast_src_addr = addr;
                break;
            }
            Ok(_) => (),
            Err(e) => error!("recv error: {e}"),
        }
    }
    loop {
        match sock.recv_from(&mut buf) {
            Ok((n, addr)) if buf[..n] == [255, 1] => {
                info!("UDP ping from {}", addr);
                sock.connect(addr).unwrap();
                let sock2 = sock.try_clone().unwrap();
                let sock3 = sock.try_clone().unwrap();
                let sock4 = sock.try_clone().unwrap();
                let read_write = UdpStream::with_capacity(sock, 1472, 1472, &[1, 0], &[1, 0]);
                let write = UdpStream::with_capacity(sock2, 0, 1472, &[], &[1, 0]);
                let conn = Arc::new(Mutex::new(Connection::new(write)));
                state.lock().unwrap().connections.push(conn.clone());
                std::thread::Builder::new()
                    .name("serve-udp".into())
                    .spawn(move || {
                        socket_serve_thread(conn, read_write, state, ui_ctx, stream_state);
                    })
                    .unwrap();
                std::thread::Builder::new()
                    .name("handle-broadcast-ping-udp".into())
                    .spawn(move || {
                        loop {
                            std::thread::sleep(Duration::from_secs(1));
                            sock3.send_to(&[1, 1], broadcast_src_addr).unwrap();
                        }
                    })
                    .unwrap();
                std::thread::Builder::new()
                    .name("handle-ping-udp".into())
                    .spawn(move || {
                        loop {
                            std::thread::sleep(Duration::from_secs(1));
                            sock4.send_to(&[1, 1], addr).unwrap();
                        }
                    })
                    .unwrap();
                break;
            }
            Ok(_) => (),
            Err(e) => error!("recv error: {e}"),
        }
    }
}

// Services the requests
fn socket_serve_thread(
    conn: Arc<Mutex<Connection>>,
    mut sock: impl Read + Write,
    state: Arc<Mutex<State>>,
    ui_ctx: iui::concurrent::Context,
    stream_state: RwSignal<StreamState>,
) {
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
        info!("pkt = {pkt:?}");

        // hold the state lock as a hack to prevent the
        // stream thread from writing at the same time
        let mut conn = conn.lock().unwrap();
        let response = match pkt.data {
            PacketData::WriteRegister(_) => None, // lmao no thanks
            PacketData::ReadRegister(r) => {
                Some(PacketData::ReadRegisterResponse(ReadRegisterResponse {
                    bank: r.bank,
                    address: r.address,
                    data: 0,
                }))
            }
            PacketData::ReadRegisterResponse(_) => unreachable!(),
            PacketData::ObjectReportRequest(_) => todo!(),
            PacketData::ObjectReport(_) => unreachable!(),
            PacketData::StreamUpdate(s) => {
                if s.action == StreamUpdateAction::DisableAll {
                    conn.stream_impact = None;
                    conn.stream_accel = None;
                    conn.stream_object_report = None;
                    conn.stream_combined_markers = None;
                } else {
                    match s.packet_id {
                        PacketType::ImpactReport() => match s.action {
                            StreamUpdateAction::Enable => conn.stream_impact = Some(pkt.id),
                            StreamUpdateAction::Disable => conn.stream_impact = None,
                            StreamUpdateAction::DisableAll => unreachable!(),
                        },
                        PacketType::AccelReport() => match s.action {
                            StreamUpdateAction::Enable => conn.stream_accel = Some(pkt.id),
                            StreamUpdateAction::Disable => conn.stream_accel = None,
                            StreamUpdateAction::DisableAll => unreachable!(),
                        },
                        PacketType::ObjectReport() => match s.action {
                            StreamUpdateAction::Enable => conn.stream_object_report = Some(pkt.id),
                            StreamUpdateAction::Disable => conn.stream_object_report = None,
                            StreamUpdateAction::DisableAll => unreachable!(),
                        },
                        PacketType::CombinedMarkersReport() => match s.action {
                            StreamUpdateAction::Enable => conn.stream_combined_markers = Some(pkt.id),
                            StreamUpdateAction::Disable => conn.stream_combined_markers = None,
                            StreamUpdateAction::DisableAll => unreachable!(),
                        },
                        _ => {}
                    }
                }
                if conn.stream_combined_markers.is_some() && first_stream_enable {
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
            PacketData::ReadConfig() => {
                Some(PacketData::ReadConfigResponse(state.lock().unwrap().general_config.clone()))
            }
            PacketData::ReadConfigResponse(_) => unreachable!(),
            PacketData::ReadProps() => Some(PacketData::ReadPropsResponse(state.lock().unwrap().props.clone())),
            PacketData::ReadPropsResponse(_) => unreachable!(),
            PacketData::Vendor(..) => None,
        };

        if let Some(data) = response {
            buf.clear();
            Packet { id: pkt.id, data }.serialize(&mut buf);
            encode_slip_frame(&mut buf);
            sock.write_all(&buf).unwrap();
            sock.flush().unwrap();
        }
    }
}

// Services the streams
fn socket_stream_thread(
    state: Arc<Mutex<State>>,
    ctrl: Receiver<StreamState>,
) {
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
            }
            Ok(StreamState::Play) => (),
            Err(TryRecvError::Empty) => (),
            Err(TryRecvError::Disconnected) => return,
        }
        if state.lock().unwrap().packets.len() == 0 {
            std::thread::sleep(Duration::from_millis(100));
            continue;
        }

        let (timestamp, mut pkt) =
            state.lock().unwrap().packets[packet_index].clone();

        if let Some(prev_timestamp) = prev_timestamp {
            let elapsed = timestamp - prev_timestamp;
            if elapsed > 0 {
                std::thread::sleep(Duration::from_millis(elapsed as u64));
            }
        }

        let state = state.lock().unwrap();

        for conn in &state.connections {
            let mut conn = conn.lock().unwrap();
            if let Some(id) = match pkt.data {
                PacketData::ImpactReport(_) => conn.stream_impact,
                PacketData::AccelReport(_) => conn.stream_accel,
                PacketData::CombinedMarkersReport(_) => conn.stream_combined_markers,
                PacketData::ObjectReport(_) => conn.stream_object_report,
                _ => None,
            } {
                pkt.id = id;
                buf.clear();
                pkt.serialize(&mut buf);
                encode_slip_frame(&mut buf);
                conn.sink.write_all(&buf).unwrap();
            }
        }
        prev_timestamp = Some(timestamp);
        packet_index = packet_index + 1;
        if packet_index >= state.packets.len() {
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
    packets: Vec<(u128, Packet)>,
    connections: Vec<Arc<Mutex<Connection>>>,
    general_config: GeneralConfig,
    props: Props,
}

struct Connection {
    stream_impact: Option<u8>,
    stream_accel: Option<u8>,
    stream_combined_markers: Option<u8>,
    stream_object_report: Option<u8>,
    sink: Box<dyn Write + Send>,
}

impl Connection {
    fn new(sink: impl Write + Send + 'static) -> Self {
        Self {
            stream_impact: None,
            stream_accel: None,
            stream_combined_markers: None,
            stream_object_report: None,
            sink: Box::new(sink),
        }
    }
}

impl State {
    fn new() -> Self {
        Self {
            packets: vec![],
            connections: vec![],
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
