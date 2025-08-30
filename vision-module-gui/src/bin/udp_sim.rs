use std::{
    f64::consts::{FRAC_1_SQRT_2, PI},
    io::{Read, Write},
    net::{Ipv4Addr, SocketAddr, UdpSocket},
    ops::{Bound, RangeBounds},
    sync::{Arc, Mutex},
    time::Duration,
};

use socket2::Socket;

use ats_usb::{
    packets::vm::{
        CombinedMarkersReport, GeneralConfig, ObjectReport, Packet, PacketData, PacketType,
        ReadRegisterResponse, StreamUpdateAction, Version,
    },
    udp_stream::UdpStream,
};
use iui::{
    controls::{
        Area, AreaDrawParams, AreaHandler, AreaKeyEvent, AreaMouseEvent, Modifiers, Window,
        WindowType,
    },
    draw::{Brush, DrawContext, FillMode, Path, SolidBrush, StrokeParams},
    UI,
};
use nalgebra::{
    coordinates::XY, Matrix3, Matrix4, Point, Point2, Point3, Rotation3, SVector, Scale2, Scale3,
    Transform3, Translation2, Translation3, Vector3, Vector4,
};
use opencv_ros_camera::RosOpenCvIntrinsics;
use tracing::{error, info};
use vision_module_gui::{custom_shapes::draw_diamond, mot_runner::sort_rectangle};

// Positive x is right
// Positive y is up
// Right hand rule (z is out from the screen)

fn main() {
    tracing_subscriber::fmt().init();
    let mut port = 23456u16;
    if let Some(port_arg) = std::env::args().nth(1) {
        port = port_arg.parse().unwrap();
    }
    let ui = UI::init().expect("Couldn't initialize UI library");
    let mut main_win = Window::new(
        &ui,
        "Vision Module Simulator",
        640,
        480,
        WindowType::NoMenubar,
    );
    let state = Arc::new(Mutex::new(State::new()));
    vision_module_gui::layout! { &ui,
        let vbox = VerticalBox(padded: false) {
            Compact : let text = Label("")
            Stretchy : let area = Area(Box::new(MainCanvas { state: state.clone() }))
        }
    }
    main_win.set_child(&ui, vbox);
    main_win.show(&ui);
    ui.ui_timer(16, {
        let ui = ui.clone();
        let area = area.clone();
        let state = state.clone();
        move || {
            let p = *state.lock().unwrap().camera_pos;
            text.set_text(&ui, &format!("p: ({:.4}, {:.4}, {:.4})", p.x, p.y, p.z));
            area.queue_redraw_all(&ui);
            true
        }
    });

    std::thread::Builder::new()
        .name("listener".into())
        .spawn(move || listener_thread(port, state))
        .unwrap();

    let mut ev = ui.event_loop();
    ev.run(&ui);
}

fn listener_thread(port: u16, state: Arc<Mutex<State>>) {
    let client_read_write_thread_map = Arc::new(Mutex::new(std::collections::HashMap::new()));
    let listener = Socket::new(
        socket2::Domain::IPV4,
        socket2::Type::DGRAM,
        Some(socket2::Protocol::UDP),
    )
    .unwrap();
    let address: SocketAddr = (Ipv4Addr::UNSPECIFIED, port).into();
    let address = address.into();
    listener.set_reuse_address(true).unwrap();
    listener.bind(&address).unwrap();
    let listener: UdpSocket = listener.into();

    let client = UdpSocket::bind(("0.0.0.0", 0)).unwrap();
    loop {
        let mut buf = vec![0; 1472];
        match listener.recv_from(&mut buf) {
            Ok((amt, src1)) => {
                if amt < 2 || buf[0] != 0xff || buf[1] != 0x03 {
                    continue;
                }

                client.send_to(&[0x01, 0x01], src1).unwrap();

                {
                    let client_read_write_thread_map = client_read_write_thread_map.clone();
                    let client_read_write_thread_map = client_read_write_thread_map.lock().unwrap();
                    if client_read_write_thread_map.contains_key(&src1) {
                        continue;
                    }
                }

                let mut buf = vec![0; 1472];
                let (amt, src2) = client.recv_from(&mut buf).unwrap();
                if amt < 2 || buf[0] != 0xff || buf[1] != 0x01 {
                    continue;
                }

                info!("Connection from {}", src2);

                client.send_to(&[0x01, 0x01], src2).unwrap();

                client.connect(src2).unwrap();

                let sock = client.try_clone().unwrap();
                let sock2 = client.try_clone().unwrap();

                let read = UdpStream::with_capacity(sock, 1472, 1472, &[1, 0], &[1, 0]);
                let write = UdpStream::with_capacity(sock2, 0, 1472, &[], &[1, 0]);
                let state_clone = state.clone();
                let read_task = std::thread::Builder::new()
                    .name("serve".into())
                    .spawn(move || {
                        socket_serve_thread(read, state_clone);
                    })
                    .unwrap();
                let state_clone = state.clone();
                let write_task = std::thread::Builder::new()
                    .name("stream".into())
                    .spawn(move || {
                        socket_stream_thread(write, state_clone);
                    })
                    .unwrap();
                let join_task = {
                    let client_read_write_thread_map = client_read_write_thread_map.clone();
                    std::thread::Builder::new()
                        .name("join".into())
                        .spawn(move || {
                            // the threads shouldn't really ever finish right now but just for the spirit
                            read_task.join().unwrap();
                            write_task.join().unwrap();
                            client_read_write_thread_map.lock().unwrap().remove(&src1);
                        })
                        .unwrap()
                };

                client_read_write_thread_map
                    .lock()
                    .unwrap()
                    .insert(src1, join_task);
            }
            Err(e) => {
                if e.raw_os_error() != Some(10040) {
                    error!("{e}");
                }
            }
        }
    }
}

// Services the requests
fn socket_serve_thread(mut sock: UdpStream, state: Arc<Mutex<State>>) {
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
            PacketData::WriteRegister(_) => None, // lmao no thanks
            PacketData::ReadRegister(r) => {
                Some(PacketData::ReadRegisterResponse(ReadRegisterResponse {
                    bank: r.bank,
                    address: r.address,
                    data: 0,
                }))
            }
            PacketData::ReadRegisterResponse(_) => unreachable!(),
            PacketData::ObjectReportRequest() => todo!(),
            PacketData::ObjectReport(_) => unreachable!(),
            PacketData::StreamUpdate(s) => {
                if s.action == StreamUpdateAction::DisableAll {
                    state.stream_mot = None;
                    state.stream_combined_markers = None;
                } else {
                    match s.packet_id {
                        PacketType::ObjectReport() => match s.action {
                            StreamUpdateAction::Enable => state.stream_mot = Some(pkt.id),
                            StreamUpdateAction::Disable => state.stream_mot = None,
                            _ => unreachable!(),
                        },
                        PacketType::CombinedMarkersReport() => match s.action {
                            StreamUpdateAction::Enable => {
                                state.stream_combined_markers = Some(pkt.id)
                            }
                            StreamUpdateAction::Disable => state.stream_combined_markers = None,
                            _ => unreachable!(),
                        },
                        _ => {}
                    }
                }
                None
            }
            PacketData::FlashSettings() => None,
            PacketData::CombinedMarkersReport(_) => unreachable!(),
            PacketData::PocMarkersReport(_) => unreachable!(),
            PacketData::ImpactReport(_) => unreachable!(),
            PacketData::AccelReport(_) => unreachable!(),
            PacketData::WriteConfig(_) => None,
            PacketData::ReadConfig() => Some(PacketData::ReadConfigResponse(GeneralConfig {
                impact_threshold: 5,
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
                suppress_ms: 0,
            })),
            PacketData::ReadConfigResponse(_) => unreachable!(),
            PacketData::ReadProps() => Some(PacketData::ReadPropsResponse(Default::default())),
            PacketData::ReadPropsResponse(_) => unreachable!(),
            PacketData::ReadVersion() => Some(PacketData::ReadVersionResponse(
                    Version { protocol_semver: [0, 1, 0], firmware_semver: [0, 1, 0] }
            )),
            PacketData::ReadVersionResponse(_) => unreachable!(),
            PacketData::Ack() => unreachable!(),
            PacketData::WriteMode(_) => None,
            PacketData::Vendor(_, _) => None,
        };

        if let Some(data) = response {
            buf.clear();
            Packet { id: pkt.id, data }.serialize(&mut buf);
            let vec = cobs::encode_vec(&mut buf);
            sock.write_all(&vec).unwrap();
            sock.flush().unwrap();
        }
    }
}

// Services the streams
fn socket_stream_thread(mut sock: UdpStream, state: Arc<Mutex<State>>) {
    let mut buf = vec![0; 1024];
    loop {
        std::thread::sleep(Duration::from_millis(10));
        let state = state.lock().unwrap();

        let mut nf_markers: Vec<_> = state.calculate_nf_positions();
        sort_rectangle(&mut nf_markers);
        let nf_markers = nf_markers;
        let wf_markers: Vec<_> = state.calculate_wf_positions();

        if let Some(id) = state.stream_mot {
            let mut object_report = ObjectReport::default();
            for (marker, mot_data) in nf_markers.iter().zip(&mut object_report.mot_data_nf) {
                if (0..4095).contains(&marker.x) && (0..4095).contains(&marker.y) {
                    mot_data.area = 1;
                    mot_data.cx = marker.x as u16;
                    mot_data.cy = marker.y as u16;
                    mot_data.boundary_left =
                        ((marker.x as f64 / 4094. * 98.0) as u8).saturating_sub(1);
                    mot_data.boundary_right = mot_data.boundary_left + 3;
                    mot_data.boundary_up =
                        ((marker.y as f64 / 4094. * 98.0) as u8).saturating_sub(1);
                    mot_data.boundary_down = mot_data.boundary_up + 3;
                }
            }
            for (marker, mot_data) in wf_markers.iter().zip(&mut object_report.mot_data_wf) {
                if (0..4096).contains(&marker.x) && (0..4096).contains(&marker.y) {
                    mot_data.area = 1;
                    mot_data.cx = marker.x as u16;
                    mot_data.cy = marker.y as u16;
                    mot_data.boundary_left =
                        ((marker.x as f64 / 4094. * 98.0) as u8).saturating_sub(1);
                    mot_data.boundary_right = mot_data.boundary_left + 3;
                    mot_data.boundary_up =
                        ((marker.y as f64 / 4094. * 98.0) as u8).saturating_sub(1);
                    mot_data.boundary_down = mot_data.boundary_up + 3;
                }
            }
            buf.clear();
            Packet {
                id,
                data: PacketData::ObjectReport(object_report),
            }
            .serialize(&mut buf);
            let vec = cobs::encode_vec(&mut buf);
            sock.write_all(&vec).unwrap();
            sock.flush().unwrap();
        }

        if let Some(id) = state.stream_combined_markers {
            // let marker_pattern = state.marker_pattern.marker_positions();
            // let r = transform_aim_point(
            //     [2048.0, 2048.0].into(),
            //     nf_markers[0].cast(),
            //     nf_markers[1].cast(),
            //     nf_markers[2].cast(),
            //     nf_markers[3].cast(),
            //     marker_pattern[0],
            //     marker_pattern[1],
            //     marker_pattern[2],
            //     marker_pattern[3],
            // ).unwrap();
            let make_positions = |markers: &[Point2<i16>]| {
                let positions = std::array::from_fn(|i| {
                    let Some(XY { x, y }) = markers.get(i).map(|x| **x) else {
                        return Point2::default();
                    };
                    if (0..4096).contains(&x) && (0..4096).contains(&y) {
                        Point2::new(x as u16, y as u16)
                    } else {
                        Point2::default()
                    }
                });
                positions
            };
            let nf_positions = make_positions(&nf_markers);
            let wf_positions = make_positions(&wf_markers);
            let pkt = Packet {
                id,
                data: PacketData::CombinedMarkersReport(CombinedMarkersReport {
                    nf_points: nf_positions,
                    wf_points: wf_positions,
                }),
            };
            buf.clear();
            pkt.serialize(&mut buf);
            let vec = cobs::encode_vec(&mut buf);
            sock.write_all(&vec).unwrap();
            sock.flush().unwrap();
        }
    }
}

struct State {
    camera_pos: Point3<f64>,
    yaw: f64,
    pitch: f64,
    markers: Vec<Point3<f64>>,
    prev_mouse: Point2<f64>,
    moving_fwd: bool,
    moving_back: bool,
    moving_left: bool,
    moving_right: bool,
    moving_up: bool,
    moving_down: bool,
    stream_mot: Option<u8>,
    stream_combined_markers: Option<u8>,
    // marker_pattern: MarkerPattern,
}

impl State {
    fn new() -> Self {
        let tf = Scale3::new(GRID_WIDTH, GRID_HEIGHT, 1.0).to_homogeneous()
            * Translation3::new(-0.5, -0.5, 0.0).to_homogeneous();
        let tf = Transform3::from_matrix_unchecked(tf);
        Self {
            camera_pos: DEFAULT_CAMERA_POS,
            yaw: 0.0,
            pitch: 0.0,
            markers: vec![
                // tf * Point3::new(0.0, 0.5, 0.0),
                // tf * Point3::new(0.35, 0.2, 0.0),
                // tf * Point3::new(0.65, 0.2, 0.0),
                // tf * Point3::new(0.35, 0.9, 0.0),
                // tf * Point3::new(0.65, 0.9, 0.0),
                // tf * Point3::new(1.0, 0.5, 0.0),
                tf * Point3::new(0.18, 1.0 - 0.29, 0.),
                tf * Point3::new(0.15, 1.0 - 0.82, 0.),
                tf * Point3::new(0.77, 1.0 - 0.8, 0.),
                tf * Point3::new(0.51, 1.0 - 0.35, 0.),
                tf * Point3::new(0.79, 1.0 - 0.25, 0.),
                tf * Point3::new(0.49, 1.0 - 0.76, 0.),
                tf * Point3::new(1.18, 1.0 - 0.29, 0.),
                tf * Point3::new(1.15, 1.0 - 0.82, 0.),
                tf * Point3::new(1.77, 1.0 - 0.8, 0.),
                tf * Point3::new(1.51, 1.0 - 0.35, 0.),
                tf * Point3::new(1.79, 1.0 - 0.25, 0.),
                tf * Point3::new(1.49, 1.0 - 0.76, 0.),
                // tf * Point3::new(0.35, 1.0, 0.0),
                // tf * Point3::new(0.65, 1.0, 0.0),
                // tf * Point3::new(0.65, 0.0, 0.0),
                // tf * Point3::new(0.35, 0.0, 0.0),

                // tf * Point3::new(GRID_WIDTH + 0.35, 1.0, 0.0),
                // tf * Point3::new(GRID_WIDTH + 0.65, 1.0, 0.0),
                // tf * Point3::new(GRID_WIDTH + 0.65, 0.0, 0.0),
                // tf * Point3::new(GRID_WIDTH + 0.35, 0.0, 0.0),
                // tf * Point3::new(GRID_WIDTH + 0.35, -0.2, 0.0),
                // tf * Point3::new(GRID_WIDTH + 0.65, -0.2, 0.0),

                // Point3::from([38.4-192./2.,  3.84-108./2.,   0.0).unwrap()]),
                // Point3::from([38.4-192./2.,  105.84-108./2., 0.0).unwrap()]),
                // Point3::from([153.6-192./2., 105.84-108./2., 0.0).unwrap()]),
                // Point3::from([96.0-192./2.,  3.84-108./2.,   0.0).unwrap()]),
                // Point3::from([153.6-192./2., 3.84-108./2.,   0.0).unwrap()]),
                // Point3::from([96.0-192./2.,  105.84-108./2., 0.0).unwrap()]),
            ],
            prev_mouse: Point2::new(0., 0.),
            moving_fwd: false,
            moving_back: false,
            moving_left: false,
            moving_right: false,
            moving_up: false,
            moving_down: false,
            stream_mot: None,
            stream_combined_markers: None,
            // marker_pattern: MarkerPattern::Rectangle,
        }
    }

    fn camera_matrix(&self) -> Matrix4<f64> {
        let mut translate = Matrix4::identity();
        translate.m14 = self.camera_pos.x;
        translate.m24 = self.camera_pos.y;
        translate.m34 = self.camera_pos.z;
        let yaw = Rotation3::from_axis_angle(&Vector3::y_axis(), self.yaw);
        let pitch = Rotation3::from_axis_angle(&Vector3::x_axis(), self.pitch);
        let mut rot = Matrix4::identity();
        rot.get_mut((0..3, 0..3))
            .unwrap()
            .copy_from((yaw * pitch).matrix());
        let camera_matrix = translate * rot;
        camera_matrix.try_inverse().unwrap()
    }

    fn camera_rotation(&self) -> Rotation3<f64> {
        let yaw = Rotation3::from_axis_angle(&Vector3::y_axis(), self.yaw);
        let pitch = Rotation3::from_axis_angle(&Vector3::x_axis(), self.pitch);
        yaw * pitch
    }

    fn nf_perspective_matrix(&self) -> Matrix4<f64> {
        let f = nf_focal_length();
        #[rustfmt::skip]
        let r = Matrix4::new(
            f, 0.,  0., 0.,
            0., f,  0., 0.,
            0., 0., 1., 0.,
            0., 0.,-1., 0.,
        );
        r
    }

    fn wf_perspective_matrix(&self) -> Matrix4<f64> {
        let f = wf_focal_length();
        #[rustfmt::skip]
        let r = Matrix4::new(
            f, 0.,  0., 0.,
            0., f,  0., 0.,
            0., 0., 1., 0.,
            0., 0.,-1., 0.,
        );
        r
    }

    /// Ratio of the image plane size of nf to wf.
    fn nf_wf_ratio(&self) -> f64 {
        let nf_fov_deg = 38.3;
        let nf_fov = nf_fov_deg / 180.0 * std::f64::consts::PI;
        let nf_s = (nf_fov / 2.).tan();

        let wf_fov_deg = 111.3;
        let nf_fov = wf_fov_deg / 180.0 * std::f64::consts::PI;
        let wf_s = (nf_fov / 2.).tan();

        nf_s / wf_s
    }

    fn calculate_nf_positions(&self) -> Vec<Point2<i16>> {
        let transform = self.nf_perspective_matrix() * self.camera_matrix();
        let device_transform = Scale2::new(2047.5, -2047.5).to_homogeneous()
            * Translation2::new(1.0, -1.0).to_homogeneous();
        self.markers
            .iter()
            .map(|p| {
                let p = transform * Vector4::new(p.x, p.y, p.z, 1.0);
                let p = device_transform.transform_point(&(p.xy() / p.w).into());
                Point2::new(p.x.round() as i16, p.y.round() as i16)
            })
            .collect()
    }

    fn calculate_wf_positions(&self) -> Vec<Point2<i16>> {
        let transform = self.wf_perspective_matrix() * self.camera_matrix();
        let device_transform = Scale2::new(2047.5, -2047.5).to_homogeneous()
            * Translation2::new(1.0, -1.0).to_homogeneous();
        self.markers
            .iter()
            .map(|p| {
                let p = transform * Vector4::new(p.x, p.y, p.z, 1.0);
                let p = device_transform.transform_point(&(p.xy() / p.w).into());
                Point2::new(p.x.round() as i16, p.y.round() as i16)
            })
            .collect()
    }
}

struct MainCanvas {
    state: Arc<Mutex<State>>,
}

// const GRID_WIDTH: f64 = 16.0 / 9.0;
// const GRID_HEIGHT: f64 = 1.0;
const GRID_WIDTH: f64 = 3.64631 * 100.;
const GRID_HEIGHT: f64 = 2.05105 * 100.;
// const DEFAULT_CAMERA_POS: Point3<f64> = Point3::new(0.0, 0.0, 360.0);
const DEFAULT_CAMERA_POS: Point3<f64> = Point3::new(0.0, 0.0, 4.99872 * 100.);

impl AreaHandler for MainCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
        let thin_line = StrokeParams {
            cap: 0,  // Bevel
            join: 0, // Flat
            thickness: 1.,
            miter_limit: 0.,
            dashes: vec![],
            dash_phase: 0.,
        };

        let ctx = &draw_params.context;
        let mut state = self.state.lock().unwrap();

        #[rustfmt::skip] {
            let movement_speed = 0.5;
            let fwd = state.camera_rotation() * -Vector3::z();
            let right = state.camera_rotation() * Vector3::x();
            let up = state.camera_rotation() * Vector3::y();
            if state.moving_fwd { state.camera_pos += fwd * movement_speed }
            if state.moving_back { state.camera_pos -= fwd * movement_speed; }
            if state.moving_left { state.camera_pos -= right * movement_speed; }
            if state.moving_right { state.camera_pos += right * movement_speed; }
            if state.moving_up { state.camera_pos += up * movement_speed; }
            if state.moving_down { state.camera_pos -= up * movement_speed; }
        };

        // we have opengl at home
        let w = draw_params.area_width;
        let h = draw_params.area_height;
        let size = w.min(h);

        // black square background
        let bg = Path::new(ctx, FillMode::Winding);
        bg.new_figure(ctx, w / 2. - size / 2., h / 2. - size / 2.);
        bg.line_to(ctx, w / 2. + size / 2., h / 2. - size / 2.);
        bg.line_to(ctx, w / 2. + size / 2., h / 2. + size / 2.);
        bg.line_to(ctx, w / 2. - size / 2., h / 2. + size / 2.);
        bg.end(ctx);
        ctx.fill(
            &bg,
            &Brush::Solid(SolidBrush {
                r: 0.,
                g: 0.,
                b: 0.,
                a: 1.,
            }),
        );

        {
            // yellow square for nearfield
            let sq = Path::new(ctx, FillMode::Winding);
            let size = size * state.nf_wf_ratio();
            sq.new_figure(ctx, w / 2. - size / 2., h / 2. - size / 2.);
            sq.line_to(ctx, w / 2. + size / 2., h / 2. - size / 2.);
            sq.line_to(ctx, w / 2. + size / 2., h / 2. + size / 2.);
            sq.line_to(ctx, w / 2. - size / 2., h / 2. + size / 2.);
            sq.close_figure(ctx);
            sq.end(ctx);
            ctx.stroke(
                &sq,
                &Brush::Solid(SolidBrush {
                    r: 1.,
                    g: 1.,
                    b: 0.,
                    a: 1.,
                }),
                &thin_line,
            );
        }

        // map normalized coordinates into screen coordinates
        let device_transform = Translation2::new(w / 2., h / 2.).to_homogeneous()
            * Scale2::new(size / 2., -size / 2.).to_homogeneous();
        let transform = state.wf_perspective_matrix() * state.camera_matrix();

        // Draw the grid at the origin
        let grid_path = Path::new(ctx, FillMode::Winding);
        draw_grid(
            ctx,
            &grid_path,
            10,
            10,
            transform
                * Scale3::new(GRID_WIDTH, GRID_HEIGHT, 1.).to_homogeneous()
                * Translation3::new(-0.5, -0.5, 0.).to_homogeneous(),
            device_transform,
        );
        grid_path.end(ctx);
        ctx.stroke(
            &grid_path,
            &Brush::Solid(SolidBrush {
                r: 1.,
                g: 0.,
                b: 0.,
                a: 1.,
            }),
            &thin_line,
        );

        let markers_path = Path::new(ctx, FillMode::Winding);
        for &marker in &state.markers {
            draw_marker(
                marker,
                0.05,
                ctx,
                &markers_path,
                transform,
                device_transform,
            );
        }
        markers_path.end(ctx);
        ctx.stroke(
            &markers_path,
            &Brush::Solid(SolidBrush {
                r: 0.,
                g: 1.,
                b: 0.,
                a: 1.,
            }),
            &thin_line,
        );
        let brush = Brush::Solid(SolidBrush {
            r: 1.,
            g: 1.,
            b: 1.,
            a: 1.,
        });
        let center_point_path = Path::new(ctx, FillMode::Winding);
        draw_diamond(
            ctx,
            &center_point_path,
            0.5 * draw_params.area_width,
            0.5 * draw_params.area_height,
            8.0,
            8.0,
        );
        center_point_path.end(ctx);
        ctx.stroke(&center_point_path, &brush, &thin_line);
    }

    fn mouse_event(&mut self, _area: &Area, mouse_event: &AreaMouseEvent) {
        let mut state = self.state.lock().unwrap();
        let pos = Point2::new(mouse_event.x, mouse_event.y);
        let d = pos - state.prev_mouse;
        state.prev_mouse = pos;

        if mouse_event.held_1_to_64 & 1 != 0 {
            state.yaw += d.x / 5000.0;
            state.pitch += d.y / 5000.0;
        }
        if mouse_event.down == 2 {
            dbg!(state.calculate_nf_positions());
        }
    }

    fn key_event(&mut self, _area: &Area, key_event: &AreaKeyEvent) -> bool {
        let mut state = self.state.lock().unwrap();
        state.moving_down = key_event.modifiers.contains(Modifiers::MODIFIER_SHIFT)
            || key_event.modifiers.contains(Modifiers::MODIFIER_CTRL);
        let field = match key_event.key {
            b'w' => &mut state.moving_fwd,
            b'a' => &mut state.moving_left,
            b's' => &mut state.moving_back,
            b'd' => &mut state.moving_right,
            b'e' | b' ' => &mut state.moving_up,
            b'q' => &mut state.moving_down,
            b'\x08' => {
                // backspace
                // reset the camera
                state.camera_pos = DEFAULT_CAMERA_POS;
                state.yaw = 0.0;
                state.pitch = 0.0;
                return true;
            }
            _ if key_event.modifier.contains(Modifiers::MODIFIER_SHIFT)
                || key_event.modifier.contains(Modifiers::MODIFIER_CTRL) =>
            {
                &mut state.moving_down
            }
            _ => return true,
        };
        *field = !key_event.up;
        true
    }
}

fn draw_line(
    ctx: &DrawContext,
    path: &Path,
    p1: Point3<f64>,
    p2: Point3<f64>,
    transform: Matrix4<f64>,
    device_transform: Matrix3<f64>,
) {
    let p1 = Vector4::new(p1.x, p1.y, p1.z, 1.0);
    let p2 = Vector4::new(p2.x, p2.y, p2.z, 1.0);
    let p1 = transform * p1;
    let p2 = transform * p2;

    let Some((p1, p2)) = clip_line(p1.into(), p2.into(), |p| p.z, ..-0.1) else {
        return;
    };
    let p1 = device_transform.transform_point(&(p1.xy() / p1.w));
    let p2 = device_transform.transform_point(&(p2.xy() / p2.w));
    path.new_figure(ctx, p1.x, p1.y);
    path.line_to(ctx, p2.x, p2.y);
}

fn draw_marker(
    p: Point3<f64>,
    size: f64,
    ctx: &DrawContext,
    path: &Path,
    transform: Matrix4<f64>,
    device_transform: Matrix3<f64>,
) {
    for i in 0..48 {
        let a1 = i as f64 / 24.0 * PI;
        let a2 = (i + 1) as f64 / 24.0 * PI;
        let p1 = p + Vector3::new(size * a1.cos(), size * a1.sin(), 0.0);
        let p2 = p + Vector3::new(size * a2.cos(), size * a2.sin(), 0.0);
        draw_line(ctx, path, p1, p2, transform, device_transform);
    }
    let p1 = p + Vector3::new(size * FRAC_1_SQRT_2, size * FRAC_1_SQRT_2, 0.0);
    let p2 = p + Vector3::new(-size * FRAC_1_SQRT_2, -size * FRAC_1_SQRT_2, 0.0);
    draw_line(ctx, path, p1, p2, transform, device_transform);
    let p1 = p + Vector3::new(-size * FRAC_1_SQRT_2, size * FRAC_1_SQRT_2, 0.0);
    let p2 = p + Vector3::new(size * FRAC_1_SQRT_2, -size * FRAC_1_SQRT_2, 0.0);
    draw_line(ctx, path, p1, p2, transform, device_transform);
}

/// Draw a grid on the xy plane between 0 and 1, transformed by `transform`.
fn draw_grid(
    ctx: &DrawContext,
    path: &Path,
    x_subdiv: usize,
    y_subdiv: usize,
    transform: Matrix4<f64>,
    device_transform: Matrix3<f64>,
) {
    for y in 0..=y_subdiv {
        let p1 = Point3::new(0.0, y as f64 / y_subdiv as f64, 0.0);
        let p2 = Point3::new(1.0, y as f64 / y_subdiv as f64, 0.0);
        draw_line(ctx, path, p1, p2, transform, device_transform);
    }

    for x in 0..=x_subdiv {
        let p1 = Point3::new(x as f64 / x_subdiv as f64, 0.0, 0.0);
        let p2 = Point3::new(x as f64 / x_subdiv as f64, 1.0, 0.0);
        draw_line(ctx, path, p1, p2, transform, device_transform);
    }
}

fn clip_line<const N: usize>(
    mut p1: Point<f64, N>,
    mut p2: Point<f64, N>,
    coord: impl Fn(SVector<f64, N>) -> f64,
    range: impl RangeBounds<f64>,
) -> Option<(Point<f64, N>, Point<f64, N>)> {
    let c1 = coord(p1.coords);
    let c2 = coord(p2.coords);
    let d = p2 - p1;
    let cd = coord(d);
    if let Bound::Included(&b) | Bound::Excluded(&b) = range.end_bound() {
        'a: {
            if c1 > b && c2 > b {
                return None;
            }
            if c1 <= b && c2 <= b {
                break 'a;
            }
            if c1 > b {
                p1 += d * ((b - c1) / cd);
            }
            if c2 > b {
                p2 += d * ((b - c2) / cd);
            }
        }
    }
    if let Bound::Included(&b) | Bound::Excluded(&b) = range.start_bound() {
        if c1 < b && c2 < b {
            return None;
        }
        if c1 >= b && c2 >= b {
            return Some((p1, p2));
        }
        if c1 < b {
            p1 += d * ((b - c1) / cd);
        }
        if c2 < b {
            p2 += d * ((b - c2) / cd);
        }
    }
    Some((p1, p2))
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
