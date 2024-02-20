use std::{f64::consts::{FRAC_1_SQRT_2, PI}, io::{Read, Write}, net::{TcpListener, TcpStream}, ops::{Bound, RangeBounds}, sync::{Arc, Mutex}, time::Duration};

use ats_cv::transform_aim_point;
use iui::{
    controls::{
        Area, AreaDrawParams, AreaHandler, AreaKeyEvent, AreaMouseEvent, Modifiers, Window,
        WindowType,
    },
    draw::{Brush, DrawContext, FillMode, Path, SolidBrush, StrokeParams},
    UI,
};
use nalgebra::{
    Matrix3, Matrix4, Point, Point2, Point3, Rotation3, SVector, Scale2, Scale3, Translation2, Translation3, Vector3, Vector4
};
use tracing::{error, info};
use vision_module_gui::{custom_shapes::draw_diamond, mot_runner::sort_rectangle, packet::{AimPointReport, GeneralConfig, MarkerPattern, ObjectReport, Packet, PacketData, ReadRegisterResponse}};

fn main() {
    let mut port = 4444u16;
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
            Stretchy : let area = Area(Box::new(MainCanvas { state: state.clone() }))
        }
    }
    main_win.set_child(&ui, vbox);
    main_win.show(&ui);
    ui.ui_timer(16, {
        let ui = ui.clone();
        let area = area.clone();
        move || {
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
                if s.mask & 0b001 != 0 {
                    state.stream_mot = if s.active { Some(pkt.id) } else { None };
                }
                if s.mask & 0b010 != 0 {
                    state.stream_aim = if s.active { Some(pkt.id) } else { None };
                }
                None
            }
            PacketData::FlashSettings => None,
            PacketData::AimPointReport(_) => unreachable!(),
            PacketData::ImpactWithAimPointReport(_) => unreachable!(),
            PacketData::WriteConfig(_) => None,
            PacketData::ReadConfig => Some(PacketData::ReadConfigResponse(GeneralConfig { impact_threshold: 0, marker_pattern: MarkerPattern::Rectangle })),
            PacketData::ReadConfigResponse(_) => unreachable!(),
        };

        if let Some(data) = response {
            buf.clear();
            Packet { id: pkt.id, data }.serialize(&mut buf);
            sock.write_all(&buf).unwrap();
        }
    }
}

// Services the streams
fn socket_stream_thread(mut sock: TcpStream, state: Arc<Mutex<State>>) {
    let mut buf = vec![0; 1024];
    loop {
        std::thread::sleep(Duration::from_millis(10));
        let state = state.lock().unwrap();

        let mut markers = state.calculate_marker_positions();
        let mut unsorted = [markers.bottom, markers.left, markers.top, markers.right];
        sort_rectangle(&mut unsorted);
        markers.bottom = unsorted[0];
        markers.left = unsorted[1];
        markers.top = unsorted[2];
        markers.right = unsorted[3];
        let markers = markers;

        if let Some(id) = state.stream_mot {
            let mut object_report = ObjectReport::default();
            if (0..4096).contains(&markers.top.x) && (0..4096).contains(&markers.top.y) {
                object_report.mot_data_nf[0].area = 1;
                object_report.mot_data_nf[0].cx = markers.top.x as u16;
                object_report.mot_data_nf[0].cy = markers.top.y as u16;
            }
            if (0..4096).contains(&markers.left.x) && (0..4096).contains(&markers.left.y) {
                object_report.mot_data_nf[1].area = 1;
                object_report.mot_data_nf[1].cx = markers.left.x as u16;
                object_report.mot_data_nf[1].cy = markers.left.y as u16;
            }
            if (0..4096).contains(&markers.bottom.x) && (0..4096).contains(&markers.bottom.y) {
                object_report.mot_data_nf[2].area = 1;
                object_report.mot_data_nf[2].cx = markers.bottom.x as u16;
                object_report.mot_data_nf[2].cy = markers.bottom.y as u16;
            }
            if (0..4096).contains(&markers.right.x) && (0..4096).contains(&markers.right.y) {
                object_report.mot_data_nf[3].area = 1;
                object_report.mot_data_nf[3].cx = markers.right.x as u16;
                object_report.mot_data_nf[3].cy = markers.right.y as u16;
            }
            buf.clear();
            Packet { id, data: PacketData::ObjectReport(object_report) }.serialize(&mut buf);
            sock.write_all(&buf).unwrap();
        }

        if let Some(id) = state.stream_aim {
            let r = transform_aim_point(
                [2048.0, 2048.0].into(),
                markers.top.cast(),
                markers.bottom.cast(),
                markers.left.cast(),
                markers.right.cast(),
                [0.5, 0.0].into(),
                [0.5, 1.0].into(),
                [0.0, 0.5].into(),
                [1.0, 0.5].into(),
            ).unwrap();
            let pkt = Packet {
                id,
                data: PacketData::AimPointReport(AimPointReport {
                    x: (r.x * 4095. - 2047.) as i16,
                    y: (r.y * 4095. - 2047.) as i16,
                })
            };
            buf.clear();
            pkt.serialize(&mut buf);
            sock.write_all(&buf).unwrap();
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct MarkerPoints {
    top: Point2<i16>,
    right: Point2<i16>,
    bottom: Point2<i16>,
    left: Point2<i16>,
}

struct State {
    camera_pos: Point3<f64>,
    yaw: f64,
    pitch: f64,
    marker_offset: MarkerPoints,
    fov: f64,
    prev_mouse: Point2<f64>,
    moving_fwd: bool,
    moving_back: bool,
    moving_left: bool,
    moving_right: bool,
    moving_up: bool,
    moving_down: bool,
    stream_mot: Option<u8>,
    stream_aim: Option<u8>,
}

impl State {
    fn new() -> Self {
        Self {
            camera_pos: Point3::new(0., 0., 10.),
            yaw: 0.0,
            pitch: 0.0,
            marker_offset: MarkerPoints {
                top: Point2::new(-500, -2047),
                right: Point2::new(500, -2047),
                bottom: Point2::new(-500, 2047),
                left: Point2::new(500, 2047),

                // top: Point2::new(-1000, -2047),
                // right: Point2::new(2047, 1000),
                // bottom: Point2::new(1000, 2047),
                // left: Point2::new(-2047, -1000),

                // top: Point2::new(0, -2047),
                // right: Point2::new(2047, 0),
                // bottom: Point2::new(0, 2047),
                // left: Point2::new(-2047, 0),
            },
            fov: 38.3 / 180.0 * std::f64::consts::PI,
            prev_mouse: Point2::new(0., 0.),
            moving_fwd: false,
            moving_back: false,
            moving_left: false,
            moving_right: false,
            moving_up: false,
            moving_down: false,
            stream_mot: None,
            stream_aim: None,
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

    fn perspective_matrix(&self) -> Matrix4<f64> {
        let f = 1. / (self.fov / 2.).tan();
        #[rustfmt::skip]
        let r = Matrix4::new(
            f, 0.,  0., 0.,
            0., f,  0., 0.,
            0., 0., 1., 0.,
            0., 0.,-1., 0.,
        );
        r
    }

    fn calculate_marker_positions(&self) -> MarkerPoints {
        let transform = self.perspective_matrix()
            * self.camera_matrix()
            * Scale3::new(GRID_WIDTH / 2.0 / 2047.0, GRID_HEIGHT / 2.0 / 2047.0, 1.0).to_homogeneous();
        let device_transform =
            Scale2::new(2047.5, -2047.5).to_homogeneous() * Translation2::new(1.0, -1.0).to_homogeneous();
        let off = self.marker_offset;
        let top = transform * Vector4::new(off.top.x, -off.top.y, 0, 1).cast::<f64>();
        let left = transform * Vector4::new(off.left.x, -off.left.y, 0, 1).cast::<f64>();
        let bottom = transform * Vector4::new(off.bottom.x, -off.bottom.y, 0, 1).cast::<f64>();
        let right = transform * Vector4::new(off.right.x, -off.right.y, 0, 1).cast::<f64>();

        let top = device_transform.transform_point(&(top.xy() / top.w).into());
        let left = device_transform.transform_point(&(left.xy() / left.w).into());
        let bottom = device_transform.transform_point(&(bottom.xy() / bottom.w).into());
        let right = device_transform.transform_point(&(right.xy() / right.w).into());

        let top = Point2::new(top.x.round() as i16, top.y.round() as i16);
        let left = Point2::new(left.x.round() as i16, left.y.round() as i16);
        let bottom = Point2::new(bottom.x.round() as i16, bottom.y.round() as i16);
        let right = Point2::new(right.x.round() as i16, right.y.round() as i16);

        MarkerPoints { top, left, bottom, right }
    }
}

struct MainCanvas {
    state: Arc<Mutex<State>>,
}

const GRID_WIDTH: f64 = 16.0 / 9.0;
const GRID_HEIGHT: f64 = 1.0;

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
            let movement_speed = 0.05;
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
        bg.new_figure(ctx, w/2. - size/2., h/2. - size/2.);
        bg.line_to(ctx, w/2. + size/2., h/2. - size/2.);
        bg.line_to(ctx, w/2. + size/2., h/2. + size/2.);
        bg.line_to(ctx, w/2. - size/2., h/2. + size/2.);
        bg.end(ctx);
        ctx.fill(&bg, &Brush::Solid(SolidBrush { r: 0., g: 0., b: 0., a: 1. }));

        // map normalized coordinates into screen coordinates
        let device_transform = Translation2::new(w / 2., h / 2.).to_homogeneous()
            * Scale2::new(size / 2., -size / 2.).to_homogeneous();
        let transform = state.perspective_matrix() * state.camera_matrix();

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
            &Brush::Solid(SolidBrush { r: 1., g: 0., b: 0., a: 1. }),
            &thin_line,
        );

        let markers_path = Path::new(ctx, FillMode::Winding);
        for marker in [
            state.marker_offset.top,
            state.marker_offset.bottom,
            state.marker_offset.left,
            state.marker_offset.right,
        ] {
            let p = Scale3::new(GRID_WIDTH / 2.0, GRID_HEIGHT / 2.0, 0.0)
                * (Point3::new(marker.x, -marker.y, 0).cast::<f64>() / 2047.0);
            draw_marker(p, 0.05, ctx, &markers_path, transform, device_transform);
        }
        markers_path.end(ctx);
        ctx.stroke(
            &markers_path,
            &Brush::Solid(SolidBrush { r: 0., g: 1., b: 0., a: 1. }),
            &thin_line,
        );
        let brush = Brush::Solid(SolidBrush { r: 1., g: 1., b: 1., a: 1. });
        let center_point_path = Path::new(ctx, FillMode::Winding);
        draw_diamond(ctx, &center_point_path, 0.5 * draw_params.area_width, 0.5 * draw_params.area_height, 8.0, 8.0);
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
            dbg!(state.calculate_marker_positions());
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
            _ if key_event.modifier.contains(Modifiers::MODIFIER_SHIFT)
            || key_event.modifier.contains(Modifiers::MODIFIER_CTRL) => &mut state.moving_down,
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

    let Some((p1, p2)) = clip_line(p1.into(), p2.into(), |p| p.z, ..-0.1) else { return };
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
        let a2 = (i+1) as f64 / 24.0 * PI;
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
