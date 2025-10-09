use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::Result;
use app_dirs2::{get_app_root, AppDataType};
use ats_usb::packets::vm::{GeneralConfig, Serialize as _};
use iui::controls::{Area, FileTypeFilter, HorizontalBox};
use iui::prelude::*;
use leptos_reactive::{
    create_effect, RwSignal, SignalGet, SignalGetUntracked, SignalSet, SignalWith,
};
use parking_lot::Mutex;
use tokio::task::AbortHandle;
use tracing::{error, info, warn, Level};
use tracing_subscriber::EnvFilter;
use vision_module_gui::mot_runner::MotRunner;
use vision_module_gui::run_canvas::RunCanvas;
use vision_module_gui::run_raw_canvas::RunRawCanvas;
use vision_module_gui::test_canvas::TestCanvas;
use vision_module_gui::{config_window, plots_window, TestFrame};
use vision_module_gui::{CloneButShorter, MotState};
#[cfg(feature = "bevy")]
use {
    bevy::{
        app::{App, Startup, Update},
        ecs::system::{Commands, Query},
        prelude::*,
        window::{PresentMode, WindowCloseRequested, WindowPlugin, WindowTheme},
        winit::WinitPlugin,
        DefaultPlugins,
    },
    bevy_atmosphere::plugin::{AtmosphereCamera, AtmospherePlugin},
    bevy_infinite_grid::{
        GridShadowCamera, InfiniteGridBundle, InfiniteGridPlugin, InfiniteGridSettings,
    },
};

use vision_module_gui::consts::APP_INFO;

use ats_common::ScreenCalibration;

// Things to avoid doing
// * Accessing signals outside of the main thread
//     * Getting panics, setting fails silently
//     * by extension, don't access signals from tokio::spawn, use ui.spawn
// * Excessive blocking in the main thread or in callbacks
//     * Freezes the GUI
// * Creating libui uiControls dynamically
//     * They are leaky
// * Holding mutexes and setting signals
//     * Drop the mutex, or use ui_ctx.queue_main()

#[cfg(feature = "bevy")]
#[derive(Resource)]
struct MotRunnerResource(Arc<Mutex<MotRunner>>);

#[cfg(feature = "bevy")]
enum Event {
    OpenWindow,
}

#[cfg(feature = "bevy")]
#[derive(Resource, Deref)]
struct StreamReceiver(crossbeam::channel::Receiver<Event>);

#[cfg(feature = "bevy")]
#[derive(Event)]
struct StreamEvent(Event);

#[cfg(feature = "bevy")]
fn read_stream(receiver: Res<StreamReceiver>, mut events: EventWriter<StreamEvent>) {
    for from_stream in receiver.try_iter() {
        events.send(StreamEvent(from_stream));
    }
}

#[cfg(feature = "bevy")]
fn show_window(
    mut commands: Commands,
    mut reader: EventReader<StreamEvent>,
    mut window: Query<&mut bevy::window::Window>,
) {
    for (_, event) in reader.read().enumerate() {
        match event.0 {
            Event::OpenWindow => {
                if window.iter().len() == 0 {
                    commands.spawn(create_bevy_window(true));
                } else {
                    window.single_mut().visible = true;
                }
            }
        }
    }
}

#[cfg(feature = "bevy")]
pub fn hide_instead_of_close(
    mut closed: EventReader<WindowCloseRequested>,
    mut window: Query<&mut bevy::window::Window>,
) {
    for event in closed.read() {
        if let Ok(mut window) = window.get_mut(event.window) {
            window.visible = false;
        }
    }
}

#[cfg(feature = "bevy")]
fn create_bevy_window(visible: bool) -> bevy::window::Window {
    bevy::window::Window {
        title: "I am a window!".into(),
        name: Some("bevy.app".into()),
        resolution: (500., 300.).into(),
        present_mode: PresentMode::AutoVsync,
        // Tells wasm not to override default event handling, like F5, Ctrl+R etc.
        prevent_default_event_handling: false,
        window_theme: Some(WindowTheme::Dark),
        enabled_buttons: bevy::window::EnabledButtons {
            maximize: false,
            ..Default::default()
        },
        // This will spawn an invisible window
        // This is useful when you want to avoid the white window that shows up before the GPU is ready to render the app.
        visible,
        ..default()
    }
}

#[cfg(feature = "bevy")]
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Spawn a cube to rotate.
    commands.spawn((PbrBundle {
        mesh: meshes.add(Cuboid::default()),
        material: materials.add(Color::WHITE),
        transform: Transform::from_translation(Vec3::ZERO),
        ..default()
    },));

    commands.spawn(InfiniteGridBundle {
        settings: InfiniteGridSettings {
            // shadow_color: None,
            ..default()
        },
        ..default()
    });

    // Spawn a camera looking at the entities to show what's happening in this example.
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 10.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        GridShadowCamera,
        AtmosphereCamera::default(),
    ));

    // Add a light source so we can see clearly.
    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_xyz(3.0, 3.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}

// This system will rotate any entity in the scene with a Rotatable component around its y-axis.
#[cfg(feature = "bevy")]
fn rotate_cube(mut camera_query: Query<(&Camera, &mut Transform)>, runner: Res<MotRunnerResource>) {
    let runner = runner.0.lock();
    let rotation = runner.state.rotation_mat;
    let translation = runner.state.translation_mat;

    let offset = Vec3::new(0.0, 1.6, 10.);

    let (_camera, mut camera_transform) = camera_query.single_mut();
    camera_transform.translation = Vec3::new(
        translation.x as f32,
        translation.y as f32,
        translation.z as f32,
    ) + offset;
    let rotation = Mat3::from_cols_slice(rotation.cast().as_slice());
    camera_transform.rotation = Quat::from_mat3(&rotation);
}

fn get_screens_dir() -> Option<PathBuf> {
    if let Some(env_override) = std::env::var_os("SCREEN_CALIBRATIONS_DIR") {
        return Some(env_override.into());
    }

    let mut app_cfg_root = match get_app_root(AppDataType::UserConfig, &APP_INFO) {
        Ok(d) => d,
        Err(e) => {
            warn!("Failed to load app config: {e}");
            return None;
        }
    };
    app_cfg_root.push("screens");
    Some(app_cfg_root)
}

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::builder()
                .with_env_var("RUST_LOG")
                .with_default_directive(Level::INFO.into())
                .from_env_lossy(),
        )
        .init();
    let tokio_rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap();
    let tokio_handle = tokio_rt.handle();
    let _enter = tokio_handle.enter();
    let leptos_rt = leptos_reactive::create_runtime();
    // Initialize the UI library
    let ui = UI::init().expect("Couldn't initialize UI library");
    let ui_ctx = ui.async_context();

    let mut simulator_addr = None;
    let mut udp_addr = None;
    match &std::env::args().collect::<Vec<_>>()[1..] {
        [flag, addr] if flag == "-u" => udp_addr = Some(addr.clone()),
        [addr] => simulator_addr = Some(addr.clone()),
        [] => (),
        _ => panic!("Unrecognized arguments"),
    }
    let datapoints: Arc<Mutex<Vec<TestFrame>>> = Arc::new(Mutex::new(Vec::new()));
    let packets = Arc::new(Mutex::new(Vec::new()));
    let state = MotState::default();
    let ui_update: RwSignal<()> = leptos_reactive::create_rw_signal(());

    let screen_calibrations: arrayvec::ArrayVec<
        (u8, ScreenCalibration<f32>),
        { (ats_common::MAX_SCREEN_ID + 1) as usize },
    > = (0..{ (ats_common::MAX_SCREEN_ID + 1) as usize })
        .filter_map(|i| {
            get_screens_dir().and_then(|config_dir| {
                let screen_path = config_dir.join(std::format!("screen_{}.json", i));
                if screen_path.exists() {
                    File::open(&screen_path).ok().and_then(|file| {
                        match serde_json::from_reader(file) {
                            Ok(calibration) => {
                                info!("Loaded {}", screen_path.display());
                                Some((i as u8, calibration))
                            }
                            Err(e) => {
                                error!("Failed to deserialize screen calibration: {}", e);
                                None
                            }
                        }
                    })
                } else {
                    None
                }
            })
        })
        .collect();

    let tracking_raw = RwSignal::new(false);
    let tracking = RwSignal::new(false);
    let testing = RwSignal::new(false);
    let recording = RwSignal::new(false);

    let mot_runner = Arc::new(Mutex::new(MotRunner {
        state,
        device: None,
        record_impact: false,
        record_packets: false,
        datapoints: datapoints.c(),
        packets: packets.c(),
        ui_update: ui_update.c(),
        ui_ctx,
        general_config: GeneralConfig::default(),
        wfnf_realign: true,
        screen_calibrations,
    }));

    // Create a main_window into which controls can be placed
    let mut main_win =
        iui::prelude::Window::new(&ui, "ATS Vision Tool", 640, 480, WindowType::NoMenubar);
    let (mut config_win, device_rs, accel_config_signal) =
        config_window::config_window(&ui, simulator_addr, udp_addr, mot_runner.c(), tokio_handle);
    let mut plots_window = plots_window::plots_window(&ui);

    let mut test_win =
        iui::prelude::Window::new(&ui, "Aimpoint Test", 640, 480, WindowType::NoMenubar);
    test_win.set_margined(&ui, false);
    test_win.set_borderless(&ui, true);

    let test_win_on_closing = move |_: &mut _| {
        testing.set(false);
    };
    test_win.on_closing(&ui, test_win_on_closing.c());
    let test_area = Area::new(
        &ui,
        Box::new(TestCanvas {
            ctx: ui.c(),
            window: test_win.c(),
            on_closing: Box::new(test_win_on_closing.c()),
            runner: mot_runner.c(),
            last_draw_width: None,
            last_draw_height: None,
        }),
    );
    let mut test_hbox = HorizontalBox::new(&ui);
    test_hbox.append(&ui, test_area.c(), LayoutStrategy::Stretchy);
    test_win.set_child(&ui, test_hbox);

    vision_module_gui::layout! { &ui,
        let vert_box = VerticalBox(padded: true) {
            Compact: let grid = LayoutGrid(padded: true) {
                (0, 0)(1, 1) Vertical (Fill, Fill) : let config_button = Button("Config")
                (1, 0)(1, 1) Vertical (Fill, Fill) : let plots_button = Button("Plots")
                // (1, 0)(1, 1) Vertical (Fill, Fill) : let marker_config_button = Button("Marker Config")
                (2, 0)(1, 1) Vertical (Fill, Fill) : let track_raw_button = Button(move || {
                    if !tracking_raw.get() { "Start Raw Tracking" } else { "Stop Raw Tracking" }
                })
                (3, 0)(1, 1) Vertical (Fill, Fill) : let track_button = Button(move || {
                    if !tracking.get() { "Start Tracking" } else { "Stop Tracking" }
                })
                (4, 0)(1, 1) Vertical (Fill, Fill) : let test_button = Button("Run Test")
                (5, 0)(1, 1) Vertical (Fill, Fill) : let windowed_checkbox = Checkbox("Windowed", checked: false)
                #[cfg(feature = "bevy")]
                (6, 0)(1, 1) Vertical (Fill, Fill) : let bevy_button = Button("Launch Bevy")
                (0, 1)(1, 1) Vertical (Fill, Fill) : let record_button = Button(move || {
                    if !recording.get() { "Start Recording" } else { "Stop Recording" }
                })
                (1, 1)(1, 1) Vertical (Fill, Fill) : let clear_packets_button = Button("Clear")
                (2, 1)(1, 1) Vertical (Fill, Fill) : let save_packets_button = Button("Save")
            }
            Compact: let separator = HorizontalSeparator()
            Compact: let spacer = Spacer()
            Compact: let form_vbox = VerticalBox(padded: true) {
                Compact: let form = Form(padded: true) {
                    (Compact, "Datapoints added:"): let collected_text = Label("")
                    (Compact, "Dataset"): let dataset_controls_group = HorizontalBox(padded: true) {
                        Compact: let add_datapoint_btn = Button("Add datapoint")
                        Compact: let remove_datapoint_btn = Button("Remove datapoint")
                        Compact: let clear_datapoints_btn = Button("Clear datapoints")
                        Compact: let record_impacts_cbx = Checkbox("Record impacts", checked: false)
                        Compact: let save_datapoints_btn = Button("Save to file")
                    }
                }
                Compact: let separator = HorizontalSeparator()
            }
            Stretchy: let run_raw_hbox = HorizontalBox() {
                Stretchy: let run_raw_area = Area(Box::new(RunRawCanvas {
                    ctx: ui.c(),
                    runner: mot_runner.c(),
                }))
            }
            Stretchy: let run_hbox = HorizontalBox() {
                Stretchy: let run_area = Area(Box::new(RunCanvas {
                    ctx: ui.c(),
                    runner: mot_runner.c(),
                }))
            }
        }
    }
    form_vbox.hide(&ui);
    run_raw_hbox.hide(&ui);
    run_hbox.hide(&ui);
    main_win.set_child(&ui, vert_box.clone());

    #[cfg(feature = "bevy")]
    let (tx, rx) = crossbeam::channel::bounded(10);

    #[cfg(feature = "bevy")]
    std::thread::spawn({
        let mot_runner = mot_runner.c();
        move || {
            App::new()
                .add_plugins((
                    DefaultPlugins
                        .set(WindowPlugin {
                            primary_window: Some(create_bevy_window(false)),
                            exit_condition: bevy::window::ExitCondition::DontExit,
                            close_when_requested: false,
                            ..default()
                        })
                        .set(WinitPlugin {
                            run_on_any_thread: true,
                            ..default()
                        }),
                    // LogDiagnosticsPlugin::default(),
                    // FrameTimeDiagnosticsPlugin,
                    InfiniteGridPlugin,
                    AtmospherePlugin,
                ))
                .add_systems(Startup, setup)
                .add_systems(
                    Update,
                    (read_stream, show_window, rotate_cube, hide_instead_of_close),
                )
                .add_event::<StreamEvent>()
                .insert_resource(StreamReceiver(rx))
                .insert_resource(MotRunnerResource(mot_runner))
                .run();
        }
    });

    #[cfg(feature = "bevy")]
    bevy_button.on_clicked(&ui, move |_| {
        tx.send(Event::OpenWindow).unwrap();
    });

    windowed_checkbox.on_toggled(&ui, {
        let test_win = test_win.c();
        let ui = ui.c();
        move |checked| {
            let mut test_win = test_win.c();
            let is_visible = test_win.visible(&ui);
            if checked {
                test_win.set_fullscreen(&ui, false);
            } else {
                test_win.set_fullscreen(&ui, true);
            }
            if !is_visible {
                test_win.hide(&ui);
            }
        }
    });

    create_effect({
        let ui = ui.c();
        let collected_text = collected_text.c();
        let datapoints = datapoints.c();
        move |_| {
            ui_update.with(|_| {
                let datapoints = datapoints.c();
                let datapoints = datapoints.lock();

                let mut collected_text = collected_text.c();

                collected_text.set_text(&ui, &datapoints.len().to_string());
            });
        }
    });

    // Disable buttons if no device is connected
    create_effect({
        let ui = ui.c();
        let test_win = test_win.c();
        let track_raw_button = track_raw_button.c();
        let track_button = track_button.c();
        let test_button = test_button.c();
        let test_win_on_closing = test_win_on_closing.c();
        move |_| {
            let mut test_win = test_win.c();
            let mut track_raw_button = track_raw_button.c();
            let mut track_button = track_button.c();
            let mut test_button = test_button.c();
            device_rs.with(|device| {
                if device.is_none() {
                    test_win_on_closing.c()(&mut test_win);
                    track_raw_button.disable(&ui);
                    track_button.disable(&ui);
                    test_button.disable(&ui);
                } else {
                    track_raw_button.enable(&ui);
                    track_button.enable(&ui);
                    test_button.enable(&ui);
                }
            });
        }
    });

    // Start/stop the mot_runner task as needed
    create_effect({
        let mot_runner = mot_runner.c();
        move |abort_handle: Option<Option<AbortHandle>>| {
            if tracking_raw.get() || tracking.get() || testing.get() {
                if let Some(Some(abort_handle)) = abort_handle {
                    // The mot_runner task is already running
                    Some(abort_handle)
                } else {
                    Some(
                        tokio::spawn(vision_module_gui::mot_runner::run(mot_runner.c()))
                            .abort_handle(),
                    )
                }
            } else {
                abort_handle??.abort();
                None
            }
        }
    });

    // Start/stop the raw frame loop task as needed
    create_effect({
        let mot_runner = mot_runner.c();
        move |abort_handle: Option<Option<AbortHandle>>| {
            if tracking_raw.get() {
                if let Some(Some(abort_handle)) = abort_handle {
                    // The mot_runner task is already running
                    Some(abort_handle)
                } else {
                    Some(
                        tokio::spawn(vision_module_gui::mot_runner::frame_loop(mot_runner.c()))
                            .abort_handle(),
                    )
                }
            } else {
                if let Some(Some(abort_handle)) = abort_handle {
                    abort_handle.abort();
                }
                None
            }
        }
    });

    // Keep the mot_runner device in sync with the selected device from the config window
    create_effect({
        let view = mot_runner.c();
        move |_| {
            let view = view.c();
            view.lock().device = device_rs.get();
        }
    });

    create_effect({
        let view = mot_runner.c();
        move |_| {
            let view = view.c();
            let madgwick = &mut view.lock().state.madgwick;
            *madgwick = ahrs::Madgwick::new_with_quat(
                1. / accel_config_signal.get().accel_odr as f32,
                0.2,
                madgwick.quat,
            );
        }
    });

    // Show the test window and form when running test or when calibrating marker offsets
    create_effect({
        let ui = ui.c();
        let form_vbox = form_vbox.c();
        let test_win = test_win.c();
        move |_| {
            if testing.get() {
                form_vbox.c().show(&ui);
                test_win.c().show(&ui);
            } else {
                form_vbox.c().hide(&ui);
                test_win.c().hide(&ui);
            }
        }
    });

    // Show the run_raw_hbox form when tracking
    create_effect({
        let ui = ui.c();
        let run_raw_hbox = run_raw_hbox.c();
        move |_| {
            if tracking_raw.get() {
                run_raw_hbox.c().show(&ui);
            } else {
                run_raw_hbox.c().hide(&ui);
            }
        }
    });

    // Show the run_hbox form when tracking
    create_effect({
        let ui = ui.c();
        let run_hbox = run_hbox.c();
        move |_| {
            if tracking.get() {
                run_hbox.c().show(&ui);
            } else {
                run_hbox.c().hide(&ui);
            }
        }
    });

    add_datapoint_btn.on_clicked(&ui, {
        let ui = ui.c();
        let datapoints = datapoints.c();
        let collected_text = collected_text.c();
        let state = mot_runner.c();
        move |_| {
            let datapoints = datapoints.c();
            let mut datapoints = datapoints.lock();
            let mut collected_text = collected_text.c();

            let mut frame = TestFrame {
                fv_aimpoint_x: None,
                fv_aimpoint_y: None,
                opposite_cant: None,
                position_x: None,
                position_y: None,
                position_z: None,
            };

            let runner = state.lock();
            let state = &runner.state;

            {
                let fv_aimpoint = state.fv_aimpoint;
                frame.fv_aimpoint_x = Some(fv_aimpoint.x);
                frame.fv_aimpoint_y = Some(fv_aimpoint.y);

                let gravity_vec = runner
                    .state
                    .orientation
                    .inverse_transform_vector(&nalgebra::Vector3::z_axis());
                let gravity_angle = (f32::atan2(-gravity_vec.z, -gravity_vec.x)
                    + std::f32::consts::PI / 2.)
                    .to_degrees();
                frame.opposite_cant = Some(gravity_angle);

                let translation = &runner.state.translation_mat;
                frame.position_x = Some(translation.x);
                frame.position_y = Some(translation.y);
                frame.position_z = Some(translation.z);
            }

            datapoints.push(frame);
            collected_text.set_text(&ui, datapoints.len().to_string().as_str());
        }
    });

    remove_datapoint_btn.on_clicked(&ui, {
        let ui = ui.c();
        let datapoints = datapoints.c();
        let collected_text = collected_text.c();
        move |_| {
            let datapoints = datapoints.c();
            let mut datapoints = datapoints.lock();
            let mut collected_text = collected_text.c();
            datapoints.pop();
            collected_text.set_text(&ui, datapoints.len().to_string().as_str());
        }
    });

    clear_datapoints_btn.on_clicked(&ui, {
        let ui = ui.c();
        let datapoints = datapoints.c();
        move |_| {
            let datapoints = datapoints.c();
            let mut datapoints = datapoints.lock();
            datapoints.clear();
            collected_text.set_text(&ui, datapoints.len().to_string().as_str());
        }
    });

    record_impacts_cbx.on_toggled(&ui, {
        let mot_runner = mot_runner.c();
        move |checked| {
            mot_runner.lock().record_impact = checked;
        }
    });

    save_datapoints_btn.on_clicked(&ui, {
        let ui = ui.c();
        let main_win = main_win.c();
        move |_| {
            let datapoints = datapoints.c();
            let datapoints = datapoints.lock();
            let path_buf =
                main_win.save_file_with_filter(&ui, &[FileTypeFilter::new("csv").extension("csv")]);
            if let Some(mut path_buf) = path_buf {
                if path_buf.extension() != Some("csv".as_ref()) {
                    path_buf.as_mut_os_string().push(".csv");
                }
                let file = File::create(path_buf).expect("Could not create file");
                let mut writer = csv::Writer::from_writer(file);

                for datapoint in datapoints.as_slice() {
                    writer.serialize(datapoint).expect("Could not serialize");
                }
                writer.flush().expect("Could not flush");
            }
        }
    });

    config_button.on_clicked(&ui, {
        let ui = ui.c();
        move |_| {
            config_win.show(&ui);
        }
    });

    plots_button.on_clicked(&ui, {
        let ui = ui.c();
        move |_| {
            plots_window.show(&ui);
        }
    });

    // marker_config_button.on_clicked(&ui, {
    //     let ui = ui.c();
    //     move |_| {
    //         marker_config_win.show(&ui);
    //     }
    // });

    track_raw_button.on_clicked(&ui, move |_| {
        tracking_raw.set(!tracking_raw.get_untracked())
    });
    track_button.on_clicked(&ui, move |_| tracking.set(!tracking.get_untracked()));
    test_button.on_clicked(&ui, move |_| testing.set(true));
    record_button.on_clicked(&ui, {
        let mot_runner = mot_runner.c();
        move |_| {
            let new_value = !recording.get_untracked();
            recording.set(new_value);
            mot_runner.lock().record_packets = new_value;
        }
    });

    clear_packets_button.on_clicked(&ui, {
        let packets = packets.c();
        move |_| {
            packets.lock().clear();
        }
    });

    save_packets_button.on_clicked(&ui, {
        let ui = ui.c();
        let main_win = main_win.c();
        let packets = packets.c();
        let mot_runner = mot_runner.c();
        move |_| {
            let packets = packets.lock();
            let path_buf =
                main_win.save_file_with_filter(&ui, &[FileTypeFilter::new("bin").extension("bin")]);
            if let Some(mut path_buf) = path_buf {
                if path_buf.extension() != Some("bin".as_ref()) {
                    path_buf.as_mut_os_string().push(".bin");
                }
                let mut file = File::create(path_buf).expect("Could not create file");
                let mut bytes = Vec::new();

                postcard::to_slice(&mot_runner
                    .lock()
                    .general_config, &mut bytes).unwrap();

                for (timestamp, packet_data) in packets.iter() {
                    let packet = ats_usb::packets::vm::Packet {
                        data: packet_data.clone(),
                        id: 0,
                    };
                    bytes.extend_from_slice(&timestamp.to_le_bytes());
                    postcard::to_slice(&packet, &mut bytes).unwrap();
                }

                file.write_all(&bytes).expect("Could not write to file");
            }
        }
    });

    main_win.show(&ui);

    ui.ui_timer(5, {
        let ui = ui.c();
        let run_raw_area = run_raw_area.c();
        let run_area = run_area.c();
        let test_area = test_area.c();
        move || {
            if tracking_raw.get_untracked() {
                run_raw_area.queue_redraw_all(&ui);
            }
            if tracking.get_untracked() {
                run_area.queue_redraw_all(&ui);
            }
            if testing.get_untracked() {
                test_area.queue_redraw_all(&ui);
            }
            true
        }
    });

    // Run the application
    let mut ev = ui.event_loop();
    ev.run(&ui);

    leptos_rt.dispose();
    Ok(())
}
