use chess::Color;
use std::{env, error::Error, thread::sleep, time::Duration};

use crate::{
    arm::Arm,
    camera::Camera,
    engine::{BoardChange, Engine},
    stream::{CAMERA_ERROR, GAME_OVER, INVALID, send_image, start_stream, wait_on_notification},
};

mod arm;
mod camera;
mod checkpoint;
mod engine;
mod stream;

const BAUD_RATE: u32 = 115_200;
const FIFTY_MS: Duration = Duration::from_millis(50);
/// To access the Raspberry Pi's camera.
const PIPELINE: &str = "libcamerasrc ! videoconvert ! \
                        video/x-raw,format=BGR888,width=480,height=480 ! \
                        appsink sync=false drop=true max-buffers=1";

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    assert!(args.len() == 8, "Please provide the 7 required arguments."); // Includes program name - arg 0.

    let device = args[1].clone();
    let program = args[2].clone();
    let lengths = [
        args[3].parse::<f32>()?,
        args[4].parse::<f32>()?,
        args[5].parse::<f32>()?,
    ];
    let speed = args[6].parse::<f32>()?;
    let square_width = args[7].parse::<f32>()?;

    let mut client = start_stream();

    loop {
        let mut camera = Camera::open(PIPELINE)?;
        let perspective = loop {
            match camera.configure() {
                Ok(colour) => break colour,
                Err(_) => {
                    client.send(CAMERA_ERROR).ok();
                    sleep(Duration::from_secs(1));
                }
            }
        };

        let mut engine = Engine::new(program.as_str())?;
        let mut arm = Arm::new(
            device.as_str(),
            lengths,
            speed,
            perspective,
            square_width,
            BAUD_RATE,
        )?;

        if perspective == Color::White {
            send_image(&mut client, &camera.last_frame);
            let (engine_move, move_type, _) = engine.get_move("")?;
            arm.execute(arm.generate_actions(engine_move, move_type));
            send_image(&mut client, &camera.last_frame);
        }

        loop {
            let (white_board, black_board) = camera.process_frame()?;
            send_image(&mut client, &camera.last_frame);

            match engine.update_state(white_board, black_board) {
                BoardChange::None => sleep(FIFTY_MS),
                BoardChange::Valid(uci) => {
                    // If move is a promotion.
                    if uci.ends_with('q') {
                        wait_on_notification(&mut client);
                    }

                    let (engine_move, move_type, is_promotion) = engine.get_move(uci.as_str())?;
                    arm.execute(arm.generate_actions(engine_move, move_type));

                    if is_promotion {
                        wait_on_notification(&mut client);
                    }
                }
                BoardChange::GameOver => {
                    client.send(GAME_OVER).ok();
                    break;
                }
                BoardChange::Invalid => {
                    client.send(INVALID).ok();
                    sleep(FIFTY_MS);
                }
            }

            send_image(&mut client, &camera.last_frame);
        }
    }
}
