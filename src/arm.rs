use std::{io::Result, thread::sleep, time::Duration};

use chess::{ChessMove, Color, File, Rank, Square};
use serialport::SerialPort;

use crate::{
    checkpoint::{Checkpoint, Coordinate, fraction_along, path_time},
    engine::MoveType,
};

const DISPATCH_DELAY_MS: u64 = 50;
/// Units measured in squares.
const DISPOSAL: Coordinate = Coordinate(-5.0, 6.0, 2.0);
const GRAB: [f32; 3] = [f32::INFINITY, f32::INFINITY, f32::INFINITY];
const RELEASE: [f32; 3] = [f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY];

/// Potential actions the robot could take.
#[derive(Debug, PartialEq)]
pub enum Action {
    Grab,
    Release,
    Move(Location, f32),
}

/// Potential locations the robot arm could move to.
#[derive(Debug, PartialEq)]
pub enum Location {
    Disposal,
    GrabLevel(Square),
    HoverLevel(Square),
    Reset,
}

/// A model of a 3 DOF robot arm. The first joint rotates
/// about the Z axis and the rest rotate about the X axis.
#[derive(Debug)]
pub struct Arm {
    device: Box<dyn SerialPort>,
    lengths: [f32; 3],
    speed: f32,
    perspective: Color,
    square_width: f32,
    reset_position: Coordinate,
    current_checkpoint: Checkpoint,
}

impl Arm {
    /// Initialise the model of the arm and open a connection to the physical device. A baud
    /// rate of `115_200` is good for most cases (`0` when testing on a Mac with a pty!).
    pub fn new(
        device: &str,
        lengths: [f32; 3],
        speed: f32,
        perspective: Color,
        square_width: f32,
        baud_rate: u32,
    ) -> Result<Arm> {
        let current = Checkpoint {
            position: Coordinate(0.0, 0.0, lengths.iter().sum::<f32>()),
            speed: 0.0,
        };

        Ok(Arm {
            device: serialport::new(device, baud_rate).open()?,
            lengths,
            speed,
            perspective,
            square_width,
            reset_position: current.position,
            current_checkpoint: current,
        })
    }

    /// Convert the angles to bytes and send to the physical robot.
    fn dispatch(&mut self, angles: [f32; 3]) {
        self.device
            // The Arduino expects little endian format.
            .write_all(angles.map(|x| x.to_le_bytes()).as_flattened())
            .ok();
    }

    /// Make the arm carry out the given list of actions by converting each action
    /// into a series of coordinates to go to - `Grab` and `Release` are mapped to
    /// special values (`±∞` for all angles).
    pub fn execute(&mut self, actions: Vec<Action>) {
        for action in actions {
            match action {
                Action::Grab => self.dispatch(GRAB),
                Action::Release => self.dispatch(RELEASE),
                Action::Move(location, speed) => {
                    let target = self.make_checkpoint(location, speed);
                    let current_angles = self.inverse_kinematics(self.current_checkpoint.position);
                    let target_angles = self.inverse_kinematics(target.position);

                    let mut time_f = 0.0;
                    let time_increment =
                        DISPATCH_DELAY_MS as f32 / path_time(self.current_checkpoint, target);

                    while time_f < 1.0 {
                        time_f += time_increment;
                        let f = fraction_along(self.current_checkpoint.speed, target.speed, time_f)
                            .min(1.0);

                        let mut output = [0.0; 3];
                        for i in 0..3 {
                            output[i] = (1.0 - f) * current_angles[i] + target_angles[i] * f;
                        }

                        self.dispatch(output);
                        sleep(Duration::from_millis(DISPATCH_DELAY_MS));
                    }

                    self.current_checkpoint = target;
                }
            }
        }
    }

    /// Map moves to a series of actions the robot needs to carry out depending
    /// on the moves type (e.g., moving to a piece, grabbing it, etc.).
    pub fn generate_actions(&self, engine_move: ChessMove, move_type: MoveType) -> Vec<Action> {
        let from = engine_move.get_source();
        let to = engine_move.get_dest();

        let move_piece = vec![
            Action::Move(Location::HoverLevel(from), self.speed),
            Action::Move(Location::GrabLevel(from), 0.0),
            Action::Grab,
            Action::Move(Location::HoverLevel(from), self.speed),
            Action::Move(Location::HoverLevel(to), self.speed),
            Action::Move(Location::GrabLevel(to), 0.0),
            Action::Release,
            Action::Move(Location::HoverLevel(to), self.speed),
            Action::Move(Location::Reset, 0.0),
        ];

        let mut actions = match move_type {
            MoveType::Normal => vec![],
            MoveType::Capture => vec![
                Action::Move(Location::HoverLevel(to), self.speed),
                Action::Move(Location::GrabLevel(to), 0.0),
                Action::Grab,
                Action::Move(Location::HoverLevel(to), self.speed),
                Action::Move(Location::Disposal, 0.0),
                Action::Release,
            ],
            MoveType::EnPassant => {
                let capture = to.ubackward(self.perspective);
                vec![
                    Action::Move(Location::HoverLevel(capture), self.speed),
                    Action::Move(Location::GrabLevel(capture), 0.0),
                    Action::Grab,
                    Action::Move(Location::HoverLevel(capture), self.speed),
                    Action::Move(Location::Disposal, 0.0),
                    Action::Release,
                ]
            }
            MoveType::Castle { rook_from, rook_to } => vec![
                Action::Move(Location::HoverLevel(rook_from), self.speed),
                Action::Move(Location::GrabLevel(rook_from), 0.0),
                Action::Grab,
                Action::Move(Location::HoverLevel(rook_to), self.speed),
                Action::Move(Location::GrabLevel(rook_to), 0.0),
                Action::Release,
                Action::Move(Location::HoverLevel(rook_to), self.speed),
            ],
        };

        actions.extend(move_piece);
        actions
    }

    /// Converts from coordinates that the robot's hand needs to reach to angles
    /// its servos should be set to in degrees using a geometric solution
    /// (derivation in the README).
    fn inverse_kinematics(&self, to: Coordinate) -> [f32; 3] {
        let r = (to.0.powi(2) + to.1.powi(2)).sqrt();
        let z = to.2 - self.lengths[0];

        let servo_1 = to.0.atan2(to.1);
        let servo_3 = ((r.powi(2) + z.powi(2) - self.lengths[1].powi(2) - self.lengths[2].powi(2))
            / (2.0 * self.lengths[1] * self.lengths[2]))
            .clamp(-1.0, 1.0)
            .acos();
        let servo_2 = r.atan2(z)
            - (self.lengths[2] * servo_3.sin())
                .atan2(self.lengths[1] + self.lengths[2] * servo_3.cos());

        [servo_1, servo_2, servo_3].map(f32::to_degrees)
    }

    /// Convert a given location and a speed value into a checkpoint to move the robot to.
    fn make_checkpoint(&self, target: Location, speed: f32) -> Checkpoint {
        let coordinate = |square: Square, z: f32| {
            let (x, y) = match self.perspective {
                Color::White => (
                    (square.get_file().to_index() as i64 - File::E.to_index() as i64) as f32,
                    (square.get_rank().to_index() + 2) as f32,
                ),
                Color::Black => (
                    (File::D.to_index() as i64 - square.get_file().to_index() as i64) as f32,
                    (Rank::Eighth.to_index() as i64 - square.get_rank().to_index() as i64 + 2)
                        as f32,
                ),
            };
            Coordinate(x + 0.5, y + 0.5, z)
        };

        Checkpoint {
            position: match target {
                Location::Disposal => DISPOSAL,
                Location::GrabLevel(square) => coordinate(square, 0.5),
                Location::HoverLevel(square) => coordinate(square, 2.0),
                Location::Reset => self.reset_position,
            }
            .mul(self.square_width),
            speed,
        }
    }
}

#[cfg(test)]
mod tests {
    use chess::{ChessMove, Color, Square};
    use nix::{pty::openpty, unistd::ttyname};
    use std::{fs::File, io::Read, str::FromStr};

    use crate::{
        arm::{Action, Arm, DISPATCH_DELAY_MS, DISPOSAL, GRAB, Location, RELEASE},
        checkpoint::{Checkpoint, Coordinate, fraction_along, path_time},
        engine::MoveType,
    };

    fn make_virtual_robot() -> (Arm, File, File) {
        let result = openpty(None, None).unwrap();
        let name = ttyname(&result.slave)
            .unwrap()
            .to_string_lossy()
            .to_string();

        let arm = Arm::new(name.as_str(), [6.5, 6.5, 14.0], 2.5, Color::White, 3.3, 0).unwrap();

        (arm, File::from(result.master), File::from(result.slave))
    }

    fn read_last_dispatch(master: &mut File) -> Vec<f32> {
        let mut received_bytes: [u8; 12] = [0; 12];
        master.read_exact(&mut received_bytes).unwrap();

        received_bytes
            .chunks_exact(4)
            .map(|f| f32::from_le_bytes(*f.as_array().unwrap()))
            .collect()
    }

    fn get_test_actions() -> [Vec<Action>; 4] {
        [
            vec![
                Action::Move(Location::HoverLevel(Square::F3), 2.5),
                Action::Move(Location::GrabLevel(Square::F3), 0.0),
                Action::Grab,
                Action::Move(Location::HoverLevel(Square::F3), 2.5),
                Action::Move(Location::HoverLevel(Square::G5), 2.5),
                Action::Move(Location::GrabLevel(Square::G5), 0.0),
                Action::Release,
                Action::Move(Location::HoverLevel(Square::G5), 2.5),
                Action::Move(Location::Reset, 0.0),
            ],
            vec![
                Action::Move(Location::HoverLevel(Square::G2), 2.5),
                Action::Move(Location::GrabLevel(Square::G2), 0.0),
                Action::Grab,
                Action::Move(Location::HoverLevel(Square::G2), 2.5),
                Action::Move(Location::Disposal, 0.0),
                Action::Release,
                Action::Move(Location::HoverLevel(Square::B7), 2.5),
                Action::Move(Location::GrabLevel(Square::B7), 0.0),
                Action::Grab,
                Action::Move(Location::HoverLevel(Square::B7), 2.5),
                Action::Move(Location::HoverLevel(Square::G2), 2.5),
                Action::Move(Location::GrabLevel(Square::G2), 0.0),
                Action::Release,
                Action::Move(Location::HoverLevel(Square::G2), 2.5),
                Action::Move(Location::Reset, 0.0),
            ],
            vec![
                Action::Move(Location::HoverLevel(Square::B5), 2.5),
                Action::Move(Location::GrabLevel(Square::B5), 0.0),
                Action::Grab,
                Action::Move(Location::HoverLevel(Square::B5), 2.5),
                Action::Move(Location::Disposal, 0.0),
                Action::Release,
                Action::Move(Location::HoverLevel(Square::C5), 2.5),
                Action::Move(Location::GrabLevel(Square::C5), 0.0),
                Action::Grab,
                Action::Move(Location::HoverLevel(Square::C5), 2.5),
                Action::Move(Location::HoverLevel(Square::B6), 2.5),
                Action::Move(Location::GrabLevel(Square::B6), 0.0),
                Action::Release,
                Action::Move(Location::HoverLevel(Square::B6), 2.5),
                Action::Move(Location::Reset, 0.0),
            ],
            vec![
                Action::Move(Location::HoverLevel(Square::A8), 2.5),
                Action::Move(Location::GrabLevel(Square::A8), 0.0),
                Action::Grab,
                Action::Move(Location::HoverLevel(Square::D8), 2.5),
                Action::Move(Location::GrabLevel(Square::D8), 0.0),
                Action::Release,
                Action::Move(Location::HoverLevel(Square::D8), 2.5),
                Action::Move(Location::HoverLevel(Square::E8), 2.5),
                Action::Move(Location::GrabLevel(Square::E8), 0.0),
                Action::Grab,
                Action::Move(Location::HoverLevel(Square::E8), 2.5),
                Action::Move(Location::HoverLevel(Square::C8), 2.5),
                Action::Move(Location::GrabLevel(Square::C8), 0.0),
                Action::Release,
                Action::Move(Location::HoverLevel(Square::C8), 2.5),
                Action::Move(Location::Reset, 0.0),
            ],
        ]
    }

    #[test]
    fn new_test() {
        make_virtual_robot();

        Arm::new("/", [3.0, 1.2, 4.8], 2.5, Color::White, 0.5, 115_200)
            .expect_err("This device should not exist.");
    }

    #[test]
    fn dispatch_angles_test() {
        let (mut arm, mut master, _) = make_virtual_robot();

        let angles = [1.24, -4.3, 1.22];
        arm.dispatch(angles);

        assert_eq!(angles.to_vec(), read_last_dispatch(&mut master))
    }

    #[test]
    fn execute_test() {
        let (mut arm, mut master, _) = make_virtual_robot();

        let start = arm.current_checkpoint;
        let start_angles = arm.inverse_kinematics(start.position);
        arm.execute(vec![
            Action::Release,
            Action::Move(Location::HoverLevel(Square::F3), 75.0),
            Action::Grab,
        ]);
        let end = arm.current_checkpoint;
        let end_angles = arm.inverse_kinematics(end.position);
        let time = path_time(start, end);

        assert_eq!(RELEASE.to_vec(), read_last_dispatch(&mut master));
        for i in 1..15 {
            let f = fraction_along(
                start.speed,
                end.speed,
                ((i * DISPATCH_DELAY_MS) as f32 / time).min(1.0),
            );
            let mut output = [0.0; 4];
            for i in 0..3 {
                output[i] = (1.0 - f) * start_angles[i] + end_angles[i] * f;
            }

            assert!(
                output
                    .iter()
                    .zip(read_last_dispatch(&mut master))
                    .all(|(x, y)| x - y < 10e-4)
            );
        }
        assert_eq!(GRAB.to_vec(), read_last_dispatch(&mut master));
    }

    #[test]
    fn generate_actions_test() {
        let (arm, _, _) = make_virtual_robot();
        let actions = get_test_actions();

        assert_eq!(
            actions[0],
            arm.generate_actions(ChessMove::from_str("f3g5").unwrap(), MoveType::Normal)
        );
        assert_eq!(
            actions[1],
            arm.generate_actions(ChessMove::from_str("b7g2").unwrap(), MoveType::Capture)
        );
        assert_eq!(
            actions[2],
            arm.generate_actions(ChessMove::from_str("c5b6").unwrap(), MoveType::EnPassant)
        );
        assert_eq!(
            actions[3],
            arm.generate_actions(
                ChessMove::from_str("e8c8").unwrap(),
                MoveType::Castle {
                    rook_from: Square::A8,
                    rook_to: Square::D8,
                },
            )
        );
    }

    #[test]
    fn inverse_kinematics_test() {
        let (arm, _, _) = make_virtual_robot();
        // forward(inverse_kinematics(coordinate)) == coordinate, so we can use this for testing.
        let forward = |angles: [f32; 3]| {
            let angles = angles.map(f32::to_radians);
            let (a, b) = (angles[1], angles[1] + angles[2]);

            let z = arm.lengths[0] + arm.lengths[1] * a.cos() + arm.lengths[2] * b.cos();
            let r = arm.lengths[1] * a.sin() + arm.lengths[2] * b.sin();

            let x = r * angles[0].sin();
            let y = r * angles[0].cos();

            Coordinate(x, y, z)
        };

        assert_eq!(DISPOSAL, forward(arm.inverse_kinematics(DISPOSAL)));
        assert_eq!(
            arm.reset_position,
            forward(arm.inverse_kinematics(arm.reset_position))
        );
        // Out of reach
        assert_ne!(
            Coordinate(-2.5, 3.5, 0.5),
            forward(arm.inverse_kinematics(Coordinate(-2.5, 3.5, 0.5)))
        );
    }

    #[test]
    fn make_checkpoint_test() {
        let (mut arm, _, _) = make_virtual_robot();

        assert_eq!(
            Checkpoint {
                position: DISPOSAL.mul(arm.square_width),
                speed: arm.speed,
            },
            arm.make_checkpoint(Location::Disposal, arm.speed)
        );
        assert_eq!(
            Checkpoint {
                position: Coordinate(-2.5, 4.5, 0.5).mul(arm.square_width),
                speed: 5.3,
            },
            arm.make_checkpoint(Location::GrabLevel(Square::B3), 5.3)
        );

        arm.perspective = Color::Black;

        assert_eq!(
            Checkpoint {
                position: arm.reset_position.mul(arm.square_width),
                speed: 0.0,
            },
            arm.make_checkpoint(Location::Reset, 0.0)
        );
        assert_eq!(
            Checkpoint {
                position: Coordinate(-1.5, 2.5, 2.0).mul(arm.square_width),
                speed: 0.7,
            },
            arm.make_checkpoint(Location::HoverLevel(Square::F8), 0.7)
        );
    }
}
