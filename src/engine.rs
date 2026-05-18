use chess::{
    BitBoard, Board, BoardStatus, ChessMove, Color, MoveGen,
    Piece::{Queen, Rook},
    Square, get_file,
};
use std::{
    io::{BufRead, BufReader, Read, Result, Write},
    process::{Child, ChildStdin, ChildStdout, Command, Stdio},
    str::FromStr,
};

const MOVE_SETUP_PROMPT: &str = "position startpos moves ";
const MOVE_GEN_PROMPT: &str = "\ngo depth 12\n";

/// Represents the types of user made changes detected on the chessboard.
/// A valid change comes with a UCI string representing the move.
#[derive(Debug, PartialEq)]
pub enum BoardChange {
    None,
    Valid(String),
    GameOver,
    Invalid,
}

/// Represents the possible types of moves on the chessboard.
/// Castle also stores the start and end squares of the rook.
#[derive(Debug, PartialEq)]
pub enum MoveType {
    Normal,
    Capture,
    EnPassant,
    Castle { rook_from: Square, rook_to: Square },
}

/// Provides access to the chess engine for move generation and
/// tracks the chessboard's current state.
#[derive(Debug)]
pub struct Engine {
    move_history: String,
    process: Child,
    input: ChildStdin,
    output: BufReader<ChildStdout>,
    board_state: Board,
}

impl Engine {
    /// Initialise a chess engine from a path to the program's executable.
    pub fn new(binary: &str) -> Result<Engine> {
        let mut process = Command::new(binary)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;

        let input = process.stdin.take().unwrap();
        let output = BufReader::new(process.stdout.take().unwrap());

        Ok(Engine {
            move_history: String::from(MOVE_SETUP_PROMPT),
            process,
            input,
            output,
            board_state: Board::default(),
        })
    }

    /// Query the engine's program for the best move in the current position, and
    /// update the current state accordingly. Assumes the chess program outputs
    /// a valid move and is not broken itself.
    pub fn get_move(&mut self, last: &str) -> Result<(ChessMove, MoveType, bool)> {
        self.move_history.push_str(last);
        self.move_history.push(' ');
        self.input.write_all(self.move_history.as_bytes())?;
        self.input.write_all(MOVE_GEN_PROMPT.as_bytes())?;

        #[allow(clippy::lines_filter_map_ok)]
        let mut best_move = ChessMove::from_str(
            self.output
                .by_ref()
                .lines()
                .flatten()
                .find(|line| line.starts_with("bestmove"))
                .unwrap()
                .split(' ')
                .nth(1)
                .unwrap(),
        )
        .unwrap();

        // Force autoqueen rule.
        if !matches!(best_move.get_promotion(), None | Some(Queen)) {
            best_move = ChessMove::new(best_move.get_source(), best_move.get_dest(), Some(Queen));
        };

        let new_board = self.board_state.make_move_new(best_move);
        let move_type = self.get_move_type(&new_board);
        self.board_state = new_board;

        Ok((best_move, move_type, best_move.get_promotion().is_some()))
    }

    /// Find the type of using bitwise comparisons on the old and new boards.
    fn get_move_type(&self, new_board: &Board) -> MoveType {
        let color = self.board_state.side_to_move();
        let capture =
            self.board_state.combined().0.count_ones() - new_board.combined().0.count_ones() != 0;

        if capture {
            let change = self.board_state.combined() ^ new_board.combined();
            // 1 square will change for a capture and 3 for en passant.
            if change.0.count_ones() == 1 {
                MoveType::Capture
            } else {
                MoveType::EnPassant
            }
        } else {
            let change = self.board_state.color_combined(color) ^ new_board.color_combined(color);
            // 2 squares (for 1 piece) will change if the move is normal, 4 (for 2 pieces) if castle.
            if change.0.count_ones() == 2 {
                MoveType::Normal
            } else {
                let rook_change = self.board_state.pieces(Rook) ^ new_board.pieces(Rook);
                // A castled rook must come from the A or H files and move to the D or F files.
                let from = rook_change & (get_file(chess::File::A) | get_file(chess::File::H));
                let to = rook_change & (get_file(chess::File::D) | get_file(chess::File::F));

                MoveType::Castle {
                    rook_from: from.to_square(),
                    rook_to: to.to_square(),
                }
            }
        }
    }

    /// Check whether a given set of bitboards represents a valid continuation from the
    /// current position, and update the board if that is the case.
    pub fn update_state(&mut self, new_white: BitBoard, new_black: BitBoard) -> BoardChange {
        let current_white = *self.board_state.color_combined(Color::White);
        let current_black = *self.board_state.color_combined(Color::Black);

        if new_white == current_white && new_black == current_black {
            return BoardChange::None;
        }

        let moves = MoveGen::new_legal(&self.board_state)
            .filter(|m| matches!(m.get_promotion(), None | Some(Queen))); // Use autoqueen rule.

        for choice in moves {
            let board = self.board_state.make_move_new(choice);
            let valid_white = *board.color_combined(Color::White);
            let valid_black = *board.color_combined(Color::Black);

            if new_white == valid_white && new_black == valid_black {
                self.board_state = board;

                return match board.status() {
                    BoardStatus::Ongoing => BoardChange::Valid(choice.to_string()),
                    _ => BoardChange::GameOver,
                };
            }
        }

        BoardChange::Invalid
    }
}

impl Drop for Engine {
    fn drop(&mut self) {
        self.process.kill().ok();
        self.process.wait().ok();
    }
}

#[cfg(test)]
mod tests {
    use chess::{Board, ChessMove, Color, Error, Square};
    use nix::{sys::signal::kill, unistd::Pid};
    use std::str::FromStr;

    use crate::engine::{BoardChange, Engine, MoveType};

    fn play_move(game: &mut Vec<(Board, String)>, uci: &str) {
        game.push((
            game.last()
                .unwrap()
                .0
                .make_move_new(ChessMove::from_str(uci).unwrap()),
            uci.to_string(),
        ))
    }

    fn opera_game() -> Vec<(Board, String)> {
        let mut opera_game: Vec<(Board, String)> = vec![(Board::default(), String::new())];

        play_move(&mut opera_game, "e2e4");
        play_move(&mut opera_game, "e7e5");
        play_move(&mut opera_game, "g1f3");
        play_move(&mut opera_game, "d7d6");
        play_move(&mut opera_game, "d2d4");
        play_move(&mut opera_game, "c8g4");
        play_move(&mut opera_game, "d4e5");
        play_move(&mut opera_game, "g4f3");
        play_move(&mut opera_game, "d1f3");
        play_move(&mut opera_game, "d6e5");
        play_move(&mut opera_game, "f1c4");
        play_move(&mut opera_game, "g8f6");
        play_move(&mut opera_game, "f3b3");
        play_move(&mut opera_game, "d8e7");
        play_move(&mut opera_game, "b1c3");
        play_move(&mut opera_game, "c7c6");
        play_move(&mut opera_game, "c1g5");
        play_move(&mut opera_game, "b7b5");
        play_move(&mut opera_game, "c3b5");
        play_move(&mut opera_game, "c6b5");
        play_move(&mut opera_game, "c4b5");
        play_move(&mut opera_game, "b8d7");
        play_move(&mut opera_game, "e1c1");
        play_move(&mut opera_game, "a8d8");
        play_move(&mut opera_game, "d1d7");
        play_move(&mut opera_game, "d8d7");
        play_move(&mut opera_game, "h1d1");
        play_move(&mut opera_game, "e7e6");
        play_move(&mut opera_game, "b5d7");
        play_move(&mut opera_game, "f6d7");
        play_move(&mut opera_game, "b3b8");
        play_move(&mut opera_game, "d7b8");
        play_move(&mut opera_game, "d1d8");

        opera_game
    }

    fn en_passsant_game() -> Vec<(Board, String)> {
        let mut en_passant_mate: Vec<(Board, String)> = vec![(Board::default(), String::new())];

        play_move(&mut en_passant_mate, "e2e4");
        play_move(&mut en_passant_mate, "e7e6");
        play_move(&mut en_passant_mate, "d2d4");
        play_move(&mut en_passant_mate, "d7d5");
        play_move(&mut en_passant_mate, "e4e5");
        play_move(&mut en_passant_mate, "c7c5");
        play_move(&mut en_passant_mate, "c2c3");
        play_move(&mut en_passant_mate, "c5d4");
        play_move(&mut en_passant_mate, "c3d4");
        play_move(&mut en_passant_mate, "f8b4");
        play_move(&mut en_passant_mate, "b1c3");
        play_move(&mut en_passant_mate, "b8c6");
        play_move(&mut en_passant_mate, "g1f3");
        play_move(&mut en_passant_mate, "g8e7");
        play_move(&mut en_passant_mate, "f1d3");
        play_move(&mut en_passant_mate, "e8g8");
        play_move(&mut en_passant_mate, "d3h7");
        play_move(&mut en_passant_mate, "g8h7");
        play_move(&mut en_passant_mate, "f3g5");
        play_move(&mut en_passant_mate, "h7g6");
        play_move(&mut en_passant_mate, "h2h4");
        play_move(&mut en_passant_mate, "c6d4");
        play_move(&mut en_passant_mate, "d1g4");
        play_move(&mut en_passant_mate, "f7f5");
        play_move(&mut en_passant_mate, "h4h5");
        play_move(&mut en_passant_mate, "g6h6");
        play_move(&mut en_passant_mate, "g5e6");
        play_move(&mut en_passant_mate, "g7g5");
        play_move(&mut en_passant_mate, "h5g6");

        en_passant_mate
    }

    fn promotion_game() -> Vec<(Board, String)> {
        let mut promotion_game: Vec<(Board, String)> = vec![(Board::default(), String::new())];

        play_move(&mut promotion_game, "e2e4");
        play_move(&mut promotion_game, "g8f6");
        play_move(&mut promotion_game, "b1c3");
        play_move(&mut promotion_game, "d7d5");
        play_move(&mut promotion_game, "e4e5");
        play_move(&mut promotion_game, "d5d4");
        play_move(&mut promotion_game, "e5f6");
        play_move(&mut promotion_game, "d4c3");
        play_move(&mut promotion_game, "d2d4");
        play_move(&mut promotion_game, "c3b2");
        play_move(&mut promotion_game, "f6g7");
        play_move(&mut promotion_game, "b2a1b");
        play_move(&mut promotion_game, "g7h8q");

        promotion_game
    }

    // Should never actually return an error.
    fn get_transitions() -> Result<Vec<(Board, Board, MoveType)>, Error> {
        Ok(vec![
            (
                Board::from_str("r2qkbnr/pp3ppp/2n1b3/2pp4/8/1PN5/PBQPPPPP/2KR1BNR b kq - 3 7")?,
                Board::from_str("r3kbnr/pp3ppp/2n1bq2/2pp4/8/1PN5/PBQPPPPP/2KR1BNR w kq - 4 8")?,
                MoveType::Normal,
            ),
            (
                Board::from_str(
                    "r1bqkbnr/pp1ppppp/2n5/2p5/2P5/1PN5/P2PPPPP/R1BQKBNR b KQkq - 0 3",
                )?,
                Board::from_str(
                    "r1bqkbnr/pp1p1ppp/2n1p3/2p5/2P5/1PN5/P2PPPPP/R1BQKBNR w KQkq - 0 4",
                )?,
                MoveType::Normal,
            ),
            (
                Board::from_str("rnbqkbnr/ppp1pppp/8/3p4/6P1/8/PPPPPP1P/RNBQKBNR w KQkq - 0 2")?,
                Board::from_str("rnbqkbnr/ppp1pppp/8/3p4/6P1/8/PPPPPPBP/RNBQK1NR b KQkq - 1 2")?,
                MoveType::Normal,
            ),
            (
                Board::from_str("rnbq1bnr/pppkpppp/8/3p4/6P1/8/PPPPPPBP/RNBQK1NR w KQ - 2 3")?,
                Board::from_str("rnbq1bnr/pppkpppp/8/3p4/6P1/8/PPPPPPBP/RNBQ1KNR b - - 3 3")?,
                MoveType::Normal,
            ),
            (
                Board::from_str("r2qkbnr/pp3ppp/2n1b3/2pp4/8/1PN5/PBQPPPPP/R3KBNR w KQkq - 2 7")?,
                Board::from_str("r2qkbnr/pp3ppp/2n1b3/2pp4/8/1PN5/PBQPPPPP/2KR1BNR b kq - 3 7")?,
                MoveType::Castle {
                    rook_from: Square::A1,
                    rook_to: Square::D1,
                },
            ),
            (
                Board::from_str(
                    "rnbqkbnr/pp4pp/2p2p2/1B1pp3/3PPP2/5N2/PPP3PP/RNBQK2R w KQkq - 0 6",
                )?,
                Board::from_str(
                    "rnbqkbnr/pp4pp/2p2p2/1B1pp3/3PPP2/5N2/PPP3PP/RNBQ1RK1 b kq - 1 6",
                )?,
                MoveType::Castle {
                    rook_from: Square::H1,
                    rook_to: Square::F1,
                },
            ),
            (
                Board::from_str("r3kbnr/pp2qppp/2n1b3/2pp4/8/1PN1P3/PBQP1PPP/2KR1BNR b kq - 0 8")?,
                Board::from_str("2kr1bnr/pp2qppp/2n1b3/2pp4/8/1PN1P3/PBQP1PPP/2KR1BNR w - - 1 9")?,
                MoveType::Castle {
                    rook_from: Square::A8,
                    rook_to: Square::D8,
                },
            ),
            (
                Board::from_str(
                    "rn1qk2r/pp2n1pp/2p2p2/1B1pp3/1b1PPPb1/2P1BN2/PP1N2PP/R2Q1RK1 b kq - 2 9",
                )?,
                Board::from_str(
                    "rn1q1rk1/pp2n1pp/2p2p2/1B1pp3/1b1PPPb1/2P1BN2/PP1N2PP/R2Q1RK1 w - - 3 10",
                )?,
                MoveType::Castle {
                    rook_from: Square::H8,
                    rook_to: Square::F8,
                },
            ),
            (
                Board::from_str(
                    "r1bqkbnr/pp3ppp/2n1p3/2pp4/2P5/1PN5/PB1PPPPP/R2QKBNR w KQkq - 0 5",
                )?,
                Board::from_str("r1bqkbnr/pp3ppp/2n1p3/2pP4/8/1PN5/PB1PPPPP/R2QKBNR b KQkq - 0 5")?,
                MoveType::Capture,
            ),
            (
                Board::from_str("r3kbnr/pp3ppp/2n1bq2/2pp4/8/1PN1P3/PBQP1PPP/2KR1BNR b kq - 0 8")?,
                Board::from_str("r3kbnr/pp3ppp/2n1b3/2pp4/8/1PN1P3/PBQP1qPP/2KR1BNR w kq - 0 9")?,
                MoveType::Capture,
            ),
            (
                Board::from_str("2kr1bnr/ppN2ppp/2n1b3/2pp4/8/1P2P3/PBQP1qPP/2KR1BNR b - - 3 10")?,
                Board::from_str("3r1bnr/ppk2ppp/2n1b3/2pp4/8/1P2P3/PBQP1qPP/2KR1BNR w - - 0 11")?,
                MoveType::Capture,
            ),
            (
                Board::from_str("3r1b1r/ppk2ppp/2n1bn2/2pp4/8/1P2P2N/PBQP1qPP/2KR1B1R w - - 2 12")?,
                Board::from_str("3r1b1r/ppk2ppp/2n1bn2/2pp4/8/1P2P3/PBQP1NPP/2KR1B1R b - - 0 12")?,
                MoveType::Capture,
            ),
            (
                Board::from_str("rnbq1bnr/pppkp1pp/8/3p1pP1/8/8/PPPPPPBP/RNBQK1NR w KQ f6 0 4")?,
                Board::from_str("rnbq1bnr/pppkp1pp/5P2/3p4/8/8/PPPPPPBP/RNBQK1NR b KQ - 0 4")?,
                MoveType::EnPassant,
            ),
            (
                Board::from_str("rnbq1bnr/pp1kp1pp/5P2/3p4/1PpP4/8/P1P1PPBP/RNBQK1NR b KQ d3 0 6")?,
                Board::from_str("rnbq1bnr/pp1kp1pp/5P2/3p4/1P6/3p4/P1P1PPBP/RNBQK1NR w KQ - 0 7")?,
                MoveType::EnPassant,
            ),
        ])
    }

    #[test]
    fn new_test() {
        Engine::new("stockfish").expect("Could not open engine.");
        Engine::new("definitely_an_engine").expect_err("This engine should not exist.");
    }

    #[test]
    fn drop_test() {
        let engine = Engine::new("stockfish").unwrap();
        let pid = engine.process.id();

        // `kill` doesn't actually kill the process.
        assert!(kill(Pid::from_raw(pid as i32), None).is_ok());
        std::mem::drop(engine);
        assert!(kill(Pid::from_raw(pid as i32), None).is_err());
    }

    #[test]
    fn get_move_test() {
        let test_position =
            |fen: &str, history: &str, last_move: &str, expected: (MoveType, &str, bool)| {
                let mut engine = Engine::new("stockfish").unwrap();

                engine.board_state = Board::from_str(fen).unwrap();
                engine.move_history = format!("position startpos moves {} ", history);

                let (choice, move_type, promotion) = engine.get_move(last_move).unwrap();
                assert_eq!(expected.0, move_type);
                assert_eq!(ChessMove::from_str(expected.1).unwrap(), choice);
                assert_eq!(expected.2, promotion);
            };

        test_position(
            "r3k2r/pp1bbppp/1qn1p3/3pP3/3P1P2/P4N2/1PB3PP/R1BQK2R w KQkq - 5 14",
            "e2e4 e7e6 d2d4 d7d5 b1d2 g8f6 e4e5 f6d7 f2f4 c7c5 g1f3 b8c6 c2c3 c5d4 f3d4 c6d4 c3d4 d8b6 d2f3 d7b8 a2a3 c8d7 f1d3 b8c6 d3c2",
            "f8e7",
            (
                MoveType::Castle {
                    rook_from: Square::H1,
                    rook_to: Square::F1,
                },
                "e1g1",
                false,
            ),
        );

        test_position(
            "8/p2R4/1p4pk/rP5p/5prP/1R4P1/7K/8 b - - 1 44",
            "e2e4 e7e6 d2d4 d7d5 b1d2 g8f6 e4e5 f6d7 f2f4 c7c5 g1f3 b8c6 c2c3 c5d4 f3d4 c6d4 c3d4 d8b6 d2f3 d7b8 a2a3 c8d7 f1d3 b8c6 d3c2 f8e7 e1g1 g7g6 a1b1 h7h5 b2b4 e8f8 c1e3 f8g7 f3g5 e7g5 f4g5 c6e5 f1f6 d7e8 e3f2 e5g4 d1f3 a8c8 c2b3 b6d8 f2g3 h8f8 f6f4 d8g5 h2h3 e6e5 d4e5 g4e5 f3d5 g5g3 d5e5 g7h6 b3d5 e8c6 d5c6 c8c6 b1f1 f8c8 e5d5 f7f5 f4f3 g3d6 f3d3 d6d5 d3d5 c6c3 d5d7 c3a3 f1f2 b7b6 b4b5 c8c5 f2b2 a3a5 h3h4 c5c4 g2g3 c4g4 b2b3 f5f4",
            "g1h2",
            (MoveType::Capture, "f4g3", false),
        );

        test_position(
            "8/p2R4/1p4pk/rP5p/6rP/4R2K/6p1/8 b - - 1 46",
            "e2e4 e7e6 d2d4 d7d5 b1d2 g8f6 e4e5 f6d7 f2f4 c7c5 g1f3 b8c6 c2c3 c5d4 f3d4 c6d4 c3d4 d8b6 d2f3 d7b8 a2a3 c8d7 f1d3 b8c6 d3c2 f8e7 e1g1 g7g6 a1b1 h7h5 b2b4 e8f8 c1e3 f8g7 f3g5 e7g5 f4g5 c6e5 f1f6 d7e8 e3f2 e5g4 d1f3 a8c8 c2b3 b6d8 f2g3 h8f8 f6f4 d8g5 h2h3 e6e5 d4e5 g4e5 f3d5 g5g3 d5e5 g7h6 b3d5 e8c6 d5c6 c8c6 b1f1 f8c8 e5d5 f7f5 f4f3 g3d6 f3d3 d6d5 d3d5 c6c3 d5d7 c3a3 f1f2 b7b6 b4b5 c8c5 f2b2 a3a5 h3h4 c5c4 g2g3 c4g4 b2b3 f5f4 g1h2 f4g3 h2h3 g3g2",
            "b3e3",
            (MoveType::Normal, "g2g1q", true),
        );
    }

    #[test]
    fn get_move_type_test() {
        let mut engine = Engine::new("stockfish").unwrap();

        for (before_move, after_move, move_type) in get_transitions().unwrap() {
            engine.board_state = before_move;
            assert_eq!(move_type, engine.get_move_type(&after_move));
        }
    }

    #[test]
    fn update_state_test() {
        let check_game_updates = |game: Vec<(Board, String)>, ends: bool| {
            let mut engine = Engine::new("stockfish").unwrap();
            let last = game.len() - if ends { 1 } else { 0 };

            for i in 1..last {
                let mut expected_move = game[i].1.clone();
                if !expected_move.chars().last().unwrap().is_numeric() {
                    expected_move.pop();
                    expected_move.push('q');
                }

                assert_eq!(
                    BoardChange::Valid(expected_move),
                    engine.update_state(
                        *game[i].0.color_combined(Color::White),
                        *game[i].0.color_combined(Color::Black),
                    )
                );
                assert_eq!(
                    BoardChange::None,
                    engine.update_state(
                        *game[i].0.color_combined(Color::White),
                        *game[i].0.color_combined(Color::Black),
                    )
                );
                assert_eq!(
                    BoardChange::Invalid,
                    engine.update_state(
                        *game[i - 1].0.color_combined(Color::White),
                        *game[i - 1].0.color_combined(Color::Black),
                    )
                );
            }

            if ends {
                assert_eq!(
                    BoardChange::GameOver,
                    engine.update_state(
                        *game.last().unwrap().0.color_combined(Color::White),
                        *game.last().unwrap().0.color_combined(Color::Black),
                    )
                )
            }
        };

        check_game_updates(opera_game(), true);
        check_game_updates(en_passsant_game(), true);
        check_game_updates(promotion_game(), false);
    }
}
