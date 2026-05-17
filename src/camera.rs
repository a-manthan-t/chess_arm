use std::error::Error;
use std::fmt::Display;

use chess::{BitBoard, Board, Color};
use opencv::core::{
    Mat, MatTraitConst, Point2i, Rect2i, Scalar, Size, Vec2f, Vector, mean_std_dev_def,
};
use opencv::imgproc::{
    COLOR_BGR2GRAY as GREY, COLOR_BGR2HSV as HSV, LINE_8, canny_def, cvt_color_def,
    gaussian_blur_def, hough_lines_def, rectangle,
};
use opencv::videoio::{CAP_ANY, VideoCapture, VideoCaptureTrait, VideoCaptureTraitConst};

/// Various errors that can occur during camera configuration and image processing.
#[derive(Debug, PartialEq)]
pub enum CameraError {
    InvalidSource,
    OpenCvError(String),
    UnclearBoard(usize),
    NotStartingPosition,
}

impl Display for CameraError {
    /// For printing CameraError as a normal error.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for CameraError {}

impl From<opencv::Error> for CameraError {
    /// A conversion from OpenCV's `Error` to `CameraError`.
    fn from(value: opencv::Error) -> Self {
        CameraError::OpenCvError(value.message)
    }
}

/// The camera object.
#[derive(Debug)]
pub struct Camera {
    camera: VideoCapture,
    pub last_frame: Mat,
    squares: [Rect2i; 64],
}

const OCCUPIED_THRESHOLD: f64 = 20.0;
const WHITE_THRESHOLD: f64 = 90.0;

/// A helper function transforming a chess board into the white and black pieces' bitboards.
fn get_bitboards(board: Board) -> (BitBoard, BitBoard) {
    (
        *board.color_combined(Color::White),
        *board.color_combined(Color::Black),
    )
}

impl Camera {
    /// Open a camera using a given source, which may be a GStreamer
    /// pipeline, a video file name, an image sequence, or a URL.
    pub fn open(source: &str) -> Result<Camera, CameraError> {
        let camera = VideoCapture::from_file(source, CAP_ANY)?;

        camera
            .is_opened()?
            .then(|| Camera {
                camera,
                last_frame: Mat::default(),
                squares: [Rect2i::default(); 64],
            })
            .ok_or(CameraError::InvalidSource)
    }

    /// Set up the camera to play a game by detecting the chess board.
    pub fn configure(&mut self) -> Result<Color, CameraError> {
        // Take camera input.
        while !matches!(self.camera.read(&mut self.last_frame), Ok(true)) {}

        // Extract grid lines.
        let (mut grey, mut blur, mut edges) = (Mat::default(), Mat::default(), Mat::default());
        let mut lines: Vector<Vec2f> = Vector::new();

        cvt_color_def(&self.last_frame, &mut grey, GREY)?;
        gaussian_blur_def(&grey, &mut blur, Size::new(5, 5), 0.0)?;
        canny_def(&blur, &mut edges, 50.0, 150.0)?;
        hough_lines_def(&edges, &mut lines, 1.0, 1.0_f64.to_radians(), 100)?;

        let (error, deg_90) = (2.0_f32.to_radians(), 90_f32.to_radians());
        let horizontal = |vec: &Vec2f| deg_90 - error < vec[1] && vec[1] < deg_90 + error;
        let vertical = |vec: &Vec2f| vec[1] < error || vec[1] > 180_f32.to_radians() - error;

        let mut points: Vec<Point2i> = Vec::new();
        let (hlines, vlines): (Vec<Vec2f>, Vec<Vec2f>) = lines
            .into_iter()
            .filter(|x| horizontal(x) || vertical(x))
            .partition(horizontal);

        // Find intersection points (corners of board squares).
        for h in &hlines {
            for v in &vlines {
                let p = Point2i::new(
                    (h[0] * v[1].sin() - v[0] * h[1].sin()).abs() as i32,
                    (v[0] * h[1].cos() - h[0] * v[1].cos()).abs() as i32,
                );

                let different = |&q: &Point2i| (p - q).norm() >= 25.0;
                if points.iter().all(different) {
                    points.push(p);
                }
            }
        }

        if !matches!(points.len(), 81 | 121) {
            return Err(CameraError::UnclearBoard(points.len()));
        }

        // Ensure there are exactly as many points as needed to generate the
        // board squares, removing an external border if it exists. Note a grid
        // of points is 9x9 on an 8x8 chessboard, and a boarder makes it 11x11.
        points.sort_by_key(|p| p.y);

        if points.len() == 81 {
            for chunk in points.as_chunks_mut::<9>().0 {
                chunk.sort_by_key(|p| p.x)
            }
        } else {
            for chunk in points.as_chunks_mut::<11>().0 {
                chunk.sort_by_key(|p| p.x)
            }

            points = points
                .chunks(11)
                .skip(1)
                .take(9)
                .flat_map(|rank| rank.iter().skip(1).take(9))
                .copied()
                .collect();
        }

        // Convert the corners into rectangles in h8 to a1 order (from our perspective).
        for i in 0..8 {
            for j in 0..8 {
                let tl = points[i * 9 + j]; // Top left corner of square.
                let br = points[(i + 1) * 9 + (j + 1)]; // Bottom right corner of square.
                let diag = (br - tl) / 5;

                self.squares[i * 8 + (7 - j)] = Rect2i::from_points(tl + diag, br - diag);
            }
        }

        // Ensure the pieces are in their starting position.
        let processed = self.process_frame()?;
        let flipped = (processed.0.reverse_colors(), processed.1.reverse_colors());
        let expected = get_bitboards(Board::default());

        if processed != expected && flipped != expected {
            return Err(CameraError::NotStartingPosition);
        }

        if processed == expected {
            Ok(Color::White)
        } else {
            self.squares.reverse(); // So processing moves from h8 to a1.
            Ok(Color::Black)
        }
    }

    /// Classify, after taking a birds eye view of the chess board, the regions
    /// mapped by the squares from configuration as unoccupied or containing a
    /// white or black piece to produce white and black piece bitboards.
    pub fn process_frame(&mut self) -> Result<(BitBoard, BitBoard), CameraError> {
        while !matches!(self.camera.read(&mut self.last_frame), Ok(true)) {}

        let mut hsv = Mat::default();
        let (mut mean, mut stddev) = (Scalar::default(), Scalar::default());
        let (mut white_bitboard, mut black_bitboard) = (BitBoard::default(), BitBoard::default());

        cvt_color_def(&self.last_frame, &mut hsv, HSV)?;

        for square in self.squares {
            mean_std_dev_def(&hsv.roi(square)?, &mut mean, &mut stddev)?;

            white_bitboard.0 <<= 1;
            black_bitboard.0 <<= 1;

            if stddev[2] > OCCUPIED_THRESHOLD {
                let colour = if (mean[0] + mean[1] + mean[2]) / 3.0 > WHITE_THRESHOLD {
                    white_bitboard.0 += 1;
                    Scalar::new(255.0, 255.0, 255.0, 0.0) // White in BGR.
                } else {
                    black_bitboard.0 += 1;
                    Scalar::new(0.0, 0.0, 0.0, 0.0) // Black in BGR.
                };

                rectangle(&mut self.last_frame, square, colour, 2, LINE_8, 0)?;
            }
        }

        Ok((white_bitboard, black_bitboard))
    }
}

#[cfg(test)]
mod tests {
    use std::str::FromStr;

    use chess::{Board, ChessMove, Color};
    use opencv::imgcodecs::imwrite_def;

    use crate::camera::{Camera, get_bitboards};

    #[test]
    fn open_test() {
        Camera::open("test_res/move%d.jpg").expect("Could not open camera.");
        Camera::open("hello").expect_err("This camera should not exist");
    }

    #[test]
    fn configure_test() {
        let mut camera = Camera::open("test_res/move%d.jpg").unwrap();
        let result = camera.configure();
        imwrite_def("test_out/configure_normal.jpg", &camera.last_frame).unwrap();
        assert_eq!(Ok(Color::White), result);

        let mut camera = Camera::open("test_res/flipped_move%d.jpg").unwrap();
        let result = camera.configure();
        imwrite_def("test_out/configure_flipped.jpg", &camera.last_frame).unwrap();
        assert_eq!(Ok(Color::Black), result);

        let mut camera = Camera::open("test_res/bad%d.jpg").unwrap();
        let result = camera.configure();
        imwrite_def("test_out/configure_bad_normal.jpg", &camera.last_frame).unwrap();
        assert!(result.is_err());

        let mut camera = Camera::open("test_res/bad_flipped%d.jpg").unwrap();
        let result = camera.configure();
        imwrite_def("test_out/configure_bad_flipped.jpg", &camera.last_frame).unwrap();
        assert!(result.is_err());
    }

    #[test]
    fn process_frame_test() {
        let mut camera = Camera::open("test_res/move%d.jpg").unwrap();
        let mut board = Board::default();

        camera.configure().expect("Check configure.");

        let mut test = |uci: &str, file: &str| {
            board = board.make_move_new(ChessMove::from_str(uci).unwrap());
            let result = camera.process_frame().unwrap();
            imwrite_def(format!("test_out/{file}.jpg").as_str(), &camera.last_frame).unwrap();
            assert_eq!(get_bitboards(board), result);
        };

        test("e2e4", "move2");
        test("c7c5", "move3");
        test("g1f3", "move4");
        test("d7d6", "move5");
    }
}
