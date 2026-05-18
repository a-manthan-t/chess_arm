/// Represents a point in 3D space `(x, y, z)`.
#[derive(Copy, Clone, Debug)]
pub struct Coordinate(pub f32, pub f32, pub f32);

/// Represents a target for the robot arm. Speed is in units/second.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Checkpoint {
    pub position: Coordinate,
    pub speed: f32,
}

impl PartialEq for Coordinate {
    /// Coordinates are equal if their difference is less than 10e-6. (for floating point errors).
    fn eq(&self, other: &Self) -> bool {
        (self.0 - other.0).abs() < 10e-6
            && (self.1 - other.1).abs() < 10e-6
            && (self.2 - other.2).abs() < 10e-6
    }
}

impl Coordinate {
    /// Scale a coordinate by a given factor.
    pub fn mul(self, scale: f32) -> Self {
        Coordinate(self.0 * scale, self.1 * scale, self.2 * scale)
    }
}

/// Calculates the expected fraction of the servo angles between their start and end
/// values a fraction `time_f` along the path's duration (derivation in the README).
pub fn fraction_along(from: f32, to: f32, time_f: f32) -> f32 {
    2. * (from * time_f + (to - from) * (time_f.powi(3) - 0.5 * time_f.powi(4))) / (from + to)
}

/// Calculates the duration of the path between the two checkpoints in milliseconds
/// using `t = 1000 * s/v`. Where `s` is the distance travelled and `v` is the average
/// speed between the checkpoints, which turns out to be the average of the speeds at
/// each checkpoint (derivation in the README).
pub fn path_time(from: Checkpoint, to: Checkpoint) -> f32 {
    let distance = ((from.position.0 - to.position.0).powi(2)
        + (from.position.1 - to.position.1).powi(2)
        + (from.position.2 - to.position.2).powi(2))
    .sqrt();
    2000. * distance / (to.speed + from.speed)
}

#[cfg(test)]
mod tests {
    use crate::checkpoint::{Checkpoint, Coordinate, fraction_along, path_time};

    const SCALES: [f32; 5] = [0.24, 9.3, -2.1, -22.0, 8.5];
    const COORDINATES: [Coordinate; 5] = [
        Coordinate(3.0, -4.0, 5.0),
        Coordinate(8.3, 9.1, -4.7),
        Coordinate(50.0, 8.234, 9.0),
        Coordinate(-3.1, 2.1, -4.9),
        Coordinate(-10.0, -0.55, -0.0),
    ];

    // Turns pairs of coordinates (positions) and abs(scales) (speeds) into checkpoints.
    fn make_checkpoints() -> Vec<Checkpoint> {
        COORDINATES
            .iter()
            .zip(SCALES.map(|x| x.abs()))
            .map(|(&position, speed)| Checkpoint { position, speed })
            .collect::<Vec<_>>()
    }

    #[test]
    fn eq_ne_test() {
        assert_eq!(COORDINATES[0], COORDINATES[0]);
        assert_eq!(COORDINATES[2], COORDINATES[2]);
        assert_eq!(COORDINATES[4], COORDINATES[4].mul(0.23).mul(1.0 / 0.23));
        assert_eq!(COORDINATES[2].mul(-4.12).mul(-1.0 / 4.12), COORDINATES[2]);

        assert_ne!(COORDINATES[0], COORDINATES[3]);
        assert_ne!(COORDINATES[2], COORDINATES[1]);
        assert_ne!(COORDINATES[4], COORDINATES[0]);

        assert_ne!(COORDINATES[2], COORDINATES[2].mul(1.001));
        assert_ne!(COORDINATES[4].mul(0.999), COORDINATES[4].mul(1.001));
    }

    #[test]
    fn mul_test() {
        let mul: Vec<Coordinate> = COORDINATES
            .iter()
            .zip(SCALES)
            .map(|(x, y)| x.mul(y))
            .collect();

        assert_eq!(
            mul,
            vec![
                Coordinate(0.72, -0.96, 1.2),
                Coordinate(77.19, 84.63, -43.71),
                Coordinate(-105.0, -17.2914, -18.9),
                Coordinate(68.2, -46.2, 107.8),
                Coordinate(-85.0, -4.675, -0.0),
            ]
        );
    }

    #[test]
    fn fraction_along_test() {
        let from = 0.0;
        let to = 15.0;
        let fractions = [
            0.0, 0.0019, 0.0144, 0.0459, 0.1024, 0.1875, 0.3024, 0.4459, 0.6144, 0.8019, 1.0,
        ];

        #[allow(clippy::needless_range_loop)]
        for i in 0..fractions.len() {
            assert!((fractions[i] - fraction_along(from, to, i as f32 / 10.0)).abs() < 10e-4);
        }
    }

    #[test]
    fn path_time_test() {
        let times: Vec<f32> = make_checkpoints()
            .windows(2)
            .map(|window| path_time(window[0], window[1]))
            .collect();

        assert_eq!(times, vec![3593.3552, 7701.9946, 4583.4727, 581.5122]);
    }
}
