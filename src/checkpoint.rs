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
    /// Add two coordinates together.
    pub fn add(self, other: Self) -> Self {
        Coordinate(self.0 + other.0, self.1 + other.1, self.2 + other.2)
    }

    /// Scale a coordinate by a given factor.
    pub fn mul(self, scale: f32) -> Self {
        Coordinate(self.0 * scale, self.1 * scale, self.2 * scale)
    }
}

/// Calculates the expected position of the arm a fraction `time_f`
/// along the path's duration (derivation in the README).
pub fn fraction_along(from: Checkpoint, to: Checkpoint, time_f: f32) -> Coordinate {
    let path_f = 2.
        * (from.speed * time_f + (to.speed - from.speed) * (time_f.powi(3) - 0.5 * time_f.powi(4)))
        / (from.speed + to.speed);
    from.position.mul(1. - path_f).add(to.position.mul(path_f))
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
    const FRACTIONAL_POSITIONS: [[Coordinate; 3]; 4] = [
        [
            Coordinate(3.204297, -3.495040, 4.626098),
            Coordinate(4.077083, -1.337775, 3.028734),
            Coordinate(5.166255, 1.354329, 1.035344),
        ],
        [
            Coordinate(24.589063, 8.761719, 0.651563),
            Coordinate(37.380263, 8.496079, 4.853947),
            Coordinate(43.253216, 8.374113, 6.783431),
        ],
        [
            Coordinate(46.487599, 7.828255, 8.080558),
            Coordinate(37.151893, 6.749814, 5.636748),
            Coordinate(26.508804, 5.520347, 2.850704),
        ],
        [
            Coordinate(-5.505014, 1.176335, -3.192092),
            Coordinate(-7.504406, 0.408453, -1.772234),
            Coordinate(-8.529508, 0.014754, -1.044262),
        ],
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
    fn add_test() {
        let sum = COORDINATES
            .iter()
            .fold(Coordinate(0.0, 0.0, 0.0), |acc, x| acc.add(*x));

        assert_eq!(sum, Coordinate(48.2, 14.884, 4.4));
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
        let test_window = |window: (&[Checkpoint], [Coordinate; 3])| {
            let (seg, pts) = window;

            assert_eq!(fraction_along(seg[0], seg[1], 0.0), seg[0].position);
            assert_eq!(fraction_along(seg[0], seg[1], 1.0), seg[1].position);
            assert_eq!(fraction_along(seg[0], seg[1], 0.25), pts[0]);
            assert_eq!(fraction_along(seg[0], seg[1], 0.5), pts[1]);
            assert_eq!(fraction_along(seg[0], seg[1], 2.0 / 3.0), pts[2]);
        };

        make_checkpoints()
            .windows(2)
            .zip(FRACTIONAL_POSITIONS)
            .for_each(test_window);
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
