use nalgebra::{RealField, SMatrix, SVector};
use num::Float;

pub struct KalmanFilter<F, const N: usize> {
    pub state: SVector<F, N>,
    pub covariance: SMatrix<F, N, N>,
}

impl<F: RealField + Float, const N: usize> KalmanFilter<F, N> {
    pub fn new(state: SVector<F, N>, initial_variance: F) -> Self {
        Self {
            state,
            covariance: SMatrix::identity() * initial_variance,
        }
    }

    pub fn step(&mut self, state_transition: SMatrix<F, N, N>, process_noise: SMatrix<F, N, N>) {
        self.state = state_transition * self.state;
        self.covariance =
            state_transition * self.covariance * state_transition.transpose() + process_noise;
    }

    /// Process an observation assuming white observation noise.
    pub fn observe_uncorrelated<const M: usize>(
        &mut self,
        sample: &[F; M],
        observation_model: SMatrix<F, M, N>,
        sample_variance: &[F; M],
    ) {
        for i in 0..M {
            let h = observation_model.row(i);
            let y = sample[i] - (h * self.state)[0];
            let r = sample_variance[i];
            let s = (h * self.covariance * h.transpose())[0] + r;
            let k = self.covariance * h.transpose() / s;
            self.state += k * y;
            self.covariance = (SMatrix::identity() - k * h) * self.covariance;
        }
    }
}

/// Simple kalman filter for position, velocity, and acceleration assuming constant acceleration
pub struct Pva2d<F> {
    filter: KalmanFilter<F, 6>,
    accel_variance: F,
    dt: F,
}

impl<F: RealField + Float> Pva2d<F> {
    pub fn new(accel_variance: F, dt: F) -> Self {
        Self {
            filter: KalmanFilter::new(SVector::zeros(), F::from(1e9).unwrap()),
            accel_variance,
            dt,
        }
    }

    pub fn step(&mut self) {
        let t = self.dt;
        let _1 = F::one();
        let _0 = F::zero();
        #[rustfmt::skip]
        self.filter.step(
            // state transition
            SMatrix::from_row_slice(&[
                _1, _0, t, _0, t*t, _0,
                _0, _1, _0, t, _0, t*t,
                _0, _0, _1, _0, t, _0,
                _0, _0, _0, _1, _0, t,
                _0, _0, _0, _0, _1, _0,
                _0, _0, _0, _0, _0, _1,
            ]),
            // process noise
            SMatrix::from_row_slice(&[
                _0, _0, _0, _0, _0, _0,
                _0, _0, _0, _0, _0, _0,
                _0, _0, _0, _0, _0, _0,
                _0, _0, _0, _0, _0, _0,
                _0, _0, _0, _0, self.accel_variance, _0,
                _0, _0, _0, _0, _0, self.accel_variance,
            ])
        );
    }

    pub fn observe(&mut self, position: &[F; 2], variance: &[F; 2]) {
        let _1 = F::one();
        let _0 = F::zero();
        #[rustfmt::skip]
        self.filter.observe_uncorrelated(
            position,
            SMatrix::from([
                [_1, _0],
                [_0, _1],
                [_0, _0],
                [_0, _0],
                [_0, _0],
                [_0, _0],
            ]),
            variance,
        );
    }

    pub fn position(&self) -> [F; 2] {
        [self.filter.state[0], self.filter.state[1]]
    }
    pub fn velocity(&self) -> [F; 2] {
        [self.filter.state[2], self.filter.state[3]]
    }
    pub fn acceleration(&self) -> [F; 2] {
        [self.filter.state[4], self.filter.state[5]]
    }
}

impl Default for Pva2d<f64> {
    fn default() -> Self { Pva2d::new(0.4, 0.4) }
}
