///! Segment abstraction + concrete Line / Arc
use nalgebra::{Matrix3, Vector3};

pub trait Segment {
    fn center(&self) -> Vector3<f32>;
    fn inertia(&self) -> Matrix3<f32>;
    fn mass(&self) -> f32;
}

pub struct LineSeg {
    pub start: Vector3<f32>,
    pub end: Vector3<f32>,
    pub mass: f32,
}
pub struct ArcSeg {
    pub center: Vector3<f32>,
    pub radius: f32,
    pub start_ang: f32,
    pub delta_ang: f32,
    pub mass: f32,
}

impl Segment for LineSeg {
    fn center(&self) -> Vector3<f32> {
        (self.start + self.end) * 0.5
    }
    fn mass(&self) -> f32 {
        self.mass
    }

    fn inertia(&self) -> Matrix3<f32> {
        let m = self.mass;
        let r0 = self.start;
        let u = self.end - self.start;
        let outer = |a: Vector3<f32>, b: Vector3<f32>| a * b.transpose();
        let c = r0.dot(&r0) + r0.dot(&u) + u.dot(&u) / 3.0;
        let q = outer(r0, r0) + outer(r0, u) * 0.5 + outer(u, r0) * 0.5 + outer(u, u) / 3.0;
        Matrix3::identity() * (m * c) - q * m
    }
}

impl Segment for ArcSeg {
    fn center(&self) -> Vector3<f32> {
        let θ0 = self.start_ang;
        let θ1 = θ0 + self.delta_ang;
        let dθ = self.delta_ang;
        let c_int = θ1.sin() - θ0.sin();
        let s_int = -θ1.cos() + θ0.cos();
        Vector3::new(
            self.center.x + self.radius * c_int / dθ,
            self.center.y + self.radius * s_int / dθ,
            self.center.z,
        )
    }
    fn mass(&self) -> f32 {
        self.mass
    }

    fn inertia(&self) -> Matrix3<f32> {
        let θ0 = self.start_ang;
        let dθ = self.delta_ang;
        let θ1 = θ0 + dθ;
        let λ = self.mass / (self.radius * dθ);
        let (s20, s21, c20, c21) = (
            (2.0 * θ0).sin(),
            (2.0 * θ1).sin(),
            (2.0 * θ0).cos(),
            (2.0 * θ1).cos(),
        );
        let ic2 = 0.5 * dθ + 0.25 * (s21 - s20); // ∫cos²
        let is2 = 0.5 * dθ - 0.25 * (s21 - s20); // ∫sin²
        let ic = θ1.sin() - θ0.sin(); // ∫cos
        let is_ = -θ1.cos() + θ0.cos(); // ∫sin
        let ics = -0.25 * (c21 - c20); // ∫cos·sin

        let (cx, cy, cz, r) = (self.center.x, self.center.y, self.center.z, self.radius);

        let a = cx * cx * dθ + 2.0 * cx * r * ic + r * r * ic2;
        let b = cy * cy * dθ + 2.0 * cy * r * is_ + r * r * is2;
        let c = cz * cz * dθ;

        let ab = cx * cy * dθ + cx * r * is_ + cy * r * ic + r * r * ics;
        let ac = cx * cz * dθ + r * cz * ic;
        let bc = cy * cz * dθ + r * cz * is_;

        let mut q = Matrix3::zeros();
        q[(0, 0)] = a;
        q[(1, 1)] = b;
        q[(2, 2)] = c;
        q[(0, 1)] = ab;
        q[(1, 0)] = ab;
        q[(0, 2)] = ac;
        q[(2, 0)] = ac;
        q[(1, 2)] = bc;
        q[(2, 1)] = bc;

        let sum_r2 = a + b + c;
        Matrix3::identity() * (λ * r * sum_r2) - q * (λ * r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    const EPS: f32 = 1e-5;

    #[test]
    fn test_line_seg_center_mass_inertia() {
        let start = Vector3::new(0.0, 0.0, 0.0);
        let end = Vector3::new(1.0, 0.0, 0.0);
        let mass = 2.0;
        let seg = LineSeg { start, end, mass };
        assert_eq!(seg.mass(), mass);
        // center at midpoint
        let c = seg.center();
        assert!((c.x - 0.5).abs() < EPS);
        assert!((c.y).abs() < EPS);
        assert!((c.z).abs() < EPS);
        // inertia: [0, 2/3, 2/3]
        let i = seg.inertia();
        assert!((i[(0, 0)] - 0.0).abs() < EPS);
        assert!((i[(1, 1)] - (2.0 / 3.0)).abs() < EPS);
        assert!((i[(2, 2)] - (2.0 / 3.0)).abs() < EPS);
        assert!((i[(0, 1)]).abs() < EPS);
        assert!((i[(0, 2)]).abs() < EPS);
        assert!((i[(1, 2)]).abs() < EPS);
    }

    #[test]
    fn test_arc_seg_center_mass_full_circle() {
        let center = Vector3::new(0.0, 0.0, 0.0);
        let radius = 1.0;
        let start_ang = 0.0;
        let delta_ang = std::f32::consts::PI * 2.0;
        let mass = 4.2;
        let seg = ArcSeg {
            center,
            radius,
            start_ang,
            delta_ang,
            mass,
        };
        assert_eq!(seg.mass(), mass);
        // center remains at origin
        let c = seg.center();
        assert!((c.x).abs() < EPS);
        assert!((c.y).abs() < EPS);
        assert!((c.z).abs() < EPS);
        // inertia diag == [mass/2, mass/2, mass]
        let i = seg.inertia();
        assert!((i[(0, 0)] - mass * 0.5).abs() < EPS);
        assert!((i[(1, 1)] - mass * 0.5).abs() < EPS);
        assert!((i[(2, 2)] - mass).abs() < EPS);
        // symmetric
        assert!((i - i.transpose()).norm() < EPS);
    }

    #[test]
    fn test_arc_seg_center_semicircle() {
        let center = Vector3::new(1.0, 2.0, 3.0);
        let radius = 2.0;
        let start_ang = 0.0;
        let delta_ang = std::f32::consts::PI;
        let mass = 6.0;
        let seg = ArcSeg {
            center,
            radius,
            start_ang,
            delta_ang,
            mass,
        };
        // expected center: original + [0, 2*radius/pi]
        let c = seg.center();
        let expected = Vector3::new(1.0, 2.0 + (2.0 * 2.0) / std::f32::consts::PI, 3.0);
        assert!((c - expected).norm() < EPS);
    }
}
