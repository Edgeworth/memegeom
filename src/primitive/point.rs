use approx::{AbsDiffEq, RelativeEq};
use auto_ops::{impl_op_ex, impl_op_ex_commutative};
use derive_more::Display;
use nalgebra::{Vector2, vector};
use serde::{Deserialize, Serialize};

use crate::geom::contains::{cap_contains_pt, circ_contains_pt, poly_contains_pt};
use crate::geom::distance::{line_pt_dist, poly_pt_dist, pt_pt_dist, pt_rt_dist, pt_seg_dist};
use crate::primitive::rect::RtPrimitive;
use crate::primitive::shape::Shape;
use crate::primitive::{Boundary, Rt, ShapeOps, pt, pti, rt};

#[must_use]
#[derive(Debug, Default, PartialEq, Copy, Clone, Display, Serialize, Deserialize)]
#[display("({x}, {y})")]
pub struct Pt {
    pub x: f64,
    pub y: f64,
}

impl Eq for Pt {}

impl Pt {
    pub const fn new(x: f64, y: f64) -> Self {
        assert!(x.is_finite() && y.is_finite(), "point coordinates must be finite");
        Self { x, y }
    }

    pub const fn zero() -> Self {
        Self::new(0.0, 0.0)
    }

    #[must_use]
    pub fn is_zero(&self) -> bool {
        *self == Self::zero()
    }

    #[must_use]
    pub fn as_array(&self) -> [f64; 2] {
        [self.x, self.y]
    }

    pub fn offset(&self, dx: f64, dy: f64) -> Pt {
        pt(self.x + dx, self.y + dy)
    }

    #[must_use]
    pub fn cross(&self, p: Pt) -> f64 {
        self.x * p.y - self.y * p.x
    }

    // Gets the normalized perpendicular (to the right). Returns None for zero vectors.
    #[must_use]
    pub fn perp(&self) -> Option<Pt> {
        pt(-self.y, self.x).norm()
    }

    #[must_use]
    pub fn dist(&self, b: Pt) -> f64 {
        (b - *self).mag()
    }

    #[must_use]
    pub fn mag(&self) -> f64 {
        self.mag2().sqrt()
    }

    #[must_use]
    pub fn mag2(&self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    #[must_use]
    pub fn dot(&self, p: Pt) -> f64 {
        self.x * p.x + self.y * p.y
    }

    #[must_use]
    pub fn norm(&self) -> Option<Pt> {
        let mag = self.mag();
        if mag == 0.0 { None } else { Some(pt(self.x / mag, self.y / mag)) }
    }

    // Clamps the point to be in the range defined by |r|.
    pub fn clamp<const B: Boundary>(&self, r: &RtPrimitive<B>) -> Pt {
        pt(self.x.clamp(r.l(), r.r()), self.y.clamp(r.b(), r.t()))
    }
}

impl AbsDiffEq for Pt {
    type Epsilon = f64;

    fn default_epsilon() -> f64 {
        f64::default_epsilon()
    }

    fn abs_diff_eq(&self, o: &Self, epsilon: f64) -> bool {
        f64::abs_diff_eq(&self.x, &o.x, epsilon) && f64::abs_diff_eq(&self.y, &o.y, epsilon)
    }
}

impl RelativeEq for Pt {
    fn default_max_relative() -> f64 {
        f64::default_max_relative()
    }

    fn relative_eq(&self, o: &Self, epsilon: f64, max_relative: f64) -> bool {
        f64::relative_eq(&self.x, &o.x, epsilon, max_relative)
            && f64::relative_eq(&self.y, &o.y, epsilon, max_relative)
    }
}

impl From<Pt> for Vector2<f64> {
    fn from(p: Pt) -> Self {
        vector![p.x, p.y]
    }
}

impl ShapeOps for Pt {
    fn bounds(&self) -> Option<Rt> {
        Some(rt(self.x, self.y, self.x, self.y))
    }

    fn shape(self) -> Shape {
        Shape::Point(self)
    }

    fn is_empty_set(&self) -> bool {
        false // A point is never empty
    }

    fn intersects_shape(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(s) => cap_contains_pt(s, self),
            Shape::CapsuleExcl(s) => cap_contains_pt(s, self),
            Shape::Circle(s) => circ_contains_pt(s, self),
            Shape::CircleExcl(s) => circ_contains_pt(s, self),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(_) => todo!(),
            Shape::Poly(s) => poly_contains_pt(s, self),
            Shape::PolyExcl(s) => poly_contains_pt(s, self),
            Shape::Rect(s) => s.contains(*self),
            Shape::RectExcl(s) => s.contains(*self),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }

    fn contains_shape(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(_) | Shape::CapsuleExcl(_) => todo!(),
            Shape::Circle(_) | Shape::CircleExcl(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(_) => todo!(),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(_) => todo!(),
            Shape::RectExcl(_) => todo!(),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }

    fn dist_to_shape(&self, s: &Shape) -> Option<f64> {
        match s {
            Shape::Capsule(_) | Shape::CapsuleExcl(_) => todo!(),
            Shape::Circle(_) | Shape::CircleExcl(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(s) => Some(line_pt_dist(s, self)),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(s) => Some(pt_pt_dist(self, s)),
            Shape::Poly(s) => poly_pt_dist(s, self),
            Shape::PolyExcl(s) => poly_pt_dist(s, self),
            Shape::Rect(s) => pt_rt_dist(self, s),
            Shape::RectExcl(s) => pt_rt_dist(self, s),
            Shape::Segment(s) => Some(pt_seg_dist(self, s)),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }
}

impl_op_ex!(-|a: &Pt| -> Pt { pt(-a.x, -a.y) });

impl_op_ex!(+ |a: &Pt, b: &Pt| -> Pt { pt(a.x + b.x, a.y + b.y) });
impl_op_ex!(+= |a: &mut Pt, b: &Pt| { a.x += b.x; a.y += b.y; });
impl_op_ex!(-|a: &Pt, b: &Pt| -> Pt { pt(a.x - b.x, a.y - b.y) });
impl_op_ex!(-= |a: &mut Pt, b: &Pt| { a.x -= b.x; a.y -= b.y; });

impl_op_ex_commutative!(*|a: &Pt, b: &f64| -> Pt { pt(a.x * b, a.y * b) });
impl_op_ex_commutative!(/|a: &Pt, b: &f64| -> Pt { pt(a.x / b, a.y / b) });

#[must_use]
#[derive(Debug, Default, PartialEq, Eq, Hash, Copy, Clone, Display, Serialize, Deserialize)]
#[display("({x}, {y})")]
pub struct PtI {
    pub x: i64,
    pub y: i64,
}

impl PtI {
    pub const fn new(x: i64, y: i64) -> Self {
        Self { x, y }
    }

    pub const fn zero() -> Self {
        Self::new(0, 0)
    }

    #[must_use]
    pub fn is_zero(&self) -> bool {
        *self == Self::zero()
    }

    #[must_use]
    pub fn dist(&self, b: PtI) -> f64 {
        (b - *self).mag()
    }

    #[must_use]
    pub fn mag(&self) -> f64 {
        (self.mag2() as f64).sqrt()
    }

    #[must_use]
    pub fn mag2(&self) -> i64 {
        self.x * self.x + self.y * self.y
    }
}

impl_op_ex!(-|a: &PtI| -> PtI { pti(-a.x, -a.y) });

impl_op_ex!(+ |a: &PtI, b: &PtI| -> PtI { pti(a.x + b.x, a.y + b.y) });
impl_op_ex!(+= |a: &mut PtI, b: &PtI| { a.x += b.x; a.y += b.y; });
impl_op_ex!(-|a: &PtI, b: &PtI| -> PtI { pti(a.x - b.x, a.y - b.y) });
impl_op_ex!(-= |a: &mut PtI, b: &PtI| { a.x -= b.x; a.y -= b.y; });

impl_op_ex_commutative!(*|a: &PtI, b: &i64| -> PtI { pti(a.x * b, a.y * b) });
impl_op_ex_commutative!(/|a: &PtI, b: &i64| -> PtI { pti(a.x / b, a.y / b) });

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn norm_zero_vector_returns_none() {
        let zero = pt(0.0, 0.0);
        assert!(zero.norm().is_none());
    }

    #[test]
    fn norm_non_zero_vector_returns_some() {
        let v = pt(3.0, 4.0);
        let n = v.norm().unwrap();
        assert_relative_eq!(n.mag(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(n.x, 0.6, epsilon = 1e-10);
        assert_relative_eq!(n.y, 0.8, epsilon = 1e-10);
    }

    #[test]
    fn norm_small_vector() {
        let v = pt(1e-100, 0.0);
        let n = v.norm().unwrap();
        assert_relative_eq!(n.mag(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn perp_zero_vector_returns_none() {
        let zero = pt(0.0, 0.0);
        assert!(zero.perp().is_none());
    }

    #[test]
    fn perp_non_zero_vector_returns_some() {
        let v = pt(1.0, 0.0);
        let p = v.perp().unwrap();
        // Perpendicular to (1, 0) is (0, 1) or (0, -1)
        assert_relative_eq!(p.mag(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(v.dot(p), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn perp_is_normalized() {
        let v = pt(3.0, 4.0);
        let p = v.perp().unwrap();
        assert_relative_eq!(p.mag(), 1.0, epsilon = 1e-10);
    }
}
