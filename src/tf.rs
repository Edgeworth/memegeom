use std::f64::consts::PI;

use auto_ops::impl_op_ex;
use nalgebra::{Matrix3, vector};

use crate::geom::math::eq;
use crate::primitive::capsule::CapsulePrimitive;
use crate::primitive::circle::CirclePrimitive;
use crate::primitive::path_shape::PathPrimitive;
use crate::primitive::point::Pt;
use crate::primitive::polygon::PolyPrimitive;
use crate::primitive::rect::RtPrimitive;
use crate::primitive::shape::Shape;
use crate::primitive::triangle::TriPrimitive;
use crate::primitive::{
    Boundary, Line, Rt, RtExcl, Segment, ShapeOps, line as prim_line, poly, poly_excl, pt,
    seg as prim_seg,
};

#[must_use]
#[derive(Debug, Default, PartialEq, Copy, Clone)]
pub struct Tf {
    m: Matrix3<f64>,
}

impl Tf {
    pub fn new() -> Self {
        Self::identity()
    }

    pub fn identity() -> Self {
        Self { m: Matrix3::identity() }
    }

    pub fn scale(p: Pt) -> Self {
        Self { m: Matrix3::new_nonuniform_scaling(&p.into()) }
    }

    pub fn translate(p: Pt) -> Self {
        Self { m: Matrix3::new_translation(&p.into()) }
    }

    pub fn rotate(deg: f64) -> Self {
        Self { m: Matrix3::new_rotation(deg / 180.0 * PI) }
    }

    #[must_use]
    pub fn affine<const B: Boundary>(from: &RtPrimitive<B>, to: &RtPrimitive<B>) -> Option<Self> {
        if eq(from.w(), 0.0) || eq(from.h(), 0.0) {
            return None;
        }
        let xscale = to.w() / from.w();
        let yscale = to.h() / from.h();
        let scale = Self::scale(pt(xscale, yscale));
        let offset = to.bl() - scale.pt(from.bl());
        Some(Self::translate(offset) * scale)
    }

    #[must_use]
    pub fn inv(&self) -> Option<Tf> {
        self.m.try_inverse().map(|m| Tf { m })
    }

    pub fn pt(&self, p: Pt) -> Pt {
        let v = self.m * vector![p.x, p.y, 1.0];
        pt(v.x, v.y)
    }

    // If there's a rotation, output will be a polygon not a Rt.
    pub fn rt(&self, r: &Rt) -> Shape {
        if eq(self.m[(1, 0)], 0.0) && eq(self.m[(0, 1)], 0.0) {
            let a = self.pt(r.bl());
            let b = self.pt(r.tr());
            Rt::enclosing(a, b).shape()
        } else {
            let p = poly(&r.pts());
            self.poly(&p).shape()
        }
    }

    // If there's a rotation, output will be a polygon not a RtExcl.
    pub fn rt_excl(&self, r: &RtExcl) -> Shape {
        if eq(self.m[(1, 0)], 0.0) && eq(self.m[(0, 1)], 0.0) {
            let a = self.pt(r.bl());
            let b = self.pt(r.tr());
            RtExcl::new(a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)).shape()
        } else {
            let p = poly_excl(&r.pts());
            self.poly(&p).shape()
        }
    }

    #[must_use]
    pub fn is_similar(&self) -> bool {
        eq(self.m[(2, 0)], 0.0)
            && eq(self.m[(2, 1)], 0.0)
            && eq(self.m[(2, 2)], 1.0)
            && eq(self.m[(0, 0)].abs(), self.m[(1, 1)].abs())
            && eq(self.m[(0, 1)], -self.m[(1, 0)])
    }

    #[must_use]
    pub fn length(&self, l: f64) -> Option<f64> {
        self.is_similar().then(|| l * pt(self.m[(0, 0)], self.m[(1, 0)]).mag())
    }

    #[must_use]
    pub fn cap<const B: Boundary>(&self, c: &CapsulePrimitive<B>) -> Option<CapsulePrimitive<B>> {
        Some(CapsulePrimitive::new(self.pt(c.st()), self.pt(c.en()), self.length(c.r())?))
    }

    #[must_use]
    pub fn circ<const B: Boundary>(&self, c: &CirclePrimitive<B>) -> Option<CirclePrimitive<B>> {
        Some(CirclePrimitive::new(self.pt(c.p()), self.length(c.r())?))
    }

    pub fn line(&self, l: &Line) -> Line {
        prim_line(self.pt(l.st()), self.pt(l.en()))
    }

    #[must_use]
    pub fn path<const B: Boundary>(&self, p: &PathPrimitive<B>) -> Option<PathPrimitive<B>> {
        let pts = p.pts().iter().map(|&v| self.pt(v)).collect::<Vec<_>>();
        Some(PathPrimitive::new(&pts, self.length(p.r())?))
    }

    pub fn poly<const B: Boundary>(&self, p: &PolyPrimitive<B>) -> PolyPrimitive<B> {
        let pts = p.pts().iter().map(|&v| self.pt(v)).collect::<Vec<_>>();
        PolyPrimitive::new(&pts)
    }

    pub fn seg(&self, s: &Segment) -> Segment {
        prim_seg(self.pt(s.st()), self.pt(s.en()))
    }

    pub fn tri<const B: Boundary>(&self, t: &TriPrimitive<B>) -> TriPrimitive<B> {
        let pts = t.pts();
        TriPrimitive::new([self.pt(pts[0]), self.pt(pts[1]), self.pt(pts[2])])
    }

    #[must_use]
    pub fn shape(&self, s: &Shape) -> Option<Shape> {
        match s {
            Shape::Capsule(s) => Some(self.cap(s)?.shape()),
            Shape::CapsuleExcl(s) => Some(self.cap(s)?.shape()),
            Shape::Circle(s) => Some(self.circ(s)?.shape()),
            Shape::CircleExcl(s) => Some(self.circ(s)?.shape()),
            Shape::Compound(_) => None,
            Shape::Line(s) => Some(self.line(s).shape()),
            Shape::Path(s) => Some(self.path(s)?.shape()),
            Shape::PathExcl(s) => Some(self.path(s)?.shape()),
            Shape::Point(s) => Some(self.pt(*s).shape()),
            Shape::Poly(s) => Some(self.poly(s).shape()),
            Shape::PolyExcl(s) => Some(self.poly(s).shape()),
            Shape::Rect(s) => Some(self.rt(s)),
            Shape::RectExcl(s) => Some(self.rt_excl(s)),
            Shape::Segment(s) => Some(self.seg(s).shape()),
            Shape::Tri(s) => Some(self.tri(s).shape()),
            Shape::TriExcl(s) => Some(self.tri(s).shape()),
        }
    }

    #[must_use]
    pub fn pts(&self, p: &[Pt]) -> Vec<Pt> {
        p.iter().map(|&v| self.pt(v)).collect()
    }
}

impl_op_ex!(*|a: &Tf, b: &Tf| -> Tf { Tf { m: a.m * b.m } });
impl_op_ex!(*= |a: &mut Tf, b: &Tf| { a.m *= b.m });

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;
    use crate::primitive::rt;

    #[test]
    fn inv_identity_returns_some() {
        let tf = Tf::identity();
        let inv = tf.inv();
        assert!(inv.is_some());
        let inv = inv.unwrap();
        assert_eq!(inv, Tf::identity());
    }

    #[test]
    fn inv_translation_returns_some() {
        let tf = Tf::translate(pt(3.0, 4.0));
        let inv = tf.inv();
        assert!(inv.is_some());
        let inv = inv.unwrap();
        // tf * inv should be identity
        let composed = tf * inv;
        assert_relative_eq!(composed.pt(pt(1.0, 1.0)).x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(composed.pt(pt(1.0, 1.0)).y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn inv_rotation_returns_some() {
        let tf = Tf::rotate(45.0);
        let inv = tf.inv();
        assert!(inv.is_some());
        let inv = inv.unwrap();
        // Applying tf then inv should return original point
        let p = pt(1.0, 0.0);
        let result = inv.pt(tf.pt(p));
        assert_relative_eq!(result.x, p.x, epsilon = 1e-10);
        assert_relative_eq!(result.y, p.y, epsilon = 1e-10);
    }

    #[test]
    fn inv_scale_zero_returns_none() {
        // Scale by zero creates a singular matrix
        let tf = Tf::scale(pt(0.0, 1.0));
        assert!(tf.inv().is_none());
    }

    #[test]
    fn inv_scale_both_zero_returns_none() {
        let tf = Tf::scale(pt(0.0, 0.0));
        assert!(tf.inv().is_none());
    }

    #[test]
    fn affine_zero_width_returns_none() {
        let from = rt(0.0, 0.0, 0.0, 1.0);
        let to = rt(1.0, 1.0, 2.0, 2.0);
        assert!(Tf::affine(&from, &to).is_none());
    }

    #[test]
    fn affine_zero_height_returns_none() {
        let from = rt(0.0, 0.0, 1.0, 0.0);
        let to = rt(1.0, 1.0, 2.0, 2.0);
        assert!(Tf::affine(&from, &to).is_none());
    }

    #[test]
    fn affine_both_zero_returns_none() {
        let from = rt(0.0, 0.0, 0.0, 0.0);
        let to = rt(1.0, 1.0, 2.0, 2.0);
        assert!(Tf::affine(&from, &to).is_none());
    }

    #[test]
    fn affine_valid_rectangles_returns_some() {
        let from = rt(0.0, 0.0, 1.0, 1.0);
        let to = rt(2.0, 2.0, 4.0, 4.0);
        let tf = Tf::affine(&from, &to);
        assert!(tf.is_some());
        let tf = tf.unwrap();
        // bl of from should map to bl of to
        assert_relative_eq!(tf.pt(from.bl()).x, to.bl().x, epsilon = 1e-10);
        assert_relative_eq!(tf.pt(from.bl()).y, to.bl().y, epsilon = 1e-10);
        // tr of from should map to tr of to
        assert_relative_eq!(tf.pt(from.tr()).x, to.tr().x, epsilon = 1e-10);
        assert_relative_eq!(tf.pt(from.tr()).y, to.tr().y, epsilon = 1e-10);
    }

    #[test]
    fn affine_to_zero_size_returns_some() {
        // Target can be zero-sized, just source can't
        let from = rt(0.0, 0.0, 1.0, 1.0);
        let to = rt(2.0, 2.0, 2.0, 2.0);
        let tf = Tf::affine(&from, &to);
        assert!(tf.is_some());
    }

    #[test]
    fn is_similar_basic() {
        assert!(Tf::identity().is_similar());
        assert!(Tf::scale(pt(2.0, 2.0)).is_similar());
        assert!(Tf::scale(pt(-2.0, -2.0)).is_similar());
        assert!(!Tf::scale(pt(2.0, 3.0)).is_similar());
        assert!(!Tf::scale(pt(-2.0, 3.0)).is_similar());
        // Reflection (opposite signs) with same magnitude IS a similarity
        assert!(Tf::scale(pt(-2.0, 2.0)).is_similar());
        assert!(Tf::scale(pt(2.0, -2.0)).is_similar());
        assert!(Tf::rotate(45.0).is_similar());
        assert!(Tf::rotate(90.0).is_similar());
        assert!(Tf::rotate(-30.0).is_similar());
        assert!(Tf::translate(pt(5.0, 10.0)).is_similar());
        let tf = Tf::translate(pt(1.0, 2.0)) * Tf::rotate(45.0) * Tf::scale(pt(2.0, 2.0));
        assert!(tf.is_similar());
    }

    #[test]
    fn length_with_reflection() {
        // Reflection about Y axis (flip X)
        let tf = Tf::scale(pt(-1.0, 1.0));
        assert!(tf.is_similar());
        assert_relative_eq!(tf.length(5.0).unwrap(), 5.0, epsilon = 1e-10);

        // Uniform negative scale (point reflection through origin)
        let tf = Tf::scale(pt(-2.0, -2.0));
        assert!(tf.is_similar());
        assert_relative_eq!(tf.length(5.0).unwrap(), 10.0, epsilon = 1e-10);
    }

    #[test]
    fn length_non_similar_returns_none() {
        let tf = Tf::scale(pt(2.0, 3.0));
        assert!(tf.length(5.0).is_none());
    }
}
