use approx::{AbsDiffEq, RelativeEq};

use crate::geom::contains::{circ_contains_pt, circ_contains_rt};
use crate::geom::distance::{
    cap_circ_dist, circ_circ_dist, circ_path_dist, circ_poly_dist, circ_pt_dist, circ_rt_dist,
};
use crate::geom::intersects::{
    cap_intersects_circ, circ_intersects_circ, circ_intersects_path, circ_intersects_poly,
    circ_intersects_rt, circ_intersects_tri,
};
use crate::geom::math::{EP, eq};
use crate::primitive::point::Pt;
use crate::primitive::shape::Shape;
use crate::primitive::{Boundary, Circle, CircleExcl, Rt, ShapeOps, rt};

#[must_use]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CirclePrimitive<const B: Boundary> {
    p: Pt,
    r: f64,
}

impl<const B: Boundary> Default for CirclePrimitive<B> {
    fn default() -> Self {
        Self::new(Pt::default(), 1.0)
    }
}

impl<const B: Boundary> Eq for CirclePrimitive<B> {}

impl<const B: Boundary> AbsDiffEq for CirclePrimitive<B> {
    type Epsilon = f64;

    fn default_epsilon() -> f64 {
        EP
    }

    fn abs_diff_eq(&self, o: &Self, epsilon: f64) -> bool {
        Pt::abs_diff_eq(&self.p, &o.p, epsilon) && f64::abs_diff_eq(&self.r, &o.r, epsilon)
    }
}

impl<const B: Boundary> RelativeEq for CirclePrimitive<B> {
    fn default_max_relative() -> f64 {
        EP
    }

    fn relative_eq(&self, o: &Self, epsilon: f64, max_relative: f64) -> bool {
        Pt::relative_eq(&self.p, &o.p, epsilon, max_relative)
            && f64::relative_eq(&self.r, &o.r, epsilon, max_relative)
    }
}

impl<const B: Boundary> CirclePrimitive<B> {
    pub const fn new(p: Pt, r: f64) -> Self {
        assert!(p.x.is_finite() && p.y.is_finite(), "circle centre coordinates must be finite");
        assert!(r.is_finite() && r >= 0.0, "circle radius must be finite and non-negative");
        Self { p, r }
    }

    #[must_use]
    pub const fn r(&self) -> f64 {
        self.r
    }

    pub const fn p(&self) -> Pt {
        self.p
    }

    #[must_use]
    pub fn rel_eq(&self, o: &Self) -> bool {
        RelativeEq::relative_eq(self, o, EP, EP)
    }

    #[must_use]
    pub fn bounds(&self) -> Option<Rt> {
        Some(rt(self.p.x - self.r, self.p.y - self.r, self.p.x + self.r, self.p.y + self.r))
    }

    /// Returns true if this circle represents the empty set.
    /// A circle is empty only if r == 0 and boundary is excluded (just a point with no interior).
    #[must_use]
    pub fn is_empty_set(&self) -> bool {
        match B {
            Boundary::Include => false, // Closed circle with r==0 is still a point (non-empty)
            Boundary::Exclude => eq(self.r, 0.0), // Open circle with r==0 has no interior
        }
    }

    fn intersects_shape_impl(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(s) => cap_intersects_circ(s, self),
            Shape::CapsuleExcl(s) => cap_intersects_circ(s, self),
            Shape::Circle(s) => circ_intersects_circ(self, s),
            Shape::CircleExcl(s) => circ_intersects_circ(self, s),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(s) => circ_intersects_path(self, s),
            Shape::PathExcl(s) => circ_intersects_path(self, s),
            Shape::Point(s) => circ_contains_pt(self, s),
            Shape::Poly(s) => circ_intersects_poly(self, s),
            Shape::PolyExcl(s) => circ_intersects_poly(self, s),
            Shape::Rect(s) => circ_intersects_rt(self, s),
            Shape::RectExcl(s) => circ_intersects_rt(self, s),
            Shape::Segment(_) => todo!(),
            Shape::Tri(s) => circ_intersects_tri(self, s),
            Shape::TriExcl(s) => circ_intersects_tri(self, s),
        }
    }

    fn contains_shape_impl(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(_) | Shape::CapsuleExcl(_) => todo!(),
            Shape::Circle(_) | Shape::CircleExcl(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(s) => circ_contains_pt(self, s),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(s) => circ_contains_rt(self, s),
            Shape::RectExcl(s) => circ_contains_rt(self, s),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }

    fn dist_to_shape_impl(&self, s: &Shape) -> Option<f64> {
        match s {
            Shape::Capsule(s) => cap_circ_dist(s, self),
            Shape::CapsuleExcl(s) => cap_circ_dist(s, self),
            Shape::Circle(s) => circ_circ_dist(self, s),
            Shape::CircleExcl(s) => circ_circ_dist(self, s),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(s) => circ_path_dist(self, s),
            Shape::PathExcl(s) => circ_path_dist(self, s),
            Shape::Point(s) => circ_pt_dist(self, s),
            Shape::Poly(s) => circ_poly_dist(self, s),
            Shape::PolyExcl(s) => circ_poly_dist(self, s),
            Shape::Rect(s) => circ_rt_dist(self, s),
            Shape::RectExcl(s) => circ_rt_dist(self, s),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }
}

impl ShapeOps for Circle {
    fn bounds(&self) -> Option<Rt> {
        self.bounds()
    }
    fn shape(self) -> Shape {
        Shape::Circle(self)
    }
    fn is_empty_set(&self) -> bool {
        CirclePrimitive::is_empty_set(self)
    }
    fn intersects_shape(&self, s: &Shape) -> bool {
        self.intersects_shape_impl(s)
    }
    fn contains_shape(&self, s: &Shape) -> bool {
        self.contains_shape_impl(s)
    }
    fn dist_to_shape(&self, s: &Shape) -> Option<f64> {
        self.dist_to_shape_impl(s)
    }
}

impl ShapeOps for CircleExcl {
    fn bounds(&self) -> Option<Rt> {
        self.bounds()
    }
    fn shape(self) -> Shape {
        Shape::CircleExcl(self)
    }
    fn is_empty_set(&self) -> bool {
        CirclePrimitive::is_empty_set(self)
    }
    fn intersects_shape(&self, s: &Shape) -> bool {
        self.intersects_shape_impl(s)
    }
    fn contains_shape(&self, s: &Shape) -> bool {
        self.contains_shape_impl(s)
    }
    fn dist_to_shape(&self, s: &Shape) -> Option<f64> {
        self.dist_to_shape_impl(s)
    }
}

#[cfg(test)]
mod tests {
    use crate::primitive::{ShapeOps, cap, circ, pt};

    #[test]
    fn circle_intersects_capsule_overlapping() {
        // Circle and capsule that clearly overlap
        let c = circ(pt(0.0, 0.0), 1.0);
        let cap = cap(pt(0.5, 0.0), pt(2.0, 0.0), 0.5);
        assert!(c.intersects_shape(&cap.shape()));
    }

    #[test]
    fn circle_intersects_capsule_touching() {
        // Circle of radius 1 at origin, capsule starting at x=2 with radius 1
        // They should be touching at x=1
        let c = circ(pt(0.0, 0.0), 1.0);
        let cap = cap(pt(2.0, 0.0), pt(4.0, 0.0), 1.0);
        assert!(c.intersects_shape(&cap.shape()));
    }

    #[test]
    fn circle_intersects_capsule_not_touching() {
        // Circle and capsule that don't touch
        let c = circ(pt(0.0, 0.0), 1.0);
        let cap = cap(pt(5.0, 0.0), pt(7.0, 0.0), 1.0);
        assert!(!c.intersects_shape(&cap.shape()));
    }
}
