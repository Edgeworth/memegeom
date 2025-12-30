use approx::{AbsDiffEq, RelativeEq};
use derive_more::Display;

use crate::geom::contains::{cap_contains_pt, cap_contains_rt};
use crate::geom::distance::{
    cap_cap_dist, cap_circ_dist, cap_path_dist, cap_poly_dist, cap_rt_dist, cap_seg_dist,
};
use crate::geom::intersects::{
    cap_intersects_cap, cap_intersects_circ, cap_intersects_path, cap_intersects_poly,
    cap_intersects_rt, cap_intersects_tri,
};
use crate::geom::math::eq;
use crate::primitive::circle::CirclePrimitive;
use crate::primitive::point::Pt;
use crate::primitive::shape::Shape;
use crate::primitive::{Boundary, Capsule, CapsuleExcl, Rt, Segment, ShapeOps, seg};

#[must_use]
#[derive(Debug, Display, Copy, Clone, PartialEq)]
#[display("Cap[{st}, {en}; {r}]")]
pub struct CapsulePrimitive<const B: Boundary> {
    st: Pt,
    en: Pt,
    r: f64,
}

impl<const B: Boundary> Default for CapsulePrimitive<B> {
    fn default() -> Self {
        Self::new(Pt::default(), Pt::default(), 1.0)
    }
}

impl<const B: Boundary> Eq for CapsulePrimitive<B> {}

impl<const B: Boundary> AbsDiffEq for CapsulePrimitive<B> {
    type Epsilon = f64;

    fn default_epsilon() -> f64 {
        f64::default_epsilon()
    }

    fn abs_diff_eq(&self, o: &Self, epsilon: f64) -> bool {
        Pt::abs_diff_eq(&self.st, &o.st, epsilon)
            && Pt::abs_diff_eq(&self.en, &o.en, epsilon)
            && f64::abs_diff_eq(&self.r, &o.r, epsilon)
    }
}

impl<const B: Boundary> RelativeEq for CapsulePrimitive<B> {
    fn default_max_relative() -> f64 {
        f64::default_max_relative()
    }

    fn relative_eq(&self, o: &Self, epsilon: f64, max_relative: f64) -> bool {
        Pt::relative_eq(&self.st, &o.st, epsilon, max_relative)
            && Pt::relative_eq(&self.en, &o.en, epsilon, max_relative)
            && f64::relative_eq(&self.r, &o.r, epsilon, max_relative)
    }
}

impl<const B: Boundary> CapsulePrimitive<B> {
    pub const fn new(st: Pt, en: Pt, r: f64) -> Self {
        assert!(
            st.x.is_finite() && st.y.is_finite() && en.x.is_finite() && en.y.is_finite(),
            "capsule endpoint coordinates must be finite"
        );
        assert!(r.is_finite() && r >= 0.0, "capsule radius must be finite and non-negative");
        Self { st, en, r }
    }

    #[must_use]
    pub const fn r(&self) -> f64 {
        self.r
    }

    pub const fn st(&self) -> Pt {
        self.st
    }

    pub const fn en(&self) -> Pt {
        self.en
    }

    pub fn dir(&self) -> Pt {
        self.en - self.st
    }

    pub fn st_cap(&self) -> CirclePrimitive<B> {
        CirclePrimitive::new(self.st(), self.r())
    }

    pub fn en_cap(&self) -> CirclePrimitive<B> {
        CirclePrimitive::new(self.en(), self.r())
    }

    // Left wall of the capsule. Returns None for degenerate capsules (st == en).
    #[must_use]
    pub fn left_seg(&self) -> Option<Segment> {
        let perp = -self.dir().perp()? * self.r();
        Some(seg(self.st + perp, self.en + perp))
    }

    // Right wall of the capsule. Returns None for degenerate capsules (st == en).
    #[must_use]
    pub fn right_seg(&self) -> Option<Segment> {
        let perp = self.dir().perp()? * self.r();
        Some(seg(self.st + perp, self.en + perp))
    }

    pub fn seg(&self) -> Segment {
        seg(self.st, self.en)
    }

    #[must_use]
    pub fn bounds(&self) -> Option<Rt> {
        Some(seg(self.st(), self.en()).bounds()?.inset(-self.r(), -self.r()))
    }

    /// Returns true if this capsule represents the empty set.
    /// A capsule is empty only if r == 0 and boundary is excluded (just a line with no interior).
    #[must_use]
    pub fn is_empty_set(&self) -> bool {
        match B {
            Boundary::Include => false, // Closed capsule with r==0 is still a segment (non-empty)
            Boundary::Exclude => eq(self.r, 0.0), // Open capsule with r==0 has no interior
        }
    }

    fn intersects_shape_impl(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(s) => cap_intersects_cap(self, s),
            Shape::CapsuleExcl(s) => cap_intersects_cap(self, s),
            Shape::Circle(s) => cap_intersects_circ(self, s),
            Shape::CircleExcl(s) => cap_intersects_circ(self, s),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(s) => cap_intersects_path(self, s),
            Shape::PathExcl(s) => cap_intersects_path(self, s),
            Shape::Point(s) => cap_contains_pt(self, s),
            Shape::Poly(s) => cap_intersects_poly(self, s),
            Shape::PolyExcl(s) => cap_intersects_poly(self, s),
            Shape::Rect(s) => cap_intersects_rt(self, s),
            Shape::RectExcl(s) => cap_intersects_rt(self, s),
            Shape::Segment(_) => todo!(),
            Shape::Tri(s) => cap_intersects_tri(self, s),
            Shape::TriExcl(s) => cap_intersects_tri(self, s),
        }
    }

    fn contains_shape_impl(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(_) | Shape::CapsuleExcl(_) => todo!(),
            Shape::Circle(_) | Shape::CircleExcl(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(s) => cap_contains_pt(self, s),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(s) => cap_contains_rt(self, s),
            Shape::RectExcl(s) => cap_contains_rt(self, s),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }

    fn dist_to_shape_impl(&self, s: &Shape) -> Option<f64> {
        match s {
            Shape::Capsule(s) => cap_cap_dist(self, s),
            Shape::CapsuleExcl(s) => cap_cap_dist(self, s),
            Shape::Circle(s) => cap_circ_dist(self, s),
            Shape::CircleExcl(s) => cap_circ_dist(self, s),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(s) => cap_path_dist(self, s),
            Shape::PathExcl(s) => cap_path_dist(self, s),
            Shape::Point(_) => todo!(),
            Shape::Poly(s) => cap_poly_dist(self, s),
            Shape::PolyExcl(s) => cap_poly_dist(self, s),
            Shape::Rect(s) => cap_rt_dist(self, s),
            Shape::RectExcl(s) => cap_rt_dist(self, s),
            Shape::Segment(s) => cap_seg_dist(self, s),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }
}

impl ShapeOps for Capsule {
    fn bounds(&self) -> Option<Rt> {
        self.bounds()
    }
    fn shape(self) -> Shape {
        Shape::Capsule(self)
    }
    fn is_empty_set(&self) -> bool {
        CapsulePrimitive::is_empty_set(self)
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

impl ShapeOps for CapsuleExcl {
    fn bounds(&self) -> Option<Rt> {
        self.bounds()
    }
    fn shape(self) -> Shape {
        Shape::CapsuleExcl(self)
    }
    fn is_empty_set(&self) -> bool {
        CapsulePrimitive::is_empty_set(self)
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
