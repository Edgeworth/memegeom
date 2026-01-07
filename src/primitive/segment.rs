use approx::{AbsDiffEq, RelativeEq};
use derive_more::Display;

use crate::geom::distance::{cap_seg_dist, pt_seg_dist, rt_seg_dist, seg_seg_dist};
use crate::geom::intersects::{rt_intersects_seg, seg_intersects_seg};
use crate::geom::math::{EP, is_collinear};
use crate::primitive::point::Pt;
use crate::primitive::shape::Shape;
use crate::primitive::{Line, Rt, ShapeOps, line, rt};

#[must_use]
#[derive(Debug, Display, Copy, Clone, PartialEq, Eq, Default)]
#[display("Seg[{st}, {en}]")]
pub struct SegmentPrimitive {
    st: Pt,
    en: Pt,
}

impl SegmentPrimitive {
    pub const fn new(st: Pt, en: Pt) -> Self {
        assert!(
            st.x.is_finite() && st.y.is_finite() && en.x.is_finite() && en.y.is_finite(),
            "segment endpoint coordinates must be finite"
        );
        Self { st, en }
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

    pub const fn line(&self) -> Line {
        line(self.st, self.en)
    }

    #[must_use]
    pub fn contains(&self, p: Pt) -> bool {
        rt(
            self.st.x.min(self.en.x),
            self.st.y.min(self.en.y),
            self.st.x.max(self.en.x),
            self.st.y.max(self.en.y),
        )
        .contains(p)
            && is_collinear(self.st, self.en, p)
    }

    #[must_use]
    pub fn rel_eq(&self, o: &Self) -> bool {
        RelativeEq::relative_eq(self, o, EP, EP)
    }
}

impl AbsDiffEq for SegmentPrimitive {
    type Epsilon = f64;

    fn default_epsilon() -> f64 {
        EP
    }

    fn abs_diff_eq(&self, o: &Self, epsilon: f64) -> bool {
        Pt::abs_diff_eq(&self.st, &o.st, epsilon) && Pt::abs_diff_eq(&self.en, &o.en, epsilon)
    }
}

impl RelativeEq for SegmentPrimitive {
    fn default_max_relative() -> f64 {
        EP
    }

    fn relative_eq(&self, o: &Self, epsilon: f64, max_relative: f64) -> bool {
        Pt::relative_eq(&self.st, &o.st, epsilon, max_relative)
            && Pt::relative_eq(&self.en, &o.en, epsilon, max_relative)
    }
}

impl ShapeOps for SegmentPrimitive {
    fn bounds(&self) -> Option<Rt> {
        Some(Rt::enclosing(self.st, self.en))
    }

    fn shape(self) -> Shape {
        Shape::Segment(self)
    }

    fn is_empty_set(&self) -> bool {
        false // Segments always contain at least their endpoints
    }

    fn intersects_shape(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(_) | Shape::CapsuleExcl(_) => todo!(),
            Shape::Circle(_) | Shape::CircleExcl(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(_) => todo!(),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(s) => rt_intersects_seg(s, self),
            Shape::RectExcl(s) => rt_intersects_seg(s, self),
            Shape::Segment(s) => seg_intersects_seg(self, s),
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
            Shape::Capsule(s) => cap_seg_dist(s, self),
            Shape::CapsuleExcl(s) => cap_seg_dist(s, self),
            Shape::Circle(_) | Shape::CircleExcl(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(s) => Some(pt_seg_dist(s, self)),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(s) => rt_seg_dist(s, self),
            Shape::RectExcl(s) => rt_seg_dist(s, self),
            Shape::Segment(s) => Some(seg_seg_dist(self, s)),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }
}
