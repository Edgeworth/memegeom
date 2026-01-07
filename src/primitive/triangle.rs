use std::ops::Index;

use approx::{AbsDiffEq, RelativeEq};
use derive_more::Display;

use crate::geom::bounds::pt_cloud_bounds;
use crate::geom::contains::{tri_contains_pt, tri_contains_rt};
use crate::geom::convex::ensure_ccw;
use crate::geom::intersects::{cap_intersects_tri, circ_intersects_tri, rt_intersects_tri};
use crate::geom::math::{EP, is_collinear};
use crate::primitive::point::Pt;
use crate::primitive::shape::Shape;
use crate::primitive::{Boundary, Rt, Segment, ShapeOps, Tri, TriExcl, seg};

// Is in CCW order.
#[must_use]
#[derive(Debug, Display, Copy, Clone, PartialEq, Default)]
#[display("Tri[{}, {}, {}]", self.pts[0], self.pts[1], self.pts[2])]
pub struct TriPrimitive<const B: Boundary> {
    pts: [Pt; 3],
}

impl<const B: Boundary> Eq for TriPrimitive<B> {}

impl<const B: Boundary> AbsDiffEq for TriPrimitive<B> {
    type Epsilon = f64;

    fn default_epsilon() -> f64 {
        EP
    }

    fn abs_diff_eq(&self, o: &Self, epsilon: f64) -> bool {
        Pt::abs_diff_eq(&self.pts[0], &o.pts[0], epsilon)
            && Pt::abs_diff_eq(&self.pts[1], &o.pts[1], epsilon)
            && Pt::abs_diff_eq(&self.pts[2], &o.pts[2], epsilon)
    }
}

impl<const B: Boundary> RelativeEq for TriPrimitive<B> {
    fn default_max_relative() -> f64 {
        EP
    }

    fn relative_eq(&self, o: &Self, epsilon: f64, max_relative: f64) -> bool {
        Pt::relative_eq(&self.pts[0], &o.pts[0], epsilon, max_relative)
            && Pt::relative_eq(&self.pts[1], &o.pts[1], epsilon, max_relative)
            && Pt::relative_eq(&self.pts[2], &o.pts[2], epsilon, max_relative)
    }
}

impl<const B: Boundary> TriPrimitive<B> {
    pub fn new(mut pts: [Pt; 3]) -> Self {
        for pt in &pts {
            assert!(
                pt.x.is_finite() && pt.y.is_finite(),
                "triangle point coordinates must be finite"
            );
        }
        ensure_ccw(&mut pts);
        Self { pts }
    }

    pub const fn pts(&self) -> &[Pt; 3] {
        &self.pts
    }

    pub fn segs(&self) -> [Segment; 3] {
        [
            seg(self.pts[0], self.pts[1]),
            seg(self.pts[1], self.pts[2]),
            seg(self.pts[2], self.pts[0]),
        ]
    }

    #[must_use]
    pub fn bounds(&self) -> Option<Rt> {
        pt_cloud_bounds(&self.pts)
    }

    /// Returns true if this triangle is degenerate (collinear points, zero area).
    #[must_use]
    pub fn is_degenerate(&self) -> bool {
        is_collinear(self.pts[0], self.pts[1], self.pts[2])
    }

    /// Returns true if this triangle represents the empty set.
    /// A triangle is empty only if it's degenerate (collinear) AND boundary is excluded.
    #[must_use]
    pub fn is_empty_set(&self) -> bool {
        match B {
            Boundary::Include => false, // Degenerate triangle still has its boundary
            Boundary::Exclude => self.is_degenerate(), // Degenerate excluded triangle has no interior
        }
    }

    #[must_use]
    pub fn rel_eq(&self, o: &Self) -> bool {
        RelativeEq::relative_eq(self, o, EP, EP)
    }

    fn intersects_shape_impl(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(s) => cap_intersects_tri(s, self),
            Shape::CapsuleExcl(s) => cap_intersects_tri(s, self),
            Shape::Circle(s) => circ_intersects_tri(s, self),
            Shape::CircleExcl(s) => circ_intersects_tri(s, self),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(s) => tri_contains_pt(self, s),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(s) => rt_intersects_tri(s, self),
            Shape::RectExcl(s) => rt_intersects_tri(s, self),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }

    fn contains_shape_impl(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(_) | Shape::CapsuleExcl(_) => todo!(),
            Shape::Circle(_) | Shape::CircleExcl(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(s) => tri_contains_pt(self, s),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(s) => tri_contains_rt(self, s),
            Shape::RectExcl(s) => tri_contains_rt(self, s),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }

    fn dist_to_shape_impl(&self, _s: &Shape) -> Option<f64> {
        // Triangle always has 3 points, never empty.
        // All distance implementations are todo!() for now.
        todo!()
    }
}

impl ShapeOps for Tri {
    fn bounds(&self) -> Option<Rt> {
        self.bounds()
    }
    fn shape(self) -> Shape {
        Shape::Tri(self)
    }
    fn is_empty_set(&self) -> bool {
        TriPrimitive::is_empty_set(self)
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

impl ShapeOps for TriExcl {
    fn bounds(&self) -> Option<Rt> {
        self.bounds()
    }
    fn shape(self) -> Shape {
        Shape::TriExcl(self)
    }
    fn is_empty_set(&self) -> bool {
        TriPrimitive::is_empty_set(self)
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

impl<const B: Boundary> Index<usize> for TriPrimitive<B> {
    type Output = Pt;

    fn index(&self, index: usize) -> &Self::Output {
        &self.pts[index]
    }
}
