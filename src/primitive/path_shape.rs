use std::ops::Index;

use crate::geom::bounds::pt_cloud_bounds;
use crate::geom::contains::path_contains_rt;
use crate::geom::convex::remove_collinear;
use crate::geom::distance::{cap_path_dist, circ_path_dist, path_poly_dist, rt_path_dist};
use crate::geom::intersects::{
    cap_intersects_path, circ_intersects_path, path_intersects_path, path_intersects_poly,
    path_intersects_rt,
};
use crate::geom::math::eq;
use crate::primitive::capsule::CapsulePrimitive;
use crate::primitive::point::Pt;
use crate::primitive::shape::Shape;
use crate::primitive::{Boundary, Path, PathExcl, Rt, ShapeOps};

#[must_use]
#[derive(Clone)]
pub struct PathPrimitive<const B: Boundary = { Boundary::Include }> {
    pts: Vec<Pt>,
    r: f64,
    bounds: Option<Rt>,
}

impl<const B: Boundary> Default for PathPrimitive<B> {
    fn default() -> Self {
        Self::new(&[], 1.0)
    }
}

impl<const B: Boundary> std::fmt::Debug for PathPrimitive<B> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?} {:?}", self.pts, self.r)
    }
}

impl<const B: Boundary> PathPrimitive<B> {
    pub fn new(pts: &[Pt], r: f64) -> Self {
        assert!(r.is_finite() && r >= 0.0, "path radius must be finite and non-negative");
        for pt in pts {
            assert!(pt.x.is_finite() && pt.y.is_finite(), "path point coordinates must be finite");
        }
        let pts = remove_collinear(pts, false);
        let bounds = pt_cloud_bounds(&pts).map(|b| b.inset(-r, -r));
        Self { pts, r, bounds }
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.pts.len()
    }

    /// Returns true if this path represents the empty set.
    /// A path is empty if it has no points, OR if r == 0 and boundary is excluded.
    #[must_use]
    pub fn is_empty_set(&self) -> bool {
        if self.pts.is_empty() {
            return true;
        }
        match B {
            Boundary::Include => false,
            Boundary::Exclude => eq(self.r, 0.0),
        }
    }

    pub fn pts(&self) -> &[Pt] {
        &self.pts
    }

    pub fn caps(&self) -> impl '_ + Iterator<Item = CapsulePrimitive<B>> {
        let singleton =
            (self.pts.len() == 1).then(|| CapsulePrimitive::new(self.pts[0], self.pts[0], self.r));
        let pairs =
            self.pts.array_windows::<2>().map(move |v| CapsulePrimitive::new(v[0], v[1], self.r));
        singleton.into_iter().chain(pairs)
    }

    #[must_use]
    pub const fn r(&self) -> f64 {
        self.r
    }

    #[must_use]
    pub fn bounds(&self) -> Option<Rt> {
        self.bounds
    }

    fn intersects_shape_impl(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(s) => cap_intersects_path(s, self),
            Shape::CapsuleExcl(s) => cap_intersects_path(s, self),
            Shape::Circle(s) => circ_intersects_path(s, self),
            Shape::CircleExcl(s) => circ_intersects_path(s, self),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(s) => path_intersects_path(self, s),
            Shape::PathExcl(s) => path_intersects_path(self, s),
            Shape::Point(_) => todo!(),
            Shape::Poly(s) => path_intersects_poly(self, s),
            Shape::PolyExcl(s) => path_intersects_poly(self, s),
            Shape::Rect(s) => path_intersects_rt(self, s),
            Shape::RectExcl(s) => path_intersects_rt(self, s),
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
            Shape::Point(_) => todo!(),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(s) => path_contains_rt(self, s),
            Shape::RectExcl(s) => path_contains_rt(self, s),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }

    fn dist_to_shape_impl(&self, s: &Shape) -> Option<f64> {
        match s {
            Shape::Capsule(s) => cap_path_dist(s, self),
            Shape::CapsuleExcl(s) => cap_path_dist(s, self),
            Shape::Circle(s) => circ_path_dist(s, self),
            Shape::CircleExcl(s) => circ_path_dist(s, self),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(_) => todo!(),
            Shape::Poly(s) => path_poly_dist(self, s),
            Shape::PolyExcl(s) => path_poly_dist(self, s),
            Shape::Rect(s) => rt_path_dist(s, self),
            Shape::RectExcl(s) => rt_path_dist(s, self),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }
}

impl ShapeOps for Path {
    fn bounds(&self) -> Option<Rt> {
        self.bounds()
    }
    fn shape(self) -> Shape {
        Shape::Path(self)
    }
    fn is_empty_set(&self) -> bool {
        PathPrimitive::is_empty_set(self)
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

impl ShapeOps for PathExcl {
    fn bounds(&self) -> Option<Rt> {
        self.bounds()
    }
    fn shape(self) -> Shape {
        Shape::PathExcl(self)
    }
    fn is_empty_set(&self) -> bool {
        PathPrimitive::is_empty_set(self)
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

impl<const B: Boundary> Index<usize> for PathPrimitive<B> {
    type Output = Pt;

    fn index(&self, index: usize) -> &Self::Output {
        &self.pts[index]
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::geom::math::EP;
    use crate::primitive::{path, pt};

    #[test]
    fn path_bounds_includes_full_radius() {
        // A horizontal path from (0,0) to (10,0) with radius 2
        // The bounds should extend from (-2, -2) to (12, 2)
        let p = path(&[pt(0.0, 0.0), pt(10.0, 0.0)], 2.0);
        let bounds = p.bounds().unwrap();

        // Bounds should extend by full radius, not half radius
        assert_relative_eq!(bounds.l(), -2.0, epsilon = EP);
        assert_relative_eq!(bounds.r(), 12.0, epsilon = EP);
        assert_relative_eq!(bounds.b(), -2.0, epsilon = EP);
        assert_relative_eq!(bounds.t(), 2.0, epsilon = EP);
    }

    #[test]
    fn path_bounds_singleton_point() {
        // A single-point path at (5, 5) with radius 3
        // The bounds should be (2, 2) to (8, 8)
        let p = path(&[pt(5.0, 5.0)], 3.0);
        let bounds = p.bounds().unwrap();

        assert_relative_eq!(bounds.l(), 2.0, epsilon = EP);
        assert_relative_eq!(bounds.r(), 8.0, epsilon = EP);
        assert_relative_eq!(bounds.b(), 2.0, epsilon = EP);
        assert_relative_eq!(bounds.t(), 8.0, epsilon = EP);
    }
}
