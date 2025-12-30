use std::cmp::Ordering;
use std::ops::Index;

use earcutr::earcut;

use crate::geom::bounds::pt_cloud_bounds;
use crate::geom::contains::{
    poly_contains_cap, poly_contains_circ, poly_contains_path, poly_contains_pt, poly_contains_rt,
    poly_contains_seg,
};
use crate::geom::convex::{ensure_ccw, is_convex_ccw, remove_collinear};
use crate::geom::distance::{
    cap_poly_dist, circ_poly_dist, path_poly_dist, poly_pt_dist, poly_rt_dist,
};
use crate::geom::intersects::{
    cap_intersects_poly, circ_intersects_poly, path_intersects_poly, poly_intersects_rt,
};
use crate::primitive::point::Pt;
use crate::primitive::shape::Shape;
use crate::primitive::triangle::TriPrimitive;
use crate::primitive::{Boundary, Poly, PolyExcl, Rt, ShapeOps};

// Represents a simple non-convex polygon.
// Stored in CCW order.
// TODO: make polygons use quadtree?
#[must_use]
#[derive(Debug, Clone, Default)]
pub struct PolyPrimitive<const B: Boundary> {
    pts: Vec<Pt>,
    tri: Vec<TriPrimitive<B>>,
    tri_idx: Vec<u32>,
    is_convex: bool,
}

impl<const B: Boundary> PolyPrimitive<B> {
    pub fn new(pts: &[Pt]) -> Self {
        for pt in pts {
            assert!(
                pt.x.is_finite() && pt.y.is_finite(),
                "polygon point coordinates must be finite"
            );
        }
        let mut pts = remove_collinear(pts, true);
        ensure_ccw(&mut pts);
        let verts: Vec<f64> = pts.iter().flat_map(|v| [v.x, v.y]).collect();
        let tri_idx: Vec<_> = earcut(&verts, &[], 2).unwrap().iter().map(|&v| v as u32).collect();
        let tri = tri_idx
            .iter()
            .array_chunks::<3>()
            .map(|v| {
                TriPrimitive::new([pts[*v[0] as usize], pts[*v[1] as usize], pts[*v[2] as usize]])
            })
            .collect();
        let is_convex = is_convex_ccw(&pts);
        Self { pts, tri, tri_idx, is_convex }
    }

    pub fn pts(&self) -> &[Pt] {
        &self.pts
    }

    pub fn edges(&self) -> EdgeIterator<'_> {
        edges(&self.pts)
    }

    pub fn tri(&self) -> &[TriPrimitive<B>] {
        &self.tri
    }

    #[must_use]
    pub fn tri_idx(&self) -> &[u32] {
        &self.tri_idx
    }

    #[must_use]
    pub const fn is_convex(&self) -> bool {
        self.is_convex
    }

    #[must_use]
    pub fn bounds(&self) -> Option<Rt> {
        pt_cloud_bounds(&self.pts)
    }

    /// Returns true if this polygon represents the empty set.
    /// For Include boundary: empty only if no points.
    /// For Exclude boundary: empty if no triangulation (< 3 points or all collinear).
    #[must_use]
    pub fn is_empty_set(&self) -> bool {
        match B {
            Boundary::Include => self.pts.is_empty(),
            Boundary::Exclude => self.tri.is_empty(),
        }
    }

    fn intersects_shape_impl(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(s) => cap_intersects_poly(s, self),
            Shape::CapsuleExcl(s) => cap_intersects_poly(s, self),
            Shape::Circle(s) => circ_intersects_poly(s, self),
            Shape::CircleExcl(s) => circ_intersects_poly(s, self),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(s) => path_intersects_poly(s, self),
            Shape::PathExcl(s) => path_intersects_poly(s, self),
            Shape::Point(s) => poly_contains_pt(self, s),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(s) => poly_intersects_rt(self, s),
            Shape::RectExcl(s) => poly_intersects_rt(self, s),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }

    fn contains_shape_impl(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(s) => poly_contains_cap(self, s),
            Shape::CapsuleExcl(s) => poly_contains_cap(self, s),
            Shape::Circle(s) => poly_contains_circ(self, s),
            Shape::CircleExcl(s) => poly_contains_circ(self, s),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(s) => poly_contains_path(self, s),
            Shape::PathExcl(s) => poly_contains_path(self, s),
            Shape::Point(s) => poly_contains_pt(self, s),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(s) => poly_contains_rt(self, s),
            Shape::RectExcl(s) => poly_contains_rt(self, s),
            Shape::Segment(s) => poly_contains_seg(self, s),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }

    fn dist_to_shape_impl(&self, s: &Shape) -> Option<f64> {
        match s {
            Shape::Capsule(s) => cap_poly_dist(s, self),
            Shape::CapsuleExcl(s) => cap_poly_dist(s, self),
            Shape::Circle(s) => circ_poly_dist(s, self),
            Shape::CircleExcl(s) => circ_poly_dist(s, self),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(s) => path_poly_dist(s, self),
            Shape::PathExcl(s) => path_poly_dist(s, self),
            Shape::Point(s) => poly_pt_dist(self, s),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(s) => poly_rt_dist(self, s),
            Shape::RectExcl(s) => poly_rt_dist(self, s),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }
}

impl ShapeOps for Poly {
    fn bounds(&self) -> Option<Rt> {
        self.bounds()
    }
    fn shape(self) -> Shape {
        Shape::Poly(self)
    }
    fn is_empty_set(&self) -> bool {
        PolyPrimitive::is_empty_set(self)
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

impl ShapeOps for PolyExcl {
    fn bounds(&self) -> Option<Rt> {
        self.bounds()
    }
    fn shape(self) -> Shape {
        Shape::PolyExcl(self)
    }
    fn is_empty_set(&self) -> bool {
        PolyPrimitive::is_empty_set(self)
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

impl<const B: Boundary> Index<usize> for PolyPrimitive<B> {
    type Output = Pt;

    fn index(&self, index: usize) -> &Self::Output {
        &self.pts[index]
    }
}

#[must_use]
pub struct EdgeIterator<'a> {
    pts: &'a [Pt],
    idx: usize,
}

impl<'a> EdgeIterator<'a> {
    pub fn new(pts: &'a [Pt]) -> Self {
        Self { pts, idx: 0 }
    }
}

impl<'a> Iterator for EdgeIterator<'a> {
    type Item = [&'a Pt; 2];

    fn next(&mut self) -> Option<Self::Item> {
        let edge = match self.pts.len().cmp(&(self.idx + 1)) {
            Ordering::Less => None,
            Ordering::Equal => Some([&self.pts[self.idx], &self.pts[0]]),
            Ordering::Greater => Some([&self.pts[self.idx], &self.pts[self.idx + 1]]),
        };
        self.idx += 1;
        edge
    }
}

pub fn edges(pts: &[Pt]) -> EdgeIterator<'_> {
    EdgeIterator::new(pts)
}

#[cfg(test)]
mod tests {
    use crate::primitive::{Poly, pt};

    #[test]
    fn poly_new_degenerate_inputs_have_empty_triangulation() {
        let p0 = Poly::new(&[]);
        assert_eq!(p0.pts().len(), 0);
        assert!(p0.tri().is_empty());
        assert!(p0.tri_idx().is_empty());

        let p1 = Poly::new(&[pt(0.0, 0.0)]);
        assert_eq!(p1.pts().len(), 1);
        assert!(p1.tri().is_empty());
        assert!(p1.tri_idx().is_empty());

        let p2 = Poly::new(&[pt(0.0, 0.0), pt(1.0, 0.0)]);
        assert_eq!(p2.pts().len(), 2);
        assert!(p2.tri().is_empty());
        assert!(p2.tri_idx().is_empty());
    }
}
