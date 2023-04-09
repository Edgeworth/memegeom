use std::f64::consts::PI;

use auto_ops::impl_op_ex;
use nalgebra::{vector, Matrix3};

use crate::geom::math::eq;
use crate::primitive::capsule::Capsule;
use crate::primitive::circle::Circle;
use crate::primitive::line_shape::Line;
use crate::primitive::path_shape::Path;
use crate::primitive::point::Pt;
use crate::primitive::polygon::Poly;
use crate::primitive::rect::Rt;
use crate::primitive::segment::Segment;
use crate::primitive::shape::Shape;
use crate::primitive::triangle::Tri;
use crate::primitive::{cap, circ, line, path, poly, pt, seg, tri, ShapeOps};

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

    pub fn affine(from: &Rt, to: &Rt) -> Self {
        let xscale = to.w() / from.w();
        let yscale = to.h() / from.h();
        let scale = Self::scale(pt(xscale, yscale));
        let offset = to.bl() - scale.pt(from.bl());
        Self::translate(offset) * scale
    }

    pub fn inv(&self) -> Tf {
        Tf { m: self.m.try_inverse().unwrap() }
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
            let poly = poly(&r.pts());
            self.poly(&poly).shape()
        }
    }

    // TODO: Assumes similarity transformation.
    fn check_similarity(&self) {
        assert!(eq(self.m[(2, 0)], 0.0));
        assert!(eq(self.m[(2, 1)], 0.0));
        assert!(eq(self.m[(2, 2)], 1.0));
        assert!(eq(self.m[(0, 0)].abs(), self.m[(1, 1)].abs()));
        assert!(eq(self.m[(0, 1)], -self.m[(1, 0)]));
    }

    #[must_use]
    pub fn length(&self, l: f64) -> f64 {
        self.check_similarity();
        l * pt(self.m[(0, 0)], self.m[(1, 0)]).mag()
    }

    pub fn cap(&self, c: &Capsule) -> Capsule {
        cap(self.pt(c.st()), self.pt(c.en()), self.length(c.r()))
    }

    pub fn circ(&self, c: &Circle) -> Circle {
        circ(self.pt(c.p()), self.length(c.r()))
    }

    pub fn line(&self, l: &Line) -> Line {
        line(self.pt(l.st()), self.pt(l.en()))
    }

    pub fn path(&self, p: &Path) -> Path {
        let pts = p.pts().iter().map(|&v| self.pt(v)).collect::<Vec<_>>();
        path(&pts, self.length(p.r()))
    }

    pub fn poly(&self, p: &Poly) -> Poly {
        let pts = p.pts().iter().map(|&v| self.pt(v)).collect::<Vec<_>>();
        poly(&pts)
    }

    pub fn seg(&self, s: &Segment) -> Segment {
        seg(self.pt(s.st()), self.pt(s.en()))
    }

    pub fn tri(&self, t: &Tri) -> Tri {
        let pts = t.pts();
        tri(self.pt(pts[0]), self.pt(pts[1]), self.pt(pts[2]))
    }

    pub fn shape(&self, s: &Shape) -> Shape {
        match s {
            Shape::Capsule(s) => self.cap(s).shape(),
            Shape::Circle(s) => self.circ(s).shape(),
            Shape::Compound(_) => todo!(),
            Shape::Line(s) => self.line(s).shape(),
            Shape::Path(s) => self.path(s).shape(),
            Shape::Point(s) => self.pt(*s).shape(),
            Shape::Polygon(s) => self.poly(s).shape(),
            Shape::Rect(s) => self.rt(s),
            Shape::Segment(s) => self.seg(s).shape(),
            Shape::Tri(s) => self.tri(s).shape(),
        }
    }

    #[must_use]
    pub fn pts(&self, p: &[Pt]) -> Vec<Pt> {
        p.iter().map(|&v| self.pt(v)).collect()
    }
}

impl_op_ex!(*|a: &Tf, b: &Tf| -> Tf { Tf { m: a.m * b.m } });
impl_op_ex!(*= |a: &mut Tf, b: &Tf| { a.m *= b.m });
