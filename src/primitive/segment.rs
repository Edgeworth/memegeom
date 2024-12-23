use derive_more::Display;

use crate::geom::distance::{cap_seg_dist, pt_seg_dist, rt_seg_dist, seg_seg_dist};
use crate::geom::intersects::{line_intersects_seg, rt_intersects_seg, seg_intersects_seg};
use crate::geom::math::is_collinear;
use crate::primitive::line_shape::Line;
use crate::primitive::point::Pt;
use crate::primitive::rect::Rt;
use crate::primitive::shape::Shape;
use crate::primitive::{ShapeOps, line};

#[must_use]
#[derive(Debug, Display, Copy, Clone)]
#[display("Seg[{st}, {en}]")]
pub struct Segment {
    st: Pt,
    en: Pt,
}

impl Segment {
    pub const fn new(st: Pt, en: Pt) -> Self {
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
        Rt::enclosing(self.st, self.en).contains(p) && is_collinear(self.st, self.en, p)
    }
}

impl ShapeOps for Segment {
    fn bounds(&self) -> Rt {
        Rt::enclosing(self.st, self.en)
    }

    fn shape(self) -> Shape {
        Shape::Segment(self)
    }

    fn intersects_shape(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(_) => todo!(),
            Shape::Circle(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(s) => line_intersects_seg(s, self),
            Shape::Path(_) => todo!(),
            Shape::Point(_) => todo!(),
            Shape::Polygon(_) => todo!(),
            Shape::Rect(s) => rt_intersects_seg(s, self),
            Shape::Segment(s) => seg_intersects_seg(self, s),
            Shape::Tri(_) => todo!(),
        }
    }

    fn contains_shape(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(_) => todo!(),
            Shape::Circle(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) => todo!(),
            Shape::Point(_) => todo!(),
            Shape::Polygon(_) => todo!(),
            Shape::Rect(_) => todo!(),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) => todo!(),
        }
    }

    fn dist_to_shape(&self, s: &Shape) -> f64 {
        match s {
            Shape::Capsule(s) => cap_seg_dist(s, self),
            Shape::Circle(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) => todo!(),
            Shape::Point(s) => pt_seg_dist(s, self),
            Shape::Polygon(_) => todo!(),
            Shape::Rect(s) => rt_seg_dist(s, self),
            Shape::Segment(s) => seg_seg_dist(self, s),
            Shape::Tri(_) => todo!(),
        }
    }
}
