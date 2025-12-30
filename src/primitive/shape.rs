use crate::geom::math::eq;
use crate::primitive::compound::Compound;
use crate::primitive::point::Pt;
use crate::primitive::{
    Capsule, CapsuleExcl, Circle, CircleExcl, Line, Path, PathExcl, Poly, PolyExcl, Rt, RtExcl,
    Segment, ShapeOps, Tri, TriExcl,
};
use crate::tf::Tf;

#[must_use]
#[derive(Debug, Clone)]
pub enum Shape {
    Capsule(Capsule),
    CapsuleExcl(CapsuleExcl),
    Circle(Circle),
    CircleExcl(CircleExcl),
    Compound(Box<Compound>),
    Line(Line),
    Path(Path),
    PathExcl(PathExcl),
    Point(Pt),
    Poly(Poly),
    PolyExcl(PolyExcl),
    Rect(Rt),
    RectExcl(RtExcl),
    Segment(Segment),
    Tri(Tri),
    TriExcl(TriExcl),
}

impl Default for Shape {
    fn default() -> Self {
        Self::Point(Pt::default())
    }
}

impl Shape {
    pub fn filled(self) -> Shape {
        match self {
            Shape::Path(p) => {
                assert!(eq(p.r(), 0.0), "path width not supported for polygons");
                Poly::new(p.pts()).shape()
            }
            Shape::PathExcl(p) => {
                assert!(eq(p.r(), 0.0), "path width not supported for polygons");
                PolyExcl::new(p.pts()).shape()
            }
            s => s,
        }
    }

    #[must_use]
    pub fn apply(&mut self, tf: &Tf) -> bool {
        if let Some(shape) = tf.shape(self) {
            *self = shape;
            true
        } else {
            false
        }
    }
}

impl ShapeOps for Shape {
    fn bounds(&self) -> Option<Rt> {
        match self {
            Shape::Capsule(s) => s.bounds(),
            Shape::CapsuleExcl(s) => s.bounds(),
            Shape::Circle(s) => s.bounds(),
            Shape::CircleExcl(s) => s.bounds(),
            Shape::Compound(s) => s.bounds(),
            Shape::Line(s) => s.bounds(),
            Shape::Path(s) => s.bounds(),
            Shape::PathExcl(s) => s.bounds(),
            Shape::Point(s) => s.bounds(),
            Shape::Poly(s) => s.bounds(),
            Shape::PolyExcl(s) => s.bounds(),
            Shape::Rect(s) => s.bounds(),
            Shape::RectExcl(s) => s.bounds(),
            Shape::Segment(s) => s.bounds(),
            Shape::Tri(s) => s.bounds(),
            Shape::TriExcl(s) => s.bounds(),
        }
    }

    fn shape(self) -> Shape {
        self
    }

    fn is_empty_set(&self) -> bool {
        match self {
            Shape::Capsule(s) => s.is_empty_set(),
            Shape::CapsuleExcl(s) => s.is_empty_set(),
            Shape::Circle(s) => s.is_empty_set(),
            Shape::CircleExcl(s) => s.is_empty_set(),
            Shape::Compound(s) => s.is_empty_set(),
            Shape::Line(s) => s.is_empty_set(),
            Shape::Path(s) => s.is_empty_set(),
            Shape::PathExcl(s) => s.is_empty_set(),
            Shape::Point(s) => s.is_empty_set(),
            Shape::Poly(s) => s.is_empty_set(),
            Shape::PolyExcl(s) => s.is_empty_set(),
            Shape::Rect(s) => s.is_empty_set(),
            Shape::RectExcl(s) => s.is_empty_set(),
            Shape::Segment(s) => s.is_empty_set(),
            Shape::Tri(s) => s.is_empty_set(),
            Shape::TriExcl(s) => s.is_empty_set(),
        }
    }

    fn intersects_shape(&self, s: &Shape) -> bool {
        match self {
            Shape::Capsule(us) => us.intersects_shape(s),
            Shape::CapsuleExcl(us) => us.intersects_shape(s),
            Shape::Circle(us) => us.intersects_shape(s),
            Shape::CircleExcl(us) => us.intersects_shape(s),
            Shape::Compound(us) => us.intersects_shape(s),
            Shape::Line(us) => us.intersects_shape(s),
            Shape::Path(us) => us.intersects_shape(s),
            Shape::PathExcl(us) => us.intersects_shape(s),
            Shape::Point(us) => us.intersects_shape(s),
            Shape::Poly(us) => us.intersects_shape(s),
            Shape::PolyExcl(us) => us.intersects_shape(s),
            Shape::Rect(us) => us.intersects_shape(s),
            Shape::RectExcl(us) => us.intersects_shape(s),
            Shape::Segment(us) => us.intersects_shape(s),
            Shape::Tri(us) => us.intersects_shape(s),
            Shape::TriExcl(us) => us.intersects_shape(s),
        }
    }

    fn contains_shape(&self, s: &Shape) -> bool {
        match self {
            Shape::Capsule(us) => us.contains_shape(s),
            Shape::CapsuleExcl(us) => us.contains_shape(s),
            Shape::Circle(us) => us.contains_shape(s),
            Shape::CircleExcl(us) => us.contains_shape(s),
            Shape::Compound(us) => us.contains_shape(s),
            Shape::Line(us) => us.contains_shape(s),
            Shape::Path(us) => us.contains_shape(s),
            Shape::PathExcl(us) => us.contains_shape(s),
            Shape::Point(us) => us.contains_shape(s),
            Shape::Poly(us) => us.contains_shape(s),
            Shape::PolyExcl(us) => us.contains_shape(s),
            Shape::Rect(us) => us.contains_shape(s),
            Shape::RectExcl(us) => us.contains_shape(s),
            Shape::Segment(us) => us.contains_shape(s),
            Shape::Tri(us) => us.contains_shape(s),
            Shape::TriExcl(us) => us.contains_shape(s),
        }
    }

    fn dist_to_shape(&self, s: &Shape) -> Option<f64> {
        match self {
            Shape::Capsule(us) => us.dist_to_shape(s),
            Shape::CapsuleExcl(us) => us.dist_to_shape(s),
            Shape::Circle(us) => us.dist_to_shape(s),
            Shape::CircleExcl(us) => us.dist_to_shape(s),
            Shape::Compound(us) => us.dist_to_shape(s),
            Shape::Line(us) => us.dist_to_shape(s),
            Shape::Path(us) => us.dist_to_shape(s),
            Shape::PathExcl(us) => us.dist_to_shape(s),
            Shape::Point(us) => us.dist_to_shape(s),
            Shape::Poly(us) => us.dist_to_shape(s),
            Shape::PolyExcl(us) => us.dist_to_shape(s),
            Shape::Rect(us) => us.dist_to_shape(s),
            Shape::RectExcl(us) => us.dist_to_shape(s),
            Shape::Segment(us) => us.dist_to_shape(s),
            Shape::Tri(us) => us.dist_to_shape(s),
            Shape::TriExcl(us) => us.dist_to_shape(s),
        }
    }
}
