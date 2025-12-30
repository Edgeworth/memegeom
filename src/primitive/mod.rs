use crate::primitive::capsule::CapsulePrimitive;
use crate::primitive::circle::CirclePrimitive;
use crate::primitive::line_shape::LinePrimitive;
use crate::primitive::path_shape::PathPrimitive;
use crate::primitive::polygon::PolyPrimitive;
use crate::primitive::rect::RtPrimitive;
use crate::primitive::segment::SegmentPrimitive;
use crate::primitive::triangle::TriPrimitive;

pub mod capsule;
pub mod circle;
pub mod compound;
pub mod line_shape;
pub mod path_shape;
pub mod point;
pub mod polygon;
pub mod rect;
pub mod segment;
pub mod shape;
pub mod triangle;

/// Specifies whether a shape's boundary is included or excluded.
#[must_use]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default, std::marker::ConstParamTy)]
pub enum Boundary {
    /// Shape includes its boundary (closed set).
    #[default]
    Include,
    /// Shape excludes its boundary (open set).
    Exclude,
}

// Type aliases - Include boundary by default
pub type Capsule = CapsulePrimitive<{ Boundary::Include }>;
pub type CapsuleExcl = CapsulePrimitive<{ Boundary::Exclude }>;
pub type Circle = CirclePrimitive<{ Boundary::Include }>;
pub type CircleExcl = CirclePrimitive<{ Boundary::Exclude }>;
pub type Line = LinePrimitive;
pub type Path = PathPrimitive<{ Boundary::Include }>;
pub type PathExcl = PathPrimitive<{ Boundary::Exclude }>;
pub type Poly = PolyPrimitive<{ Boundary::Include }>;
pub type PolyExcl = PolyPrimitive<{ Boundary::Exclude }>;
pub type Rt = RtPrimitive<{ Boundary::Include }>;
pub type RtExcl = RtPrimitive<{ Boundary::Exclude }>;
pub type Segment = SegmentPrimitive;
pub type Tri = TriPrimitive<{ Boundary::Include }>;
pub type TriExcl = TriPrimitive<{ Boundary::Exclude }>;

pub use compound::Compound;
pub use point::{Pt, PtI};
pub use rect::RtI;
pub use shape::Shape;

pub trait ShapeOps {
    fn bounds(&self) -> Option<Rt>;
    fn shape(self) -> Shape;

    /// Returns true iff this shape is the empty set (contains no points).
    fn is_empty_set(&self) -> bool;

    /// Returns true iff the two shapes have at least one point in common.
    /// The empty set intersects nothing (including itself).
    fn intersects_shape(&self, s: &Shape) -> bool;

    /// Returns true iff all points in `s` are contained within this shape.
    /// The empty set is contained by everything.
    /// The empty set contains only the empty set.
    fn contains_shape(&self, s: &Shape) -> bool;

    /// Returns the shortest distance between any pair of points in the two shapes.
    /// Returns None if either shape is the empty set.
    fn dist_to_shape(&self, s: &Shape) -> Option<f64>;
}

// Capsule helpers
pub fn cap_prim<const B: Boundary>(st: Pt, en: Pt, r: f64) -> CapsulePrimitive<B> {
    CapsulePrimitive::new(st, en, r)
}
pub fn cap(st: Pt, en: Pt, r: f64) -> Capsule {
    cap_prim(st, en, r)
}
pub fn cap_excl(st: Pt, en: Pt, r: f64) -> CapsuleExcl {
    cap_prim(st, en, r)
}

// Circle helpers
pub fn circ_prim<const B: Boundary>(p: Pt, r: f64) -> CirclePrimitive<B> {
    CirclePrimitive::new(p, r)
}
pub fn circ(p: Pt, r: f64) -> Circle {
    circ_prim(p, r)
}
pub fn circ_excl(p: Pt, r: f64) -> CircleExcl {
    circ_prim(p, r)
}

// Line helpers
pub const fn line(st: Pt, en: Pt) -> Line {
    LinePrimitive::new(st, en)
}

// Path helpers
pub fn path_prim<const B: Boundary>(pts: &[Pt], r: f64) -> PathPrimitive<B> {
    PathPrimitive::new(pts, r)
}
pub fn path(pts: &[Pt], r: f64) -> Path {
    path_prim(pts, r)
}
pub fn path_excl(pts: &[Pt], r: f64) -> PathExcl {
    path_prim(pts, r)
}

// Point helpers
pub const fn pt(x: f64, y: f64) -> Pt {
    Pt::new(x, y)
}

pub const fn pti(x: i64, y: i64) -> PtI {
    PtI::new(x, y)
}

// Polygon helpers
pub fn poly_prim<const B: Boundary>(pts: &[Pt]) -> PolyPrimitive<B> {
    PolyPrimitive::new(pts)
}
pub fn poly(pts: &[Pt]) -> Poly {
    poly_prim(pts)
}
pub fn poly_excl(pts: &[Pt]) -> PolyExcl {
    poly_prim(pts)
}

// Rect helpers
pub const fn rt_prim<const B: Boundary>(l: f64, b: f64, r: f64, t: f64) -> RtPrimitive<B> {
    RtPrimitive::new(l, b, r, t)
}
pub const fn rt(l: f64, b: f64, r: f64, t: f64) -> Rt {
    rt_prim(l, b, r, t)
}
pub const fn rt_excl(l: f64, b: f64, r: f64, t: f64) -> RtExcl {
    rt_prim(l, b, r, t)
}

// Segment helpers
pub const fn seg(st: Pt, en: Pt) -> Segment {
    SegmentPrimitive::new(st, en)
}

// Triangle helpers
pub fn tri_prim<const B: Boundary>(a: Pt, b: Pt, c: Pt) -> TriPrimitive<B> {
    TriPrimitive::new([a, b, c])
}
pub fn tri(a: Pt, b: Pt, c: Pt) -> Tri {
    tri_prim(a, b, c)
}
pub fn tri_excl(a: Pt, b: Pt, c: Pt) -> TriExcl {
    tri_prim(a, b, c)
}
