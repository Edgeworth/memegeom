use auto_ops::{impl_op_ex, impl_op_ex_commutative};
use derive_more::Display;

use crate::geom::contains::{
    rt_contains_cap, rt_contains_circ, rt_contains_path, rt_contains_poly, rt_contains_seg,
    rt_contains_tri,
};
use crate::geom::distance::{
    cap_rt_dist, circ_rt_dist, poly_rt_dist, pt_rt_dist, rt_path_dist, rt_rt_dist, rt_seg_dist,
};
use crate::geom::intersects::{
    cap_intersects_rt, circ_intersects_rt, path_intersects_rt, poly_intersects_rt,
    rt_intersects_rt, rt_intersects_seg, rt_intersects_tri,
};
use crate::geom::math::{eq, ge, gt, le, lt};
use crate::primitive::point::{Pt, PtI};
use crate::primitive::segment::Segment;
use crate::primitive::shape::Shape;
use crate::primitive::{ShapeOps, pt, pti, rt, seg};

#[must_use]
#[derive(Debug, Copy, Clone, Display)]
#[display("({l}, {b}, {r}, {t})")]
pub struct Rt {
    l: f64,
    b: f64,
    r: f64,
    t: f64,
}

impl Default for Rt {
    fn default() -> Self {
        Self::empty()
    }
}

// Rt covers the range [l, r] . [b, t]. Empty rectangles have r < l or t < b.
// Note that a rectangle with l == r and b == t may not be empty although have
// a width and height of zero.
impl Rt {
    pub const fn new(l: f64, b: f64, r: f64, t: f64) -> Self {
        Self { l, b, r, t }
    }

    pub const fn empty() -> Self {
        rt(0.0, 0.0, -1.0, -1.0)
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        lt(self.r, self.l) || lt(self.t, self.b)
    }

    #[must_use]
    pub fn w(&self) -> f64 {
        self.r - self.l
    }

    #[must_use]
    pub fn h(&self) -> f64 {
        self.t - self.b
    }

    #[must_use]
    pub const fn l(&self) -> f64 {
        self.l
    }

    #[must_use]
    pub const fn t(&self) -> f64 {
        self.t
    }

    #[must_use]
    pub const fn r(&self) -> f64 {
        self.r
    }

    #[must_use]
    pub const fn b(&self) -> f64 {
        self.b
    }

    pub const fn bl(&self) -> Pt {
        pt(self.l(), self.b())
    }

    pub fn bl_quadrant(&self) -> Rt {
        rt(self.l(), self.b(), self.center().x, self.center().y)
    }

    pub const fn br(&self) -> Pt {
        pt(self.r(), self.b())
    }

    pub fn br_quadrant(&self) -> Rt {
        rt(self.center().x, self.b(), self.r(), self.center().y)
    }

    pub const fn tl(&self) -> Pt {
        pt(self.l(), self.t())
    }

    pub fn tl_quadrant(&self) -> Rt {
        rt(self.l(), self.center().y, self.center().x, self.t())
    }

    pub const fn tr(&self) -> Pt {
        pt(self.r(), self.t())
    }

    pub fn tr_quadrant(&self) -> Rt {
        rt(self.center().x, self.center().y, self.r(), self.t())
    }

    pub const fn pts(&self) -> [Pt; 4] {
        [self.bl(), self.br(), self.tr(), self.tl()]
    }

    pub fn segs(&self) -> [Segment; 4] {
        let pts = self.pts();
        [seg(pts[0], pts[1]), seg(pts[1], pts[2]), seg(pts[2], pts[3]), seg(pts[3], pts[0])]
    }

    pub fn center(&self) -> Pt {
        pt((self.l + self.r) / 2.0, (self.b + self.t) / 2.0)
    }

    #[must_use]
    pub fn area(&self) -> f64 {
        self.w() * self.h()
    }

    // Insetting a rectangle more than its size will produce a rectangle
    // containing the single center point.
    pub fn inset(&self, dx: f64, dy: f64) -> Rt {
        let wsub = self.w().min(2.0 * dx) / 2.0;
        let hsub = self.h().min(2.0 * dy) / 2.0;
        rt(self.l + wsub, self.b + hsub, self.r - wsub, self.t - hsub)
    }

    #[must_use]
    pub fn contains(&self, p: Pt) -> bool {
        ge(p.x, self.l()) && ge(p.y, self.b()) && le(p.x, self.r()) && le(p.y, self.t())
    }

    #[must_use]
    pub fn contains_rt(&self, r: &Rt) -> bool {
        ge(r.l(), self.l()) && le(r.r(), self.r()) && ge(r.b(), self.b()) && le(r.t(), self.t())
    }

    #[must_use]
    pub fn intersects(&self, r: &Rt) -> bool {
        le(self.l(), r.r()) && ge(self.r(), r.l()) && gt(self.t(), r.b()) && le(self.b(), r.t())
    }

    pub fn united(&self, rect: &Rt) -> Rt {
        if rect.is_empty() {
            *self
        } else if self.is_empty() {
            *rect
        } else {
            let l = self.l.min(rect.l);
            let b = self.b.min(rect.b);
            let r = self.r().max(rect.r());
            let t = self.t().max(rect.t());
            rt(l, b, r, t)
        }
    }

    pub fn enclosing(pa: Pt, pb: Pt) -> Rt {
        let l = pa.x.min(pb.x);
        let b = pa.y.min(pb.y);
        let r = pa.x.max(pb.x);
        let t = pa.y.max(pb.y);
        rt(l, b, r, t)
    }

    // Returns a rectangle with the same area that matches the aspect ratio of |r|.
    pub fn match_aspect(&self, r: &Rt) -> Rt {
        if eq(r.w(), 0.0) {
            rt(self.l, self.b, self.l, self.t)
        } else if eq(r.h(), 0.0) {
            rt(self.l, self.b, self.r, self.b)
        } else {
            let aspect = (r.w() / r.h()).sqrt();
            let len = self.area().sqrt();
            rt(self.l, self.b, self.l + len * aspect, self.b + len / aspect)
        }
    }
}

impl PartialEq for Rt {
    fn eq(&self, o: &Self) -> bool {
        self.tl() == o.tl() && self.br() == o.br()
    }
}

impl ShapeOps for Rt {
    fn bounds(&self) -> Rt {
        *self
    }

    fn shape(self) -> Shape {
        Shape::Rect(self)
    }

    fn intersects_shape(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(s) => cap_intersects_rt(s, self),
            Shape::Circle(s) => circ_intersects_rt(s, self),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(s) => path_intersects_rt(s, self),
            Shape::Point(s) => self.contains(*s),
            Shape::Polygon(s) => poly_intersects_rt(s, self),
            Shape::Rect(s) => rt_intersects_rt(self, s),
            Shape::Segment(s) => rt_intersects_seg(self, s),
            Shape::Tri(s) => rt_intersects_tri(self, s),
        }
    }

    fn contains_shape(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(s) => rt_contains_cap(self, s),
            Shape::Circle(s) => rt_contains_circ(self, s),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(s) => rt_contains_path(self, s),
            Shape::Point(s) => self.contains(*s),
            Shape::Polygon(s) => rt_contains_poly(self, s),
            Shape::Rect(s) => self.contains_rt(s),
            Shape::Segment(s) => rt_contains_seg(self, s),
            Shape::Tri(s) => rt_contains_tri(self, s),
        }
    }

    fn dist_to_shape(&self, s: &Shape) -> f64 {
        match s {
            Shape::Capsule(s) => cap_rt_dist(s, self),
            Shape::Circle(s) => circ_rt_dist(s, self),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(s) => rt_path_dist(self, s),
            Shape::Point(s) => pt_rt_dist(s, self),
            Shape::Polygon(s) => poly_rt_dist(s, self),
            Shape::Rect(s) => rt_rt_dist(self, s),
            Shape::Segment(s) => rt_seg_dist(self, s),
            Shape::Tri(_) => todo!(),
        }
    }
}

impl_op_ex_commutative!(*|a: &Rt, b: &f64| -> Rt { rt(a.l * b, a.b * b, a.r * b, a.t * b) });
impl_op_ex_commutative!(/|a: &Rt, b: &f64| -> Rt { rt(a.l / b, a.b / b, a.r / b, a.t / b) });
impl_op_ex_commutative!(*|a: &Rt, b: &i64| -> Rt {
    let b = *b as f64;
    rt(a.l * b, a.b * b, a.r * b, a.t * b)
});
impl_op_ex_commutative!(/|a: &Rt, b: &i64| -> Rt {
    let b = *b as f64;
    rt(a.l / b, a.b / b, a.r / b, a.t / b)
});

#[must_use]
#[derive(Debug, Default, PartialEq, Eq, Copy, Clone, Display)]
#[display("({x}, {y}, {w}, {h})")]
pub struct RtI {
    x: i64,
    y: i64,
    w: i64,
    h: i64,
}

impl RtI {
    pub const fn new(x: i64, y: i64, w: i64, h: i64) -> Self {
        Self { x, y, w, h }
    }

    #[must_use]
    pub const fn w(&self) -> i64 {
        self.w
    }

    #[must_use]
    pub const fn h(&self) -> i64 {
        self.h
    }

    #[must_use]
    pub const fn l(&self) -> i64 {
        self.x
    }

    #[must_use]
    pub const fn t(&self) -> i64 {
        self.y + self.h
    }

    #[must_use]
    pub const fn r(&self) -> i64 {
        self.x + self.w
    }

    #[must_use]
    pub const fn b(&self) -> i64 {
        self.y
    }

    pub const fn bl(&self) -> PtI {
        pti(self.l(), self.b())
    }

    pub const fn br(&self) -> PtI {
        pti(self.r(), self.b())
    }

    pub const fn tl(&self) -> PtI {
        pti(self.l(), self.t())
    }

    pub const fn tr(&self) -> PtI {
        pti(self.r(), self.t())
    }

    pub fn inset(&self, dx: i64, dy: i64) -> RtI {
        let wsub = if 2 * dx < self.w { 2 * dx } else { self.w };
        let hsub = if 2 * dy < self.h { 2 * dy } else { self.h };
        RtI::new(self.x + wsub / 2, self.y + hsub / 2, self.w - wsub, self.h - hsub)
    }

    pub fn enclosing(pa: PtI, pb: PtI) -> RtI {
        let x = pa.x.min(pb.x);
        let y = pa.y.min(pb.y);
        let r = pa.x.max(pb.x);
        let t = pa.y.max(pb.y);
        RtI::new(x, y, r - x, t - y)
    }
}

impl_op_ex!(+ |a: &RtI, b: &RtI| -> RtI { RtI::new(a.x + b.x, a.y + b.y, a.w + b.w, a.h + b.h) });
impl_op_ex!(+= |a: &mut RtI, b: &RtI| { a.x += b.x; a.y += b.y; a.w += b.w; a.h += b.h; });
impl_op_ex!(-|a: &RtI, b: &RtI| -> RtI { RtI::new(a.x - b.x, a.y - b.y, a.w - b.w, a.h - b.h) });
impl_op_ex!(-= |a: &mut RtI, b: &RtI| { a.x -= b.x; a.y -= b.y; a.w -= b.w; a.h -= b.h; });

impl_op_ex_commutative!(+|a: &RtI, b: &PtI| -> RtI { RtI::new(a.x + b.x, a.y + b.y, a.w, a.h) });
impl_op_ex_commutative!(-|a: &RtI, b: &PtI| -> RtI { RtI::new(a.x - b.x, a.y - b.y, a.w, a.h) });

impl_op_ex_commutative!(*|a: &RtI, b: &i64| -> RtI {
    RtI::new(a.x * b, a.y * b, a.w * b, a.h * b)
});
