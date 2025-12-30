use crate::geom::contains::poly_contains_pt;
use crate::geom::intersects::{
    cap_intersects_poly, circ_intersects_poly, circ_intersects_rt, poly_intersects_rt,
    rt_intersects_seg, seg_intersects_seg,
};
use crate::geom::math::eq;
use crate::primitive::capsule::CapsulePrimitive;
use crate::primitive::circle::CirclePrimitive;
use crate::primitive::line_shape::LinePrimitive;
use crate::primitive::path_shape::PathPrimitive;
use crate::primitive::point::Pt;
use crate::primitive::polygon::{PolyPrimitive, edges};
use crate::primitive::rect::RtPrimitive;
use crate::primitive::segment::SegmentPrimitive;
use crate::primitive::{Boundary, pt, seg};

// Distance functions should return 0 if there is intersection or containment.
// This property is used by quadtree which returns 0 if it detects an intersection
// by e.g. regular intersection tests.

// Returns None if the iterator is empty.
fn min_dist_opt(iter: impl Iterator<Item = f64>) -> Option<f64> {
    let mut best: Option<f64> = None;
    for i in iter {
        best = Some(best.unwrap_or(i).min(i));
        if eq(i, 0.0) {
            return best;
        }
    }
    best
}

#[must_use]
pub fn cap_cap_dist<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &CapsulePrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    let d = seg_seg_dist(&a.seg(), &b.seg()) - a.r() - b.r();
    Some(d.max(0.0))
}

#[must_use]
pub fn cap_circ_dist<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &CirclePrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    let d = pt_seg_dist(&b.p(), &a.seg()) - a.r() - b.r();
    Some(d.max(0.0))
}

#[must_use]
pub fn cap_path_dist<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &PathPrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    min_dist_opt(b.caps().filter_map(|cap| cap_cap_dist(a, &cap)))
}

#[must_use]
pub fn cap_poly_dist<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &PolyPrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    if cap_intersects_poly(a, b) {
        Some(0.0)
    } else {
        min_dist_opt(b.edges().filter_map(|[&p0, &p1]| cap_seg_dist(a, &seg(p0, p1))))
    }
}

#[must_use]
pub fn cap_rt_dist<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &RtPrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    let d = rt_seg_dist(b, &a.seg())? - a.r();
    Some(d.max(0.0))
}

#[must_use]
pub fn cap_seg_dist<const B: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &SegmentPrimitive,
) -> Option<f64> {
    if a.is_empty_set() {
        return None;
    }
    let d = seg_seg_dist(&a.seg(), b) - a.r();
    Some(d.max(0.0))
}

#[must_use]
pub fn circ_circ_dist<const B: Boundary, const B2: Boundary>(
    a: &CirclePrimitive<B>,
    b: &CirclePrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    let d = pt_pt_dist(&a.p(), &b.p()) - a.r() - b.r();
    Some(d.max(0.0))
}

#[must_use]
pub fn circ_path_dist<const B: Boundary, const B2: Boundary>(
    a: &CirclePrimitive<B>,
    b: &PathPrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    min_dist_opt(b.caps().filter_map(|cap| cap_circ_dist(&cap, a)))
}

#[must_use]
pub fn circ_poly_dist<const B: Boundary, const B2: Boundary>(
    a: &CirclePrimitive<B>,
    b: &PolyPrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    if circ_intersects_poly(a, b) {
        Some(0.0)
    } else {
        let d = poly_pt_dist(b, &a.p())? - a.r();
        Some(d.max(0.0))
    }
}

#[must_use]
pub fn circ_rt_dist<const B: Boundary, const B2: Boundary>(
    a: &CirclePrimitive<B>,
    b: &RtPrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    if circ_intersects_rt(a, b) {
        Some(0.0)
    } else {
        // Project circle centre onto the rectangle:
        let p = a.p().clamp(b);
        Some((p.dist(a.p()) - a.r()).max(0.0))
    }
}

#[must_use]
pub fn circ_pt_dist<const B: Boundary>(a: &CirclePrimitive<B>, b: &Pt) -> Option<f64> {
    if a.is_empty_set() {
        return None;
    }
    Some((pt_pt_dist(&a.p(), b) - a.r()).max(0.0))
}

#[must_use]
pub fn line_pt_dist(a: &LinePrimitive, b: &Pt) -> f64 {
    b.dist(a.project(*b))
}

#[must_use]
pub fn path_poly_dist<const B: Boundary, const B2: Boundary>(
    a: &PathPrimitive<B>,
    b: &PolyPrimitive<B2>,
) -> Option<f64> {
    if a.pts().is_empty() || b.pts().is_empty() {
        return None;
    }
    // cap_poly_dist returns Some for non-empty poly, so unwrap is safe here
    min_dist_opt(a.caps().map(|cap| cap_poly_dist(&cap, b).unwrap()))
}

// Distance to a polygon outline. Returns None if the polygon is empty.
#[must_use]
pub fn polyline_pt_dist(a: &[Pt], b: &Pt) -> Option<f64> {
    min_dist_opt(edges(a).map(|[&p0, &p1]| pt_seg_dist(b, &seg(p0, p1))))
}

#[must_use]
pub fn poly_pt_dist<const B: Boundary>(a: &PolyPrimitive<B>, b: &Pt) -> Option<f64> {
    if a.pts().is_empty() {
        None
    } else if poly_contains_pt(a, b) {
        Some(0.0)
    } else {
        polyline_pt_dist(a.pts(), b)
    }
}

#[must_use]
pub fn poly_rt_dist<const B: Boundary, const B2: Boundary>(
    a: &PolyPrimitive<B>,
    b: &RtPrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    if poly_intersects_rt(a, b) {
        Some(0.0)
    } else {
        min_dist_opt(a.edges().filter_map(|[&p0, &p1]| rt_seg_dist(b, &seg(p0, p1))))
    }
}

#[must_use]
pub fn pt_pt_dist(a: &Pt, b: &Pt) -> f64 {
    a.dist(*b)
}

#[must_use]
pub fn pt_rt_dist<const B: Boundary>(a: &Pt, b: &RtPrimitive<B>) -> Option<f64> {
    if b.is_empty_set() {
        return None;
    }
    if b.contains(*a) {
        Some(0.0)
    } else {
        // Project centre onto the rectangle:
        let p = a.clamp(b);
        Some(p.dist(*a))
    }
}

#[must_use]
pub fn pt_seg_dist(a: &Pt, b: &SegmentPrimitive) -> f64 {
    let st_dist = a.dist(b.st());
    let en_dist = a.dist(b.en());
    let project = b.line().project(*a);
    let dist = st_dist.min(en_dist);
    if b.contains(project) { dist.min(a.dist(project)) } else { dist }
}

#[must_use]
pub fn rt_path_dist<const B: Boundary, const B2: Boundary>(
    a: &RtPrimitive<B>,
    b: &PathPrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    min_dist_opt(b.caps().filter_map(|cap| cap_rt_dist(&cap, a)))
}

#[must_use]
pub fn rt_rt_dist<const B: Boundary, const B2: Boundary>(
    a: &RtPrimitive<B>,
    b: &RtPrimitive<B2>,
) -> Option<f64> {
    if a.is_empty_set() || b.is_empty_set() {
        return None;
    }
    // Compute shortest distance between each axis.
    // Select a non-negative distance. Not possible for both axis differences to be positive.
    let x = (a.l() - b.r()).max(b.l() - a.r()).max(0.0);
    let y = (a.b() - b.t()).max(b.b() - a.t()).max(0.0);
    Some(pt(x, y).mag())
}

#[must_use]
pub fn rt_seg_dist<const B: Boundary>(a: &RtPrimitive<B>, b: &SegmentPrimitive) -> Option<f64> {
    if a.is_empty_set() {
        return None;
    }
    if rt_intersects_seg(a, b) {
        Some(0.0)
    } else {
        // Check for closest distance from the segment to the edges of the rectangle.
        // Rect always has 4 segments, so this always produces Some.
        min_dist_opt(a.segs().iter().map(|seg| seg_seg_dist(seg, b)))
    }
}

#[must_use]
pub fn seg_seg_dist(a: &SegmentPrimitive, b: &SegmentPrimitive) -> f64 {
    // Closest distance must be between an endpoint and a segment, unless
    // the segments cross, in which case it is zero.
    if seg_intersects_seg(a, b) {
        return 0.0;
    }
    let mut best = pt_seg_dist(&a.st(), b);
    best = best.min(pt_seg_dist(&a.en(), b));
    best = best.min(pt_seg_dist(&b.st(), a));
    best = best.min(pt_seg_dist(&b.en(), a));
    best
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;
    use crate::geom::math::EP;
    use crate::primitive::{cap, circ, line, pt, rt};

    #[test]
    fn circ_circ() {
        let circ1 = circ(pt(0.0, 0.0), 0.4);
        assert_relative_eq!(0.0, circ_circ_dist(&circ1, &circ1).unwrap());
        assert_relative_eq!(
            130.94659781997535,
            circ_circ_dist(&circ(pt(111.6414, -70.632), 0.762), &circ1).unwrap()
        );
    }

    #[test]
    fn cap_cap() {
        let cap1 = cap(pt(47.0, -119.4), pt(47.8, -118.6), 0.125);
        let cap2 = cap(pt(47.0, -119.8), pt(46.6, -120.2), 0.125);

        assert_relative_eq!(0.15, cap_cap_dist(&cap1, &cap2).unwrap(), epsilon = EP);
    }

    #[test]
    fn cap_circ() {
        let cap = cap(pt(19.8, -100.6), pt(35.8, -100.6), 0.125);
        let circ = circ(pt(24.5, -98.25), 2.05);

        assert_relative_eq!(0.175, cap_circ_dist(&cap, &circ).unwrap(), epsilon = EP);
    }

    #[test]
    fn rt_rt() {
        let rt1 = rt(0.0, 0.0, 1.0, 1.0);

        assert_relative_eq!(0.0, rt_rt_dist(&rt1, &rt1).unwrap(), epsilon = EP);
        assert_relative_eq!(0.0, rt_rt_dist(&rt(1.0, 1.0, 2.0, 2.0), &rt1).unwrap(), epsilon = EP);
        assert_relative_eq!(1.0, rt_rt_dist(&rt(2.0, 0.5, 2.0, 2.0), &rt1).unwrap(), epsilon = EP);
        assert_relative_eq!(
            1.0,
            rt_rt_dist(&rt(-2.0, 0.5, -1.0, 2.0), &rt1).unwrap(),
            epsilon = EP
        );
        assert_relative_eq!(
            2.0_f64.sqrt(),
            rt_rt_dist(&rt(2.0, 2.0, 3.0, 3.0), &rt1).unwrap(),
            epsilon = EP
        );
    }

    #[test]
    fn line_pt_dist_degenerate_line() {
        let l = line(pt(1.0, 2.0), pt(1.0, 2.0));
        let p = pt(2.0, 2.0);

        assert_relative_eq!(line_pt_dist(&l, &p), 1.0, epsilon = EP);
    }

    #[test]
    fn circ_rt_dist_never_negative() {
        // Circle just touching rectangle - distance should be 0, not negative
        let c = circ(pt(0.0, 0.0), 1.0);
        let r = rt(1.0, -1.0, 2.0, 1.0);
        let dist = circ_rt_dist(&c, &r).unwrap();
        assert!(dist >= 0.0, "circ_rt_dist returned negative value: {dist}");
    }

    #[test]
    fn circ_rt_dist_non_intersecting() {
        // Circle clearly separated from rectangle
        let c = circ(pt(0.0, 0.0), 1.0);
        let r = rt(3.0, 0.0, 4.0, 1.0);
        let dist = circ_rt_dist(&c, &r).unwrap();
        assert!(dist >= 0.0);
        assert_relative_eq!(dist, 2.0, epsilon = EP);
    }
}
