use crate::geom::contains::{cap_contains_pt, tri_contains_pt};
use crate::geom::distance::{rt_seg_dist, seg_seg_dist};
use crate::geom::math::{eq, le, lt, ne, orientation, pts_strictly_right_of};
use crate::primitive::capsule::CapsulePrimitive;
use crate::primitive::circle::CirclePrimitive;
use crate::primitive::line_shape::LinePrimitive;
use crate::primitive::path_shape::PathPrimitive;
use crate::primitive::polygon::PolyPrimitive;
use crate::primitive::rect::RtPrimitive;
use crate::primitive::segment::SegmentPrimitive;
use crate::primitive::triangle::TriPrimitive;
use crate::primitive::{Boundary, Rt, cap_prim};

// For intersection: touching at boundary counts only when both shapes include boundaries.
fn both_include<const B: Boundary, const B2: Boundary>() -> bool {
    matches!((B, B2), (Boundary::Include, Boundary::Include))
}

fn dist_intersects<const B: Boundary, const B2: Boundary>(dist: f64, threshold: f64) -> bool {
    if both_include::<B, B2>() { le(dist, threshold) } else { lt(dist, threshold) }
}

// Returns true if bounds don't intersect (allowing early rejection).
// Returns false if either has no bounds or bounds do intersect.
fn bounds_disjoint(a: Option<Rt>, b: Option<Rt>) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => !a.intersects(&b),
        _ => false,
    }
}

fn bounds_disjoint_rt<const B: Boundary>(a: Option<Rt>, b: &RtPrimitive<B>) -> bool {
    match a {
        Some(a) => !a.intersects(b),
        None => false,
    }
}

#[must_use]
pub fn cap_intersects_cap<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &CapsulePrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    if bounds_disjoint(a.bounds(), b.bounds()) {
        return false;
    }
    dist_intersects::<B, B2>(seg_seg_dist(&a.seg(), &b.seg()), a.r() + b.r())
}

#[must_use]
pub fn cap_intersects_circ<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &CirclePrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    // Compute minkowski sum of |a| and |b| and check containment.
    // The sum capsule inherits the combined boundary semantics.
    if both_include::<B, B2>() {
        let sum: CapsulePrimitive<{ Boundary::Include }> = cap_prim(a.st(), a.en(), a.r() + b.r());
        cap_contains_pt(&sum, &b.p())
    } else {
        let sum: CapsulePrimitive<{ Boundary::Exclude }> = cap_prim(a.st(), a.en(), a.r() + b.r());
        cap_contains_pt(&sum, &b.p())
    }
}

#[must_use]
pub fn cap_intersects_path<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &PathPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    // Check if any cap in the path intersects this cap.
    for cap in b.caps() {
        if cap_intersects_cap(a, &cap) {
            return true;
        }
    }
    false
}

#[must_use]
pub fn cap_intersects_poly<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &PolyPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    for tri in b.tri() {
        if cap_intersects_tri(a, tri) {
            return true;
        }
    }
    false
}

#[must_use]
pub fn cap_intersects_rt<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &RtPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    if bounds_disjoint_rt(a.bounds(), b) {
        return false;
    }
    if b.contains(a.st()) || b.contains(a.en()) {
        return true;
    }
    // rt_seg_dist returns Some because we checked b.is_empty() above
    dist_intersects::<B, B2>(rt_seg_dist(b, &a.seg()).unwrap(), a.r())
}

#[must_use]
pub fn cap_intersects_tri<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &TriPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    // Check if the capsule is contained within the triangle:
    if tri_contains_pt(b, &a.st()) || tri_contains_pt(b, &a.en()) {
        return true;
    }
    // Otherwise to intersect, the triangle boundary needs to be intersecting
    // the capsule.
    for seg in b.segs() {
        if dist_intersects::<B, B2>(seg_seg_dist(&a.seg(), &seg), a.r()) {
            return true;
        }
    }
    false
}

#[must_use]
pub fn circ_intersects_circ<const B: Boundary, const B2: Boundary>(
    a: &CirclePrimitive<B>,
    b: &CirclePrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    dist_intersects::<B, B2>(a.p().dist(b.p()), a.r() + b.r())
}

#[must_use]
pub fn circ_intersects_path<const B: Boundary, const B2: Boundary>(
    a: &CirclePrimitive<B>,
    b: &PathPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    // Test all capsules in path against circle.
    for cap in b.caps() {
        if cap_intersects_circ(&cap, a) {
            return true;
        }
    }
    false
}

#[must_use]
pub fn circ_intersects_poly<const B: Boundary, const B2: Boundary>(
    a: &CirclePrimitive<B>,
    b: &PolyPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    if bounds_disjoint(a.bounds(), b.bounds()) {
        return false;
    }
    for tri in b.tri() {
        if circ_intersects_tri(a, tri) {
            return true;
        }
    }
    false
}

#[must_use]
pub fn circ_intersects_rt<const B: Boundary, const B2: Boundary>(
    a: &CirclePrimitive<B>,
    b: &RtPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    if bounds_disjoint_rt(a.bounds(), b) {
        return false;
    }
    // Check if the circle centre is contained in the rect or
    // the distance from the boundary of the rect to the circle is less than 0.
    // Project circle centre onto the rectangle:
    let p = a.p().clamp(b);
    let d = p.dist(a.p());
    b.contains(a.p()) || dist_intersects::<B, B2>(d, a.r())
}

#[must_use]
pub fn circ_intersects_tri<const B: Boundary, const B2: Boundary>(
    a: &CirclePrimitive<B>,
    b: &TriPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    if bounds_disjoint(a.bounds(), b.bounds()) {
        return false;
    }
    // Take the minkowski sum of the circle and triangle. Just need to test
    // if the triangle contains the centre of the circle or any of its
    // capsules contain the point.
    if tri_contains_pt(b, &a.p()) {
        return true;
    }
    for seg in b.segs() {
        // The capsule boundary combines both boundaries.
        if both_include::<B, B2>() {
            let c: CapsulePrimitive<{ Boundary::Include }> = cap_prim(seg.st(), seg.en(), a.r());
            if cap_contains_pt(&c, &a.p()) {
                return true;
            }
        } else {
            let c: CapsulePrimitive<{ Boundary::Exclude }> = cap_prim(seg.st(), seg.en(), a.r());
            if cap_contains_pt(&c, &a.p()) {
                return true;
            }
        }
    }
    false
}

#[must_use]
pub fn line_intersects_line(a: &LinePrimitive, b: &LinePrimitive) -> bool {
    let a_dir = a.dir();
    let b_dir = b.dir();
    let a_is_point = eq(a_dir.mag2(), 0.0);
    let b_is_point = eq(b_dir.mag2(), 0.0);

    if a_is_point && b_is_point {
        return a.st() == b.st();
    }
    if a_is_point {
        return orientation(b, a.st()) == 0;
    }
    if b_is_point {
        return orientation(a, b.st()) == 0;
    }

    // Intersects if not parallel, otherwise intersects iff collinear.
    ne(a_dir.cross(b_dir), 0.0) || orientation(a, b.st()) == 0
}

#[must_use]
pub fn line_intersects_seg(_a: &LinePrimitive, _b: &SegmentPrimitive) -> bool {
    todo!()
}

#[must_use]
pub fn path_intersects_path<const B: Boundary, const B2: Boundary>(
    a: &PathPrimitive<B>,
    b: &PathPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    for cap_a in a.caps() {
        for cap_b in b.caps() {
            if cap_intersects_cap(&cap_a, &cap_b) {
                return true;
            }
        }
    }
    false
}

#[must_use]
pub fn path_intersects_rt<const B: Boundary, const B2: Boundary>(
    a: &PathPrimitive<B>,
    b: &RtPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    // Check whether each capsule in the path intersects the rectangle.
    for cap in a.caps() {
        if cap_intersects_rt(&cap, b) {
            return true;
        }
    }
    false
}

#[must_use]
pub fn path_intersects_poly<const B: Boundary, const B2: Boundary>(
    a: &PathPrimitive<B>,
    b: &PolyPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    // Check path capsules.
    for cap in a.caps() {
        if cap_intersects_poly(&cap, b) {
            return true;
        }
    }
    false
}

#[must_use]
pub fn poly_intersects_rt<const B: Boundary, const B2: Boundary>(
    a: &PolyPrimitive<B>,
    b: &RtPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    for tri in a.tri() {
        if rt_intersects_tri(b, tri) {
            return true;
        }
    }
    false
}

#[must_use]
pub fn rt_intersects_rt<const B: Boundary, const B2: Boundary>(
    a: &RtPrimitive<B>,
    b: &RtPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    a.intersects(b)
}

#[must_use]
pub fn rt_intersects_tri<const B: Boundary, const B2: Boundary>(
    a: &RtPrimitive<B>,
    b: &TriPrimitive<B2>,
) -> bool {
    if a.is_empty_set() || b.is_empty_set() {
        return false;
    }
    let rt = &a.pts();
    let tri = b.pts();
    // Test tri axes:
    for seg in b.segs() {
        if pts_strictly_right_of(&seg.line(), rt) {
            return false;
        }
    }
    // Test rect axes:
    for seg in a.segs() {
        if pts_strictly_right_of(&seg.line(), tri) {
            return false;
        }
    }
    true
}

#[must_use]
pub fn rt_intersects_seg<const B: Boundary>(a: &RtPrimitive<B>, b: &SegmentPrimitive) -> bool {
    if a.is_empty_set() {
        return false;
    }
    if a.contains(b.st()) || a.contains(b.en()) {
        return true;
    }
    // Test seg axis:
    if pts_strictly_right_of(&b.line(), &a.pts()) {
        return false;
    }
    // Test rect axes:
    for seg in a.segs() {
        if pts_strictly_right_of(&seg.line(), &[b.st(), b.en()]) {
            return false;
        }
    }
    true
}

#[must_use]
pub fn seg_intersects_seg(a: &SegmentPrimitive, b: &SegmentPrimitive) -> bool {
    // Check if the segment endpoints are on opposite sides of the other segment.
    let a_st = orientation(&b.line(), a.st());
    let a_en = orientation(&b.line(), a.en());
    let b_st = orientation(&a.line(), b.st());
    let b_en = orientation(&a.line(), b.en());
    // No collinear points. Everything on different sides.
    if a_st != a_en && b_st != b_en {
        return true;
    }
    // Check collinear cases. Need to check both x and y coordinates to handle
    // vertical and horizontal segments.
    let a_rt = Rt::enclosing(a.st(), a.en());
    let b_rt = Rt::enclosing(b.st(), b.en());
    if a_st == 0 && b_rt.contains(a.st()) {
        return true;
    }
    if a_en == 0 && b_rt.contains(a.en()) {
        return true;
    }
    if b_st == 0 && a_rt.contains(b.st()) {
        return true;
    }
    if b_en == 0 && a_rt.contains(b.en()) {
        return true;
    }
    false
}

#[cfg(test)]
mod tests {
    use itertools::Itertools;
    use pretty_assertions::assert_eq;

    use super::*;
    use crate::primitive::{
        Segment, Tri, cap, cap_excl, circ, circ_excl, path, poly, pt, rt, seg, tri,
    };
    use crate::tf::Tf;

    fn seg_seg_permutations(a: &Segment, b: &Segment, res: bool) {
        // Try each permutation of orderings
        assert_eq!(seg_intersects_seg(a, b), res, "{} {} intersects? {}", a, b, res);
        assert_eq!(seg_intersects_seg(b, a), res, "{} {} intersects? {}", a, b, res);
        let a = seg(a.en(), a.st());
        let b = seg(b.en(), b.st());
        assert_eq!(seg_intersects_seg(&a, &b), res, "{} {} intersects? {}", a, b, res);
        assert_eq!(seg_intersects_seg(&b, &a), res, "{} {} intersects? {}", a, b, res);
    }

    #[test]
    fn seg_seg() {
        let tests = &[
            // Crossing
            (seg(pt(1.0, 1.0), pt(3.0, 4.0)), seg(pt(2.0, 4.0), pt(3.0, 1.0)), true),
            // Shared endpoints, not parallel
            (seg(pt(1.0, 1.0), pt(2.0, 3.0)), seg(pt(2.0, 3.0), pt(4.0, 1.0)), true),
            // Shared endpoints, parallel, one point of intersection
            (seg(pt(1.0, 1.0), pt(3.0, 2.0)), seg(pt(3.0, 2.0), pt(5.0, 3.0)), true),
            // Endpoint abutting segment, perpendicular
            (seg(pt(1.0, 1.0), pt(3.0, 3.0)), seg(pt(2.0, 4.0), pt(4.0, 2.0)), true),
            // Same segments
            (seg(pt(1.0, 1.0), pt(1.0, 1.0)), seg(pt(1.0, 1.0), pt(1.0, 1.0)), true),
            // Parallel and overlapping
            (seg(pt(1.0, 1.0), pt(3.0, 1.0)), seg(pt(2.0, 1.0), pt(4.0, 1.0)), true),
            // Parallel and contained
            (seg(pt(1.0, 1.0), pt(4.0, 1.0)), seg(pt(2.0, 1.0), pt(3.0, 1.0)), true),
            // Parallel segments with one shared endpoint overlapping
            (seg(pt(1.0, 1.0), pt(3.0, 1.0)), seg(pt(1.0, 1.0), pt(4.0, 1.0)), true),
            // Degenerate: One segment is a point, on the other segment.
            (seg(pt(1.0, 1.0), pt(3.0, 1.0)), seg(pt(2.0, 1.0), pt(2.0, 1.0)), true),
            // Degenerate: One segment is a point, on the other segment's endpoint
            (seg(pt(1.0, 1.0), pt(3.0, 1.0)), seg(pt(3.0, 1.0), pt(3.0, 1.0)), true),
            // Degenerate: Same segments and they are points
            (seg(pt(1.0, 1.0), pt(1.0, 1.0)), seg(pt(1.0, 1.0), pt(1.0, 1.0)), true),
            // Parallel, not intersecting
            (seg(pt(1.0, 3.0), pt(3.0, 1.0)), seg(pt(2.0, 4.0), pt(4.0, 2.0)), false),
            // Perpendicular, not intersecting, projection of endpoint onto other is
            // an endpoint
            (seg(pt(1.0, 1.0), pt(3.0, 3.0)), seg(pt(4.0, 2.0), pt(5.0, 1.0)), false),
            // Perpendicular, not intersecting
            (seg(pt(1.0, 1.0), pt(3.0, 3.0)), seg(pt(3.0, 1.0), pt(4.0, 0.0)), false),
            // Degenerate: Both are points, not intersecting
            (seg(pt(1.0, 1.0), pt(1.0, 1.0)), seg(pt(2.0, 1.0), pt(2.0, 1.0)), false),
            // Degenerate: One is a point, collinear with the other segment, not intersecting.
            (seg(pt(1.0, 1.0), pt(3.0, 3.0)), seg(pt(4.0, 4.0), pt(4.0, 4.0)), false),
            // Degenerate: One is a point, not intersecting.
            (seg(pt(1.0, 1.0), pt(3.0, 3.0)), seg(pt(1.0, 2.0), pt(1.0, 2.0)), false),
        ];

        for (a, b, res) in tests {
            seg_seg_permutations(a, b, *res);
            // Negating pts should not change result.
            let a = &seg(-a.st(), -a.en());
            let b = &seg(-b.st(), -b.en());
            seg_seg_permutations(a, b, *res);
            // Rotating should not change result.
            let tf = Tf::rotate(42.0);
            let a = &tf.seg(a);
            let b = &tf.seg(b);
            seg_seg_permutations(a, b, *res);
            // Translating should not change result.
            let tf = Tf::translate(pt(-3.0, 4.0));
            let a = &tf.seg(a);
            let b = &tf.seg(b);
            seg_seg_permutations(a, b, *res);
            // Scaling should not change result.
            let tf = Tf::scale(pt(-0.4, 0.7));
            let a = &tf.seg(a);
            let b = &tf.seg(b);
            seg_seg_permutations(a, b, *res);
        }
    }

    fn permute_tri(t: &Tri) -> Vec<Tri> {
        t.pts().iter().permutations(3).map(|v| tri(*v[0], *v[1], *v[2])).collect()
    }

    #[test]
    fn rt_tri() {
        let tests = &[
            // Regular intersection
            (rt(1.0, 2.0, 3.0, 3.0), tri(pt(2.0, 2.5), pt(2.0, 1.0), pt(3.0, 1.0)), true),
            // Just touching the rect.
            (rt(1.0, 2.0, 3.0, 3.0), tri(pt(3.0, 3.0), pt(4.0, 3.0), pt(4.0, 5.0)), true),
            (rt(1.0, 2.0, 3.0, 3.0), tri(pt(1.0, 4.0), pt(3.0, 4.0), pt(2.0, 5.0)), false),
            (
                rt(14.4, -148.8, 15.20, -148.0),
                tri(pt(52.5, -19.75), pt(34.0, -19.75), pt(15.0, -50.75)),
                false,
            ),
        ];

        for (a, t, res) in tests {
            for b in permute_tri(t) {
                assert_eq!(rt_intersects_tri(a, &b), *res, "{} {} intersect? {}", a, b, res);
            }
        }
    }

    #[test]
    fn cap_rt() {
        let tests = &[
            (cap(pt(1.0, 1.0), pt(7.0, 1.0), 1.0), rt(1.0, 1.0, 2.0, 2.0), true),
            (cap(pt(1.0, 1.0), pt(7.0, 1.0), 1.0), rt(3.0, 1.0, 3.0, 2.0), true),
            (cap(pt(122.8, -44.4), pt(109.2, -44.4), 0.32), rt(113.6, -44.8, 114.4, -44.0), true),
            (cap(pt(1.0, 1.0), pt(7.0, 1.0), 1.0), rt(3.0, 0.0, 3.0, 1.0), true),
            (cap(pt(1.0, 1.0), pt(7.0, 1.0), 1.0), rt(2.0, 3.0, 3.0, 4.0), false),
        ];

        for (a, b, res) in tests {
            assert_eq!(cap_intersects_rt(a, b), *res, "{} {} intersect? {}", a, b, res);
        }
    }

    #[test]
    fn path_path_empty() {
        let empty = path(&[], 0.0);
        let other = path(&[pt(0.0, 0.0), pt(1.0, 0.0)], 0.0);

        assert!(!path_intersects_path(&empty, &other));
        assert!(!path_intersects_path(&other, &empty));
    }

    #[test]
    fn path_path_singleton() {
        let singleton = path(&[pt(0.0, 0.0)], 0.0);
        let other = path(&[pt(0.0, 0.0), pt(1.0, 0.0)], 0.0);

        assert!(path_intersects_path(&singleton, &other));
        assert!(path_intersects_path(&other, &singleton));
    }

    #[test]
    fn path_path_all_segment_pairs() {
        let a = path(&[pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0)], 0.0);
        let b = path(&[pt(1.5, -1.0), pt(1.5, 1.0)], 0.0);

        assert!(path_intersects_path(&a, &b));
        assert!(path_intersects_path(&b, &a));
    }

    #[test]
    fn boundary_touching_circles() {
        // Two circles of radius 1, 2 units apart - touching at boundary
        // Include+Include: touch counts as intersection
        assert!(circ_intersects_circ(&circ(pt(0.0, 0.0), 1.0), &circ(pt(2.0, 0.0), 1.0)));
        // Any exclude: touch doesn't count
        assert!(!circ_intersects_circ(&circ(pt(0.0, 0.0), 1.0), &circ_excl(pt(2.0, 0.0), 1.0)));
        assert!(!circ_intersects_circ(
            &circ_excl(pt(0.0, 0.0), 1.0),
            &circ_excl(pt(2.0, 0.0), 1.0)
        ));
        // Overlapping always intersects
        assert!(circ_intersects_circ(&circ_excl(pt(0.0, 0.0), 1.0), &circ_excl(pt(1.5, 0.0), 1.0)));
    }

    #[test]
    fn boundary_touching_capsules() {
        // Two capsules with radius 1, segments 2 units apart
        assert!(cap_intersects_cap(
            &cap(pt(0.0, 0.0), pt(0.0, 1.0), 1.0),
            &cap(pt(2.0, 0.0), pt(2.0, 1.0), 1.0)
        ));
        assert!(!cap_intersects_cap(
            &cap_excl(pt(0.0, 0.0), pt(0.0, 1.0), 1.0),
            &cap(pt(2.0, 0.0), pt(2.0, 1.0), 1.0)
        ));
    }

    #[test]
    fn boundary_touching_circle_rect() {
        let r = rt(1.0, -1.0, 2.0, 1.0);
        assert!(circ_intersects_rt(&circ(pt(0.0, 0.0), 1.0), &r));
        assert!(!circ_intersects_rt(&circ_excl(pt(0.0, 0.0), 1.0), &r));
    }

    #[test]
    fn boundary_touching_capsule_rect() {
        let r = rt(0.0, 1.0, 1.0, 2.0);
        assert!(cap_intersects_rt(&cap(pt(0.0, 0.0), pt(1.0, 0.0), 1.0), &r));
        assert!(!cap_intersects_rt(&cap_excl(pt(0.0, 0.0), pt(1.0, 0.0), 1.0), &r));
    }

    #[test]
    fn boundary_touching_capsule_tri() {
        let t = tri(pt(0.0, 1.0), pt(2.0, 1.0), pt(1.0, 3.0));
        assert!(cap_intersects_tri(&cap(pt(0.0, 0.0), pt(1.0, 0.0), 1.0), &t));
        assert!(!cap_intersects_tri(&cap_excl(pt(0.0, 0.0), pt(1.0, 0.0), 1.0), &t));
    }

    #[test]
    fn boundary_touching_circle_tri() {
        let t = tri(pt(-1.0, 1.0), pt(1.0, 1.0), pt(0.0, 3.0));
        assert!(circ_intersects_tri(&circ(pt(0.0, 0.0), 1.0), &t));
        assert!(!circ_intersects_tri(&circ_excl(pt(0.0, 0.0), 1.0), &t));
    }

    #[test]
    fn degenerate_shapes() {
        // Zero-radius circle
        assert!(circ_intersects_rt(&circ(pt(1.0, 1.0), 0.0), &rt(1.0, 1.0, 2.0, 2.0)));
        assert!(!circ_intersects_rt(&circ(pt(0.0, 0.0), 0.0), &rt(1.0, 1.0, 2.0, 2.0)));
        // Capsule where st == en (effectively a circle)
        assert!(cap_intersects_rt(&cap(pt(0.0, 0.0), pt(0.0, 0.0), 1.0), &rt(0.5, 0.0, 1.5, 1.0)));
        // Zero-radius capsule (effectively a segment)
        assert!(cap_intersects_rt(
            &cap(pt(0.0, 0.0), pt(1.0, 0.0), 0.0),
            &rt(-0.5, -0.5, 0.5, 0.5)
        ));
        // Point segment
        assert!(rt_intersects_seg(&rt(0.0, 0.0, 2.0, 2.0), &seg(pt(1.0, 1.0), pt(1.0, 1.0))));
        // Degenerate polygons
        assert!(!poly_intersects_rt(&poly(&[]), &rt(0.0, 0.0, 1.0, 1.0)));
        assert!(poly(&[pt(0.5, 0.5)]).tri().is_empty());
        assert!(poly(&[pt(0.0, 0.0), pt(1.0, 0.0)]).tri().is_empty());
    }

    #[test]
    fn path_edge_cases() {
        let empty = path(&[], 0.0);
        let singleton = path(&[pt(0.0, 0.0)], 0.0);
        let other = path(&[pt(0.0, 0.0), pt(1.0, 0.0)], 0.0);
        let r = rt(0.0, 0.0, 1.0, 1.0);

        // Empty path intersects nothing
        assert!(!path_intersects_path(&empty, &other));
        assert!(!path_intersects_rt(&path(&[], 1.0), &r));
        // Singleton path
        assert!(path_intersects_path(&singleton, &other));
        assert!(path_intersects_rt(&path(&[pt(0.5, 0.5)], 0.1), &r));
        // Collinear points reduced
        assert_eq!(path(&[pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0)], 0.1).pts().len(), 2);
    }
}
