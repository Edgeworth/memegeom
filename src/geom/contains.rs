use crate::geom::distance::{polyline_pt_dist, pt_seg_dist};
use crate::geom::math::{ge, is_left_of, is_right_of, le, lt, ne, orientation};
use crate::primitive::capsule::CapsulePrimitive;
use crate::primitive::circle::CirclePrimitive;
use crate::primitive::path_shape::PathPrimitive;
use crate::primitive::point::Pt;
use crate::primitive::polygon::PolyPrimitive;
use crate::primitive::rect::RtPrimitive;
use crate::primitive::segment::SegmentPrimitive;
use crate::primitive::triangle::TriPrimitive;
use crate::primitive::{Boundary, Rt, ShapeOps, line};

// For containment checks: returns true if container's bounds don't contain the point.
fn bounds_exclude_pt(container: Option<Rt>, pt: Pt) -> bool {
    container.is_some_and(|b| !b.contains(pt))
}

// For containment checks: returns true if container's bounds don't contain contained's bounds.
fn bounds_exclude_rt<const B: Boundary>(container: Option<Rt>, contained: &RtPrimitive<B>) -> bool {
    container.is_some_and(|b| !b.contains_rt(contained))
}

// For containment checks: returns true if container's bounds don't contain contained's bounds.
fn bounds_exclude_bounds(container: Option<Rt>, contained: Option<Rt>) -> bool {
    match (container, contained) {
        (Some(a), Some(b)) => !a.contains_rt(&b),
        _ => false,
    }
}

#[must_use]
pub fn cap_contains_pt<const B: Boundary>(a: &CapsulePrimitive<B>, b: &Pt) -> bool {
    if a.is_empty_set() {
        return false;
    }
    if bounds_exclude_pt(a.bounds(), *b) {
        return false;
    }
    let dist = pt_seg_dist(b, &a.seg());
    match B {
        Boundary::Exclude => lt(dist, a.r()),
        Boundary::Include => le(dist, a.r()),
    }
}

#[must_use]
pub fn cap_contains_rt<const B: Boundary, const B2: Boundary>(
    a: &CapsulePrimitive<B>,
    b: &RtPrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    if bounds_exclude_rt(a.bounds(), b) {
        return false;
    }
    for p in b.pts() {
        if !cap_contains_pt(a, &p) {
            return false;
        }
    }
    true
}

#[must_use]
pub fn circ_contains_rt<const B: Boundary, const B2: Boundary>(
    a: &CirclePrimitive<B>,
    b: &RtPrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    // Sufficient to check all rectangle points are within the circle.
    circ_contains_pt(a, &b.bl())
        && circ_contains_pt(a, &b.br())
        && circ_contains_pt(a, &b.tr())
        && circ_contains_pt(a, &b.tl())
}

#[must_use]
pub fn circ_contains_pt<const B: Boundary>(a: &CirclePrimitive<B>, b: &Pt) -> bool {
    if a.is_empty_set() {
        return false;
    }
    let dist = a.p().dist(*b);
    match B {
        Boundary::Exclude => lt(dist, a.r()),
        Boundary::Include => le(dist, a.r()),
    }
}

#[must_use]
pub fn path_contains_rt<const B: Boundary, const B2: Boundary>(
    a: &PathPrimitive<B>,
    b: &RtPrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    if bounds_exclude_rt(a.bounds(), b) {
        return false;
    }
    // This function is too complicated to have an exact solution.
    // An approach is to split |a| into quads and circles, then compute the
    // intersection of the quads and |b|. Then, do voronoi with the circles
    // and ensure the non-intersected parts of |b| are covered.
    // This function is only used in the quadtree and it doesn't have to
    // be exact so instead just check each capsule. It will miss cases
    // where the rectangle goes over multiple capsules.
    for cap in a.caps() {
        if cap_contains_rt(&cap, b) {
            return true;
        }
    }
    false
}

#[must_use]
pub fn path_contains_seg<const B: Boundary>(_a: &PathPrimitive<B>, _b: &SegmentPrimitive) -> bool {
    todo!()
}

#[must_use]
pub fn poly_contains_cap<const B: Boundary, const B2: Boundary>(
    a: &PolyPrimitive<B>,
    b: &CapsulePrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    if bounds_exclude_bounds(a.bounds(), b.bounds()) {
        return false;
    }
    // For degenerate capsule (st == en), it's just a circle.
    if b.st() == b.en() {
        return poly_contains_circ(a, &b.st_cap());
    }

    // First check both end caps are in the polygon.
    if !poly_contains_circ(a, &b.st_cap()) {
        return false;
    }
    if !poly_contains_circ(a, &b.en_cap()) {
        return false;
    }
    // Check left and right walls of the segment are in the polygon.
    // Safe to unwrap: degenerate case (st == en) already handled above.
    if !poly_contains_seg(a, &b.left_seg().unwrap()) {
        return false;
    }
    if !poly_contains_seg(a, &b.right_seg().unwrap()) {
        return false;
    }
    true
}

#[must_use]
pub fn poly_contains_circ<const B: Boundary, const B2: Boundary>(
    a: &PolyPrimitive<B>,
    b: &CirclePrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    // Test that the centre of the circle is in the polygon.
    if !poly_contains_pt(a, &b.p()) {
        return false;
    }
    // Polygon is non-empty if we got here (passed poly_contains_pt check)
    ge(polyline_pt_dist(a.pts(), &b.p()).unwrap_or(0.0), b.r())
}

#[must_use]
pub fn poly_contains_path<const B: Boundary, const B2: Boundary>(
    a: &PolyPrimitive<B>,
    b: &PathPrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    if bounds_exclude_bounds(a.bounds(), b.bounds()) {
        return false;
    }
    for cap in b.caps() {
        if !poly_contains_cap(a, &cap) {
            return false;
        }
    }
    true
}

#[must_use]
pub fn poly_contains_pt<const B: Boundary>(a: &PolyPrimitive<B>, b: &Pt) -> bool {
    if a.is_empty_set() {
        return false;
    }
    if bounds_exclude_pt(a.bounds(), *b) {
        return false;
    }
    // Winding number test. Look at horizontal line at b.y and count crossings
    // of edges from |a|.
    let mut winding = 0;
    for [&p0, &p1] in a.edges() {
        // Treat points at b.y as slightly above it.
        if ge(p0.y, b.y) {
            // Downward crossing edge with |b| to the right of it decreases
            // winding number.
            if lt(p1.y, b.y) && is_right_of(&line(p0, p1), *b) {
                winding -= 1;
            }
        } else if ge(p1.y, b.y) && is_left_of(&line(p0, p1), *b) {
            // Upward crossing edge with |b| to the left of it increases
            // winding number.
            winding += 1;
        }
    }
    if winding == 0 {
        return false;
    }
    match B {
        // Polygon is non-empty if winding != 0 (we got here after that check)
        Boundary::Exclude => ne(polyline_pt_dist(a.pts(), b).unwrap_or(0.0), 0.0),
        Boundary::Include => true,
    }
}

#[must_use]
pub fn poly_contains_rt<const B: Boundary, const B2: Boundary>(
    a: &PolyPrimitive<B>,
    b: &RtPrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    if bounds_exclude_rt(a.bounds(), b) {
        return false;
    }
    // Check point containment of |b| in |a|.
    for p in b.pts() {
        if !poly_contains_pt(a, &p) {
            return false;
        }
    }
    // Check segment containment of |b| in |a| if |a| is non-convex.
    if !a.is_convex() {
        for seg in b.segs() {
            if !poly_contains_seg(a, &seg) {
                return false;
            }
        }
    }
    true
}

#[must_use]
pub fn poly_contains_seg<const B: Boundary>(a: &PolyPrimitive<B>, b: &SegmentPrimitive) -> bool {
    if a.is_empty_set() {
        return false;
    }
    if bounds_exclude_bounds(a.bounds(), b.bounds()) {
        return false;
    }
    // Check that both endpoints of |b| are in a.
    if !poly_contains_pt(a, &b.st()) || !poly_contains_pt(a, &b.en()) {
        return false;
    }

    // If |a| is convex only need to check endpoint containment.
    if a.is_convex() {
        return true;
    }

    // Check that |b| does not cross any edge of |a|.
    for [&p0, &p1] in a.edges() {
        let p_st = orientation(&b.line(), p0);
        let p_en = orientation(&b.line(), p1);
        let b_st = orientation(&line(p0, p1), b.st());
        let b_en = orientation(&line(p0, p1), b.en());
        // Segments are crossing and no collinear points.
        if p_st != p_en && b_st != b_en {
            return false;
        }
    }
    true
}

#[must_use]
pub fn rt_contains_cap<const B: Boundary, const B2: Boundary>(
    a: &RtPrimitive<B>,
    b: &CapsulePrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    if b.bounds().is_some_and(|bb| !a.contains_rt(&bb)) {
        return false;
    }
    // For degenerate capsule (st == en), it's just a circle.
    if b.st() == b.en() {
        return rt_contains_circ(a, &b.st_cap());
    }

    // First check both end caps are in the rect.
    if !rt_contains_circ(a, &b.st_cap()) {
        return false;
    }
    if !rt_contains_circ(a, &b.en_cap()) {
        return false;
    }
    // Check left and right walls of the segment are in the rect.
    // Safe to unwrap: degenerate case (st == en) already handled above.
    if !rt_contains_seg(a, &b.left_seg().unwrap()) {
        return false;
    }
    if !rt_contains_seg(a, &b.right_seg().unwrap()) {
        return false;
    }
    true
}

#[must_use]
pub fn rt_contains_circ<const B: Boundary, const B2: Boundary>(
    a: &RtPrimitive<B>,
    b: &CirclePrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    // Check the centre is in the rectangle:
    if !a.contains(b.p()) {
        return false;
    }
    // Check the shortest distance to the wall is less than or equal to the
    // radius.
    let x_dist = (b.p().x - a.l()).min(a.r() - b.p().x);
    if lt(x_dist, b.r()) {
        return false;
    }
    let y_dist = (b.p().y - a.b()).min(a.t() - b.p().y);
    if lt(y_dist, b.r()) {
        return false;
    }
    true
}

#[must_use]
pub fn rt_contains_path<const B: Boundary, const B2: Boundary>(
    a: &RtPrimitive<B>,
    b: &PathPrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    if b.bounds().is_some_and(|bb| !a.contains_rt(&bb)) {
        return false;
    }
    // Just check all points in |b| are in |a|.
    for cap in b.caps() {
        if !rt_contains_cap(a, &cap) {
            return false;
        }
    }
    true
}

#[must_use]
pub fn rt_contains_poly<const B: Boundary, const B2: Boundary>(
    a: &RtPrimitive<B>,
    b: &PolyPrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    if b.bounds().is_some_and(|bb| !a.contains_rt(&bb)) {
        return false;
    }
    // Just check all points in |b| are in |a|.
    for p in b.pts() {
        if !a.contains(*p) {
            return false;
        }
    }
    true
}

#[must_use]
pub fn rt_contains_seg<const B: Boundary>(a: &RtPrimitive<B>, b: &SegmentPrimitive) -> bool {
    if a.is_empty_set() {
        return false;
    }
    // Just need to check containment of both endpoints.
    a.contains(b.st()) && a.contains(b.en())
}

#[must_use]
pub fn rt_contains_tri<const B: Boundary, const B2: Boundary>(
    a: &RtPrimitive<B>,
    b: &TriPrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    // Just check all points in |b| are in |a|.
    for p in b.pts() {
        if !a.contains(*p) {
            return false;
        }
    }
    true
}

#[must_use]
pub fn tri_contains_pt<const B: Boundary>(a: &TriPrimitive<B>, b: &Pt) -> bool {
    if a.is_empty_set() {
        return false;
    }
    let o0 = orientation(&line(a[0], a[1]), *b);
    let o1 = orientation(&line(a[1], a[2]), *b);
    let o2 = orientation(&line(a[2], a[0]), *b);
    let min = o0.min(o1).min(o2);
    let max = o0.max(o1).max(o2);
    match B {
        Boundary::Exclude => min > 0 || max < 0,
        Boundary::Include => min >= 0 || max <= 0,
    }
}

#[must_use]
pub fn tri_contains_rt<const B: Boundary, const B2: Boundary>(
    a: &TriPrimitive<B>,
    b: &RtPrimitive<B2>,
) -> bool {
    if b.is_empty_set() {
        return true;
    }
    if a.is_empty_set() {
        return false;
    }
    for p in b.pts() {
        if !tri_contains_pt(a, &p) {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitive::{cap, poly, pt, rt, tri, tri_excl};

    #[test]
    fn tri_contains_pt_interior() {
        let t = tri(pt(0.0, 0.0), pt(2.0, 0.0), pt(1.0, 2.0));
        assert!(tri_contains_pt(&t, &pt(1.0, 0.5)));
        assert!(tri_contains_pt(&t, &pt(1.0, 1.0)));
    }

    #[test]
    fn tri_contains_pt_boundary() {
        let t = tri(pt(0.0, 0.0), pt(2.0, 0.0), pt(1.0, 2.0));
        // On edges
        assert!(tri_contains_pt(&t, &pt(1.0, 0.0)));
        assert!(tri_contains_pt(&t, &pt(0.5, 1.0)));
        assert!(tri_contains_pt(&t, &pt(1.5, 1.0)));
        // At vertices
        assert!(tri_contains_pt(&t, &pt(0.0, 0.0)));
        assert!(tri_contains_pt(&t, &pt(2.0, 0.0)));
        assert!(tri_contains_pt(&t, &pt(1.0, 2.0)));
    }

    #[test]
    fn tri_contains_pt_outside() {
        let t = tri(pt(0.0, 0.0), pt(2.0, 0.0), pt(1.0, 2.0));
        assert!(!tri_contains_pt(&t, &pt(1.0, -1.0))); // below
        assert!(!tri_contains_pt(&t, &pt(-1.0, 1.0))); // left
        assert!(!tri_contains_pt(&t, &pt(3.0, 1.0))); // right
        assert!(!tri_contains_pt(&t, &pt(1.0, 3.0))); // above
    }

    #[test]
    fn tri_contains_pt_excluded_boundary() {
        let t = tri_excl(pt(0.0, 0.0), pt(2.0, 0.0), pt(1.0, 2.0));
        // Interior should be contained
        assert!(tri_contains_pt(&t, &pt(1.0, 0.5)));
        // Boundary should NOT be contained
        assert!(!tri_contains_pt(&t, &pt(1.0, 0.0)));
        assert!(!tri_contains_pt(&t, &pt(0.0, 0.0)));
    }

    #[test]
    fn poly_contains_degenerate_capsule_as_circle() {
        let square = poly(&[pt(-10.0, -10.0), pt(10.0, -10.0), pt(10.0, 10.0), pt(-10.0, 10.0)]);
        let degenerate = cap(pt(0.0, 0.0), pt(0.0, 0.0), 1.0);
        assert!(poly_contains_cap(&square, &degenerate));
    }

    #[test]
    fn rt_contains_degenerate_capsule_as_circle() {
        let bounds = rt(-10.0, -10.0, 10.0, 10.0);
        let degenerate = cap(pt(0.0, 0.0), pt(0.0, 0.0), 1.0);
        assert!(rt_contains_cap(&bounds, &degenerate));
    }
}
