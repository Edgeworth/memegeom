use crate::geom::math::{is_collinear, is_left_of, is_strictly_left_of};
use crate::primitive::line;
use crate::primitive::point::Pt;

#[must_use]
pub fn remove_collinear(pts: &[Pt], wrap_around: bool) -> Vec<Pt> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    let mut out = vec![pts[0], pts[1]];
    for &p in pts.iter().skip(2) {
        let l = out.len();
        if is_collinear(out[l - 2], out[l - 1], p) {
            out.pop();
        }
        out.push(p);
    }
    if wrap_around && out.len() >= 3 {
        // Track how many elements to skip from the front instead of removing them (O(n)).
        let mut start = 0;
        // Check wrap-around: last, first, second
        while out.len() - start >= 3 && is_collinear(out[out.len() - 1], out[start], out[start + 1])
        {
            start += 1;
        }
        // Check wrap-around: second-to-last, last, first
        while out.len() - start >= 3
            && is_collinear(out[out.len() - 2], out[out.len() - 1], out[start])
        {
            out.pop();
        }
        // Build final result if we skipped any front elements.
        if start > 0 {
            out = out[start..].to_vec();
        }
    }
    out
}

pub fn ensure_ccw(pts: &mut [Pt]) {
    if pts.len() > 2 && !is_left_of(&line(pts[0], pts[1]), pts[2]) {
        pts.reverse();
    }
}

// Tests if a CCW polygon |pts| is convex.
#[must_use]
pub fn is_convex_ccw(pts: &[Pt]) -> bool {
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        let c = pts[(i + 2) % pts.len()];
        if !is_strictly_left_of(&line(a, b), c) {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitive::pt;

    #[test]
    fn remove_collinear_no_wrap() {
        let pts = vec![pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0)];
        let result = remove_collinear(&pts, false);
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn remove_collinear_no_wrap_preserves_endpoints() {
        let pts = vec![pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0), pt(3.0, 1.0)];
        let result = remove_collinear(&pts, false);
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn remove_collinear_with_wrap() {
        let pts = vec![pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0), pt(1.0, 1.0)];
        let result = remove_collinear(&pts, true);
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn remove_collinear_wrap_all_collinear() {
        let pts = vec![pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0), pt(3.0, 0.0)];
        let result = remove_collinear(&pts, true);
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn remove_collinear_wrap_removes_from_end() {
        // Square with collinear point on left edge
        let pts = vec![pt(0.0, 0.0), pt(1.0, 0.0), pt(1.0, 1.0), pt(0.0, 1.0), pt(0.0, 0.5)];
        let result = remove_collinear(&pts, true);
        assert_eq!(result.len(), 4);
    }

    #[test]
    fn remove_collinear_wrap_removes_from_front() {
        // E,A,B collinear on y=0, removes A
        let pts = vec![pt(1.0, 0.0), pt(2.0, 0.0), pt(2.0, 1.0), pt(0.0, 1.0), pt(0.0, 0.0)];
        let result = remove_collinear(&pts, true);
        assert_eq!(result.len(), 4);
    }

    #[test]
    fn remove_collinear_wrap_removes_from_both() {
        // A,B,D,E on y=0, C off. After wrap: removes A and E
        let pts = vec![pt(0.0, 0.0), pt(1.0, 0.0), pt(1.0, 1.0), pt(2.0, 0.0), pt(3.0, 0.0)];
        let result = remove_collinear(&pts, true);
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn remove_collinear_empty() {
        let result = remove_collinear(&[], true);
        assert!(result.is_empty());
    }

    #[test]
    fn remove_collinear_single_point() {
        let result = remove_collinear(&[pt(1.0, 2.0)], true);
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn remove_collinear_two_points() {
        let result = remove_collinear(&[pt(0.0, 0.0), pt(1.0, 1.0)], true);
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn is_convex_ccw_triangle() {
        assert!(is_convex_ccw(&[pt(0.0, 0.0), pt(1.0, 0.0), pt(0.5, 1.0)]));
    }

    #[test]
    fn is_convex_ccw_square() {
        assert!(is_convex_ccw(&[pt(0.0, 0.0), pt(1.0, 0.0), pt(1.0, 1.0), pt(0.0, 1.0)]));
    }

    #[test]
    fn is_convex_ccw_concave() {
        let pts = vec![
            pt(0.0, 0.0),
            pt(2.0, 0.0),
            pt(2.0, 1.0),
            pt(1.0, 1.0),
            pt(1.0, 2.0),
            pt(0.0, 2.0),
        ];
        assert!(!is_convex_ccw(&pts));
    }
}
