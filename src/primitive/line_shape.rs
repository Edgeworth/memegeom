use approx::{AbsDiffEq, RelativeEq};

use crate::geom::distance::line_pt_dist;
use crate::geom::intersects::{line_intersects_line, line_intersects_seg};
use crate::primitive::point::Pt;
use crate::primitive::shape::Shape;
use crate::primitive::{Rt, ShapeOps};

#[must_use]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct LinePrimitive {
    st: Pt,
    en: Pt,
}

impl AbsDiffEq for LinePrimitive {
    type Epsilon = f64;

    fn default_epsilon() -> f64 {
        f64::default_epsilon()
    }

    fn abs_diff_eq(&self, o: &Self, epsilon: f64) -> bool {
        Pt::abs_diff_eq(&self.st, &o.st, epsilon) && Pt::abs_diff_eq(&self.en, &o.en, epsilon)
    }
}

impl RelativeEq for LinePrimitive {
    fn default_max_relative() -> f64 {
        f64::default_max_relative()
    }

    fn relative_eq(&self, o: &Self, epsilon: f64, max_relative: f64) -> bool {
        Pt::relative_eq(&self.st, &o.st, epsilon, max_relative)
            && Pt::relative_eq(&self.en, &o.en, epsilon, max_relative)
    }
}

impl LinePrimitive {
    pub const fn new(st: Pt, en: Pt) -> Self {
        assert!(
            st.x.is_finite() && st.y.is_finite() && en.x.is_finite() && en.y.is_finite(),
            "line endpoint coordinates must be finite"
        );
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

    // Projects |p| onto this line.
    pub fn project(&self, p: Pt) -> Pt {
        let dir = self.dir();
        let mag2 = dir.mag2();
        // For a degenerate line (st == en), return the single point.
        if mag2 == 0.0 {
            return self.st;
        }
        let k = dir.dot(p - self.st) / mag2;
        self.st + k * dir
    }
}

impl ShapeOps for LinePrimitive {
    fn bounds(&self) -> Option<Rt> {
        None
    }

    fn shape(self) -> Shape {
        Shape::Line(self)
    }

    fn is_empty_set(&self) -> bool {
        false // Lines are infinite and never empty
    }

    fn intersects_shape(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(_) | Shape::CapsuleExcl(_) => todo!(),
            Shape::Circle(_) | Shape::CircleExcl(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(s) => line_intersects_line(self, s),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(_) => todo!(),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(_) => todo!(),
            Shape::RectExcl(_) => todo!(),
            Shape::Segment(s) => line_intersects_seg(self, s),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }

    fn contains_shape(&self, s: &Shape) -> bool {
        match s {
            Shape::Capsule(_) | Shape::CapsuleExcl(_) => todo!(),
            Shape::Circle(_) | Shape::CircleExcl(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(_) => todo!(),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(_) => todo!(),
            Shape::RectExcl(_) => todo!(),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }

    fn dist_to_shape(&self, s: &Shape) -> Option<f64> {
        match s {
            Shape::Capsule(_) | Shape::CapsuleExcl(_) => todo!(),
            Shape::Circle(_) | Shape::CircleExcl(_) => todo!(),
            Shape::Compound(_) => todo!(),
            Shape::Line(_) => todo!(),
            Shape::Path(_) | Shape::PathExcl(_) => todo!(),
            Shape::Point(s) => Some(line_pt_dist(self, s)),
            Shape::Poly(_) | Shape::PolyExcl(_) => todo!(),
            Shape::Rect(_) => todo!(),
            Shape::RectExcl(_) => todo!(),
            Shape::Segment(_) => todo!(),
            Shape::Tri(_) | Shape::TriExcl(_) => todo!(),
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::primitive::{line, pt};

    #[test]
    fn project() {
        assert_relative_eq!(line(pt(1.0, 1.0), pt(3.0, 5.0)).project(pt(3.0, 3.0)), pt(2.2, 3.4));
    }
}
