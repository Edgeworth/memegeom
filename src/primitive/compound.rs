use std::cell::{Ref, RefCell};

use crate::Result;
use crate::geom::qt::quadtree::{QuadTree, ShapeIdx};
use crate::geom::qt::query::{ALL, Query, ShapeInfo};
use crate::primitive::shape::Shape;
use crate::primitive::{Rt, ShapeOps};

// Represents a collection of shapes.
// Backed by a quadtree-like spatial data structure.
#[must_use]
#[derive(Debug, Clone)]
pub struct Compound {
    qt: RefCell<QuadTree>,
}

impl Default for Compound {
    fn default() -> Self {
        Self::with_bounds(&Rt::default())
    }
}

impl Compound {
    pub fn with_bounds(r: &Rt) -> Self {
        Self { qt: RefCell::new(QuadTree::with_bounds(r)) }
    }

    pub fn add_shape(&self, shape: ShapeInfo) -> Result<Vec<ShapeIdx>> {
        self.qt.borrow_mut().add_shape(shape)
    }

    pub fn remove_shape(&mut self, s: ShapeIdx) {
        self.qt.borrow_mut().remove_shape(s);
    }

    pub fn intersects(&self, s: &Shape, q: Query) -> bool {
        self.qt.borrow_mut().intersects(s, q)
    }

    // N.B. this will check if any one shape in the compound contains |s|.
    // If |s| is covered using multiple shapes then that won't be detected.
    pub fn contains(&self, s: &Shape, q: Query) -> bool {
        self.qt.borrow_mut().contains(s, q)
    }

    pub fn dist(&self, s: &Shape, q: Query) -> Option<f64> {
        self.qt.borrow_mut().dist(s, q)
    }

    pub fn quadtree(&self) -> Ref<'_, QuadTree> {
        self.qt.borrow()
    }
}

impl ShapeOps for Compound {
    fn bounds(&self) -> Option<Rt> {
        self.qt.borrow().bounds()
    }

    fn shape(self) -> Shape {
        Shape::Compound(Box::new(self))
    }

    fn is_empty_set(&self) -> bool {
        // Compound is empty if all contained shapes are empty (including if there are no shapes)
        self.qt.borrow().shapes().all(|s| s.shape().is_empty_set())
    }

    fn intersects_shape(&self, s: &Shape) -> bool {
        self.qt.borrow_mut().intersects(s, ALL)
    }

    // N.B. this will check if any one shape in the compound contains |s|.
    // If |s| is covered using multiple shapes then that won't be detected.
    fn contains_shape(&self, s: &Shape) -> bool {
        self.qt.borrow_mut().contains(s, ALL)
    }

    fn dist_to_shape(&self, s: &Shape) -> Option<f64> {
        self.qt.borrow_mut().dist(s, ALL)
    }
}
