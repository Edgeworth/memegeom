use ahash::HashMap;
use ordered_float::OrderedFloat;
use smallvec::{SmallVec, smallvec};

use crate::geom::distance::rt_rt_dist;
use crate::geom::qt::query::{
    Query, ShapeInfo, cached_contains, cached_dist, cached_intersects, decompose_shape,
    matches_query,
};
use crate::primitive::shape::Shape;
use crate::primitive::{Rt, ShapeOps};
use crate::{Error, Result};

type NodeIdx = usize;
pub type ShapeIdx = usize;

// How many tests to do before splitting a node.
const TEST_THRESHOLD: usize = 4;
const MAX_DEPTH: usize = 7;
const NO_NODE: NodeIdx = 0;

// Returns the minimum of two optional distances. None represents "no distance found".
fn min_opt(a: Option<f64>, b: Option<f64>) -> Option<f64> {
    match (a, b) {
        (None, x) | (x, None) => x,
        (Some(a), Some(b)) => Some(a.min(b)),
    }
}

#[must_use]
#[derive(Debug, Copy, Clone)]
struct IntersectData {
    shape_idx: ShapeIdx,
    tests: usize, // How many times we had to test against shapes directly.
}

#[must_use]
#[derive(Debug, Clone)]
struct Node {
    intersect: Vec<IntersectData>, // Which shapes intersect this node.
    contain: Vec<ShapeIdx>,        // Which shapes contain this node.
    bl: NodeIdx,
    br: NodeIdx,
    tr: NodeIdx,
    tl: NodeIdx,
}

impl Default for Node {
    fn default() -> Self {
        Self {
            intersect: Vec::new(),
            contain: Vec::new(),
            bl: NO_NODE,
            br: NO_NODE,
            tr: NO_NODE,
            tl: NO_NODE,
        }
    }
}

#[must_use]
#[derive(Debug, Clone)]
pub struct QuadTree {
    shapes: Vec<Option<ShapeInfo>>,
    free_shapes: Vec<ShapeIdx>, // List of indices of shapes that have been deleted.
    nodes: Vec<Node>,
    bounds: Option<Rt>,
    intersect_cache: HashMap<ShapeIdx, bool>, // Caches intersection tests.
    contain_cache: HashMap<ShapeIdx, bool>,   // Caches containment tests.
    dist_cache: HashMap<ShapeIdx, Option<f64>>, // Caches distance tests.
}

impl QuadTree {
    fn ensure_has_bounds(shape: &ShapeInfo) -> Result<Rt> {
        shape.shape().bounds().ok_or(Error::NoBounds)
    }

    fn rebuild_nodes(&mut self) {
        self.nodes = vec![
            Node::default(),
            Node {
                intersect: self
                    .shapes
                    .iter()
                    .enumerate()
                    .filter_map(|(shape_idx, s)| {
                        s.as_ref().map(|_| IntersectData { shape_idx, tests: 0 })
                    })
                    .collect(),
                ..Default::default()
            },
        ];
        self.free_shapes = self
            .shapes
            .iter()
            .enumerate()
            .filter_map(|(idx, s)| s.is_none().then_some(idx))
            .collect();
        self.reset_cache();
    }

    pub fn new(shapes: Vec<ShapeInfo>) -> Result<Self> {
        let mut bounds: Option<Rt> = None;
        for shape in &shapes {
            let b = Self::ensure_has_bounds(shape)?;
            bounds = Some(bounds.map_or(b, |a| a.united(&b)));
        }

        let shapes: Vec<_> = shapes.into_iter().map(Some).collect();
        let nodes = vec![
            Node::default(),
            Node {
                intersect: (0..shapes.len())
                    .map(|shape_idx| IntersectData { shape_idx, tests: 0 })
                    .collect(),
                ..Default::default()
            },
        ];

        Ok(Self {
            shapes,
            nodes,
            bounds,
            free_shapes: Vec::new(),
            intersect_cache: HashMap::default(),
            contain_cache: HashMap::default(),
            dist_cache: HashMap::default(),
        })
    }

    pub fn with_bounds(r: &Rt) -> Self {
        Self {
            shapes: Vec::new(),
            nodes: vec![Node::default(), Node::default()],
            bounds: Some(*r),
            free_shapes: Vec::new(),
            intersect_cache: HashMap::default(),
            contain_cache: HashMap::default(),
            dist_cache: HashMap::default(),
        }
    }

    // Gets the current rectangles of the quad tree.
    #[must_use]
    pub fn rts(&self) -> Vec<Rt> {
        let mut rts = Vec::new();
        if let Some(bounds) = self.bounds() {
            self.rts_internal(1, bounds, &mut rts);
        }
        rts
    }

    pub fn shapes(&self) -> impl Iterator<Item = &ShapeInfo> {
        self.shapes.iter().filter_map(Option::as_ref)
    }

    fn rts_internal(&self, idx: NodeIdx, r: Rt, rts: &mut Vec<Rt>) {
        if idx == NO_NODE {
            return;
        }
        rts.push(r);
        self.rts_internal(self.nodes[idx].bl, r.bl_quadrant(), rts);
        self.rts_internal(self.nodes[idx].br, r.br_quadrant(), rts);
        self.rts_internal(self.nodes[idx].tr, r.tr_quadrant(), rts);
        self.rts_internal(self.nodes[idx].tl, r.tl_quadrant(), rts);
    }

    pub fn add_shape(&mut self, s: ShapeInfo) -> Result<Vec<ShapeIdx>> {
        // If this shape expands the bounds, rebuild the tree.
        // TODO: Don't rebuild the tree?
        let shapes = decompose_shape(s);

        let mut new_bounds = self.bounds;
        for shape in &shapes {
            let b = Self::ensure_has_bounds(shape)?;
            new_bounds = Some(new_bounds.map_or(b, |a| a.united(&b)));
        }

        let bounds_changed = new_bounds != self.bounds;
        let mut shape_idxs = Vec::with_capacity(shapes.len());

        for shape in shapes {
            let shape_idx = if let Some(shape_idx) = self.free_shapes.pop() {
                self.shapes[shape_idx] = Some(shape);
                shape_idx
            } else {
                self.shapes.push(Some(shape));
                self.shapes.len() - 1
            };
            shape_idxs.push(shape_idx);
            if !bounds_changed {
                self.nodes[1].intersect.push(IntersectData { shape_idx, tests: 0 });
            }
        }

        if bounds_changed {
            self.bounds = new_bounds;
            self.rebuild_nodes();
        }

        Ok(shape_idxs)
    }

    pub fn remove_shape(&mut self, s: ShapeIdx) {
        let Some(slot) = self.shapes.get_mut(s) else {
            return;
        };
        if slot.is_none() {
            return;
        }
        *slot = None;

        // Remove everything referencing this shape.
        for node in &mut self.nodes {
            node.intersect.retain(|v| v.shape_idx != s);
            node.contain.retain(|&v| v != s);
        }
        self.free_shapes.push(s);
    }

    #[must_use]
    pub fn bounds(&self) -> Option<Rt> {
        self.bounds
    }

    fn reset_cache(&mut self) {
        self.intersect_cache.clear();
        self.contain_cache.clear();
        self.dist_cache.clear();
    }

    pub fn intersects(&mut self, s: &Shape, q: Query) -> bool {
        self.reset_cache();
        match self.bounds() {
            Some(bounds) => self.inter(s, q, 1, bounds, 0),
            None => false,
        }
    }

    pub fn contains(&mut self, s: &Shape, q: Query) -> bool {
        self.reset_cache();
        match self.bounds() {
            Some(bounds) => self.contain(s, q, 1, bounds, 0),
            None => false,
        }
    }

    pub fn dist(&mut self, s: &Shape, q: Query) -> Option<f64> {
        self.reset_cache();
        match self.bounds() {
            Some(bounds) => self.distance(s, q, 1, bounds, None, 0),
            None => None,
        }
    }

    fn inter(&mut self, s: &Shape, q: Query, idx: NodeIdx, r: Rt, depth: usize) -> bool {
        // No intersection in this node if we don't intersect the bounds.
        if !s.intersects_shape(&r.shape()) {
            return false;
        }

        // If there are any shapes containing this node they must intersect with
        // |s| since it intersects |bounds|.
        for &contain in &self.nodes[idx].contain {
            let shape_info = self.shapes[contain].as_ref().unwrap();
            if matches_query(shape_info, q) {
                return true;
            }
        }

        // TODO: Could check if |s| contains the bounds here and return true if
        // intersect is non-empty.

        // Check children, if they exist. Do this first as we expect traversing
        // the tree to be faster. Only actually do intersection tests if we have
        // to.
        if self.nodes[idx].bl != NO_NODE
            && self.inter(s, q, self.nodes[idx].bl, r.bl_quadrant(), depth + 1)
        {
            return true;
        }
        if self.nodes[idx].br != NO_NODE
            && self.inter(s, q, self.nodes[idx].br, r.br_quadrant(), depth + 1)
        {
            return true;
        }
        if self.nodes[idx].tr != NO_NODE
            && self.inter(s, q, self.nodes[idx].tr, r.tr_quadrant(), depth + 1)
        {
            return true;
        }
        if self.nodes[idx].tl != NO_NODE
            && self.inter(s, q, self.nodes[idx].tl, r.tl_quadrant(), depth + 1)
        {
            return true;
        }

        // Check shapes that intersect this node:
        let mut had_intersection = false;
        for inter in &mut self.nodes[idx].intersect {
            inter.tests += 1;
            let shape_info = self.shapes[inter.shape_idx].as_ref().unwrap();
            if cached_intersects(&mut self.intersect_cache, inter.shape_idx, shape_info, s, q) {
                had_intersection = true;
                break;
            }
        }
        self.maybe_push_down(idx, r, depth);

        had_intersection
    }

    fn contain(&mut self, s: &Shape, q: Query, idx: NodeIdx, r: Rt, depth: usize) -> bool {
        // No containment of |s| if the bounds don't intersect |s|.
        if !r.intersects_shape(s) {
            return false;
        }

        // If bounds contains |s| and there is something that contains the
        // bounds, then that contains |s|.
        if r.contains_shape(s) {
            for &contain in &self.nodes[idx].contain {
                let shape_info = self.shapes[contain].as_ref().unwrap();
                if matches_query(shape_info, q) {
                    return true;
                }
            }
        }

        // Check children, if they exist. Do this first as we expect traversing
        // the tree to be faster. Only actually do intersection tests if we have
        // to.
        if self.nodes[idx].bl != NO_NODE
            && self.contain(s, q, self.nodes[idx].bl, r.bl_quadrant(), depth + 1)
        {
            return true;
        }
        if self.nodes[idx].br != NO_NODE
            && self.contain(s, q, self.nodes[idx].br, r.br_quadrant(), depth + 1)
        {
            return true;
        }
        if self.nodes[idx].tr != NO_NODE
            && self.contain(s, q, self.nodes[idx].tr, r.tr_quadrant(), depth + 1)
        {
            return true;
        }
        if self.nodes[idx].tl != NO_NODE
            && self.contain(s, q, self.nodes[idx].tl, r.tl_quadrant(), depth + 1)
        {
            return true;
        }

        // Check shapes that intersect this node:
        let mut had_containment = false;
        for inter in &mut self.nodes[idx].intersect {
            inter.tests += 1;
            let shape_info = self.shapes[inter.shape_idx].as_ref().unwrap();
            if cached_contains(&mut self.contain_cache, inter.shape_idx, shape_info, s, q) {
                had_containment = true;
                break;
            }
        }
        self.maybe_push_down(idx, r, depth);

        had_containment
    }

    fn distance(
        &mut self,
        s: &Shape,
        q: Query,
        idx: NodeIdx,
        r: Rt,
        mut best: Option<f64>,
        depth: usize,
    ) -> Option<f64> {
        // Early pruning: if the distance to this node's bounds is already >= best,
        // skip this node entirely as it can't improve the result.
        let b = s.bounds();
        if let (Some(bb), Some(current_best)) = (b, best) {
            // Both r and bb are valid bounding boxes, so rt_rt_dist always returns Some
            let lower_bound = rt_rt_dist(&r, &bb).unwrap_or(0.0);
            if lower_bound >= current_best {
                return best;
            }
        }

        // If bounds intersects |s| and there is something that contains the
        // bounds, then the distance is zero (intersecting a shape).
        if b.is_some_and(|bb| r.contains_rt(&bb)) {
            for &contain in &self.nodes[idx].contain {
                let shape_info = self.shapes[contain].as_ref().unwrap();
                if matches_query(shape_info, q) {
                    return Some(0.0);
                }
            }
        }

        // Traverse children in order of shortest AABB distance. This optimises the
        // good case where a small object goes directly to objects near it.
        let mut children: SmallVec<[(f64, usize, Rt); 4]> = smallvec![];
        let child_dist =
            |child_rt: &Rt| b.map_or(0.0, |bb| rt_rt_dist(child_rt, &bb).unwrap_or(0.0));
        if self.nodes[idx].bl != NO_NODE {
            let child_rt = r.bl_quadrant();
            children.push((child_dist(&child_rt), self.nodes[idx].bl, child_rt));
        }
        if self.nodes[idx].br != NO_NODE {
            let child_rt = r.br_quadrant();
            children.push((child_dist(&child_rt), self.nodes[idx].br, child_rt));
        }
        if self.nodes[idx].tr != NO_NODE {
            let child_rt = r.tr_quadrant();
            children.push((child_dist(&child_rt), self.nodes[idx].tr, child_rt));
        }
        if self.nodes[idx].tl != NO_NODE {
            let child_rt = r.tl_quadrant();
            children.push((child_dist(&child_rt), self.nodes[idx].tl, child_rt));
        }
        children.sort_unstable_by_key(|v| OrderedFloat(v.0));

        // If we can't do better than the current best in this node, give up.
        for (lower_bound, child_idx, child_rt) in children {
            // Distance must be greater than lower bound, and this is sorted by
            // lower bound dist, so early exit.
            if best.is_some_and(|b| b < lower_bound) {
                break;
            }
            best = min_opt(best, self.distance(s, q, child_idx, child_rt, best, depth + 1));
        }

        // Check shapes that intersect this node:
        for inter in &mut self.nodes[idx].intersect {
            inter.tests += 1;
            let shape_info = self.shapes[inter.shape_idx].as_ref().unwrap();
            let d = cached_dist(&mut self.dist_cache, inter.shape_idx, shape_info, s, q);
            best = min_opt(best, d);
        }
        self.maybe_push_down(idx, r, depth);

        best
    }

    // Move any shapes to child nodes, if necessary.
    fn maybe_push_down(&mut self, idx: NodeIdx, r: Rt, depth: usize) {
        if depth > MAX_DEPTH {
            return;
        }
        let push_down: Vec<_> =
            self.nodes[idx].intersect.extract_if(.., |v| v.tests >= TEST_THRESHOLD).collect();
        if !push_down.is_empty() {
            self.ensure_children(idx);

            for inter in push_down {
                let Node { bl, br, tr, tl, .. } = self.nodes[idx];
                let shape = self.shapes[inter.shape_idx].as_ref().unwrap().shape();

                // Put it into all children it intersects.
                for (quad, quad_idx) in [
                    (r.bl_quadrant().shape(), bl),
                    (r.br_quadrant().shape(), br),
                    (r.tr_quadrant().shape(), tr),
                    (r.tl_quadrant().shape(), tl),
                ] {
                    if shape.intersects_shape(&quad) {
                        self.nodes[quad_idx]
                            .intersect
                            .push(IntersectData { shape_idx: inter.shape_idx, tests: 0 });

                        if shape.contains_shape(&quad.shape()) {
                            self.nodes[quad_idx].contain.push(inter.shape_idx);
                        }
                    }
                }
            }
        }
    }

    fn ensure_children(&mut self, idx: NodeIdx) {
        if self.nodes[idx].bl == NO_NODE {
            self.nodes[idx].bl = self.nodes.len();
            self.nodes.push(Node::default());
            self.nodes[idx].br = self.nodes.len();
            self.nodes.push(Node::default());
            self.nodes[idx].tr = self.nodes.len();
            self.nodes.push(Node::default());
            self.nodes[idx].tl = self.nodes.len();
            self.nodes.push(Node::default());
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use pretty_assertions::assert_eq;
    use rand::prelude::SmallRng;
    use rand::{Rng, SeedableRng};
    use rust_dense_bitset::{BitSet, DenseBitSet};

    use super::*;
    use crate::geom::qt::query::{ALL, Kinds, KindsQuery, Query, Tag, TagQuery};
    use crate::primitive::{circ, poly, pt, rt, tri};

    #[test]
    fn quadtree_tri() -> Result<()> {
        let mut qt = QuadTree::new(vec![ShapeInfo::anon(
            tri(pt(1.0, 2.0), pt(5.0, 2.0), pt(4.0, 5.0)).shape(),
        )])?;
        for _ in 0..TEST_THRESHOLD {
            assert!(qt.intersects(&pt(3.0, 3.0).shape(), ALL));
        }

        assert!(qt.intersects(&pt(3.0, 3.0).shape(), ALL));
        assert!(qt.intersects(&rt(3.0, 3.0, 4.0, 4.0).shape(), ALL));
        Ok(())
    }

    #[test]
    fn quadtree_poly() -> Result<()> {
        let mut qt = QuadTree::new(vec![ShapeInfo::anon(
            poly(&[pt(1.0, 2.0), pt(5.0, 2.0), pt(4.0, 5.0)]).shape(),
        )])?;
        for _ in 0..TEST_THRESHOLD {
            assert!(qt.intersects(&pt(3.0, 3.0).shape(), ALL));
        }

        assert!(qt.intersects(&pt(3.0, 3.0).shape(), ALL));
        assert!(qt.intersects(&rt(3.0, 3.0, 4.0, 4.0).shape(), ALL));
        assert!(qt.contains(&pt(3.0, 3.0).shape(), ALL));
        assert!(!qt.contains(&rt(3.0, 3.0, 4.0, 4.0).shape(), ALL));
        assert_relative_eq!(qt.dist(&pt(3.0, 3.0).shape(), ALL).unwrap(), 0.0);
        assert_relative_eq!(qt.dist(&rt(3.0, 3.0, 4.0, 4.0).shape(), ALL).unwrap(), 0.0);
        assert_relative_eq!(qt.dist(&pt(5.0, 1.0).shape(), ALL).unwrap(), 1.0);
        Ok(())
    }

    #[test]
    fn quadtree_poly2() -> Result<()> {
        let poly = poly(&[
            pt(136.606, -131.891),
            pt(139.152, -134.437),
            pt(141.344, -132.245),
            pt(138.798, -129.699),
        ])
        .shape();
        let mut qt = QuadTree::new(vec![ShapeInfo::anon(poly.clone())])?;

        let mut r = SmallRng::seed_from_u64(0);
        for _ in 0..100 {
            let p0 = pt(r.random_range(-50.0..150.0), r.random_range(-150.0..-100.0));
            let p1 = pt(r.random_range(-50.0..150.0), r.random_range(-150.0..-100.0));
            assert_eq!(poly.contains_shape(&p0.shape()), qt.contains(&p0.shape(), ALL));
            let rt = Rt::enclosing(p0, p1);
            assert_eq!(poly.contains_shape(&rt.shape()), qt.contains(&rt.shape(), ALL));
            let c = circ(p0, r.random_range(0.01..100.0));
            assert_eq!(poly.contains_shape(&c.shape()), qt.contains(&c.shape(), ALL));
        }
        Ok(())
    }

    #[test]
    fn quadtree_remove_shape() -> Result<()> {
        let mut qt = QuadTree::new(vec![
            ShapeInfo::anon(rt(0.0, 0.0, 1.0, 1.0).shape()),
            ShapeInfo::anon(rt(2.0, 2.0, 3.0, 3.0).shape()),
        ])?;

        // Both shapes should intersect their respective areas
        assert!(qt.intersects(&pt(0.5, 0.5).shape(), ALL));
        assert!(qt.intersects(&pt(2.5, 2.5).shape(), ALL));

        // Remove the first shape
        qt.remove_shape(0);

        // First shape should no longer intersect
        assert!(!qt.intersects(&pt(0.5, 0.5).shape(), ALL));
        // Second shape should still intersect
        assert!(qt.intersects(&pt(2.5, 2.5).shape(), ALL));
        Ok(())
    }

    #[test]
    fn quadtree_remove_then_bounds_expansion_does_not_resurrect() -> Result<()> {
        let mut qt = QuadTree::new(vec![
            ShapeInfo::anon(rt(0.0, 0.0, 1.0, 1.0).shape()),
            ShapeInfo::anon(rt(2.0, 2.0, 3.0, 3.0).shape()),
        ])?;

        qt.remove_shape(0);
        assert!(!qt.intersects(&pt(0.5, 0.5).shape(), ALL));

        // Force a bounds expansion + tree rebuild.
        qt.add_shape(ShapeInfo::anon(rt(10.0, 10.0, 11.0, 11.0).shape()))?;

        // Removed shape must not come back after rebuild.
        assert!(!qt.intersects(&pt(0.5, 0.5).shape(), ALL));
        // Existing shapes still query correctly.
        assert!(qt.intersects(&pt(2.5, 2.5).shape(), ALL));
        assert!(qt.intersects(&pt(10.5, 10.5).shape(), ALL));

        Ok(())
    }

    #[test]
    fn quadtree_add_shape_reuses_free_slot() -> Result<()> {
        let mut qt = QuadTree::with_bounds(&rt(0.0, 0.0, 10.0, 10.0));

        // Add two shapes
        let idx1 = qt.add_shape(ShapeInfo::anon(rt(0.0, 0.0, 1.0, 1.0).shape()))?;
        let idx2 = qt.add_shape(ShapeInfo::anon(rt(2.0, 2.0, 3.0, 3.0).shape()))?;

        assert_eq!(idx1.len(), 1);
        assert_eq!(idx2.len(), 1);
        let first_idx = idx1[0];

        // Remove the first shape
        qt.remove_shape(first_idx);

        // Add a new shape - should reuse the free slot
        let idx3 = qt.add_shape(ShapeInfo::anon(rt(4.0, 4.0, 5.0, 5.0).shape()))?;
        assert_eq!(idx3.len(), 1);
        assert_eq!(idx3[0], first_idx); // Should reuse the same index
        Ok(())
    }

    #[test]
    fn quadtree_empty() -> Result<()> {
        let mut qt = QuadTree::new(vec![])?;
        assert!(!qt.intersects(&pt(0.0, 0.0).shape(), ALL));
        assert!(!qt.contains(&pt(0.0, 0.0).shape(), ALL));
        assert!(qt.dist(&pt(0.0, 0.0).shape(), ALL).is_none());
        Ok(())
    }

    #[test]
    fn quadtree_with_bounds_empty() {
        let mut qt = QuadTree::with_bounds(&rt(0.0, 0.0, 10.0, 10.0));
        assert!(!qt.intersects(&pt(5.0, 5.0).shape(), ALL));
        assert!(qt.bounds().is_some());
    }

    #[test]
    fn quadtree_bounds_expansion() -> Result<()> {
        let mut qt = QuadTree::new(vec![ShapeInfo::anon(rt(0.0, 0.0, 1.0, 1.0).shape())])?;
        let initial_bounds = qt.bounds().unwrap();

        // Add a shape outside the initial bounds
        qt.add_shape(ShapeInfo::anon(rt(10.0, 10.0, 11.0, 11.0).shape()))?;

        // Bounds should have expanded
        let new_bounds = qt.bounds().unwrap();
        assert!(new_bounds.w() > initial_bounds.w());
        assert!(new_bounds.h() > initial_bounds.h());
        Ok(())
    }

    #[test]
    fn query_by_tag() -> Result<()> {
        let tag1 = Tag(1);
        let tag2 = Tag(2);
        let mut qt = QuadTree::new(vec![
            ShapeInfo::new(rt(0.0, 0.0, 1.0, 1.0).shape(), tag1, Kinds(DenseBitSet::new())),
            ShapeInfo::new(rt(0.0, 0.0, 1.0, 1.0).shape(), tag2, Kinds(DenseBitSet::new())),
        ])?;

        let query_tag1 = Query(TagQuery::Tag(tag1), KindsQuery::All);
        let query_tag2 = Query(TagQuery::Tag(tag2), KindsQuery::All);

        // Query for tag1 should find the first shape
        assert!(qt.intersects(&pt(0.5, 0.5).shape(), query_tag1));
        // Query for tag2 should also find the shape (same position)
        assert!(qt.intersects(&pt(0.5, 0.5).shape(), query_tag2));
        Ok(())
    }

    #[test]
    fn query_except_tag() -> Result<()> {
        let tag1 = Tag(1);
        let tag2 = Tag(2);
        let mut qt = QuadTree::new(vec![
            ShapeInfo::new(rt(0.0, 0.0, 1.0, 1.0).shape(), tag1, Kinds(DenseBitSet::new())),
            ShapeInfo::new(rt(2.0, 2.0, 3.0, 3.0).shape(), tag2, Kinds(DenseBitSet::new())),
        ])?;

        let query_except_tag1 = Query(TagQuery::Except(tag1), KindsQuery::All);

        // Point in first rect should not match (tag1 excluded)
        assert!(!qt.intersects(&pt(0.5, 0.5).shape(), query_except_tag1));
        // Point in second rect should match (tag2 not excluded)
        assert!(qt.intersects(&pt(2.5, 2.5).shape(), query_except_tag1));
        Ok(())
    }

    #[test]
    fn query_by_kinds() -> Result<()> {
        let mut kinds1 = DenseBitSet::new();
        kinds1.set_bit(0, true);
        let mut kinds2 = DenseBitSet::new();
        kinds2.set_bit(1, true);

        let mut qt = QuadTree::new(vec![
            ShapeInfo::new(rt(0.0, 0.0, 1.0, 1.0).shape(), Tag(0), Kinds(kinds1)),
            ShapeInfo::new(rt(2.0, 2.0, 3.0, 3.0).shape(), Tag(0), Kinds(kinds2)),
        ])?;

        let query_kinds1 = Query(TagQuery::All, KindsQuery::HasCommon(Kinds(kinds1)));

        // Point in first rect should match (has kind bit 0)
        assert!(qt.intersects(&pt(0.5, 0.5).shape(), query_kinds1));
        // Point in second rect should not match (only has kind bit 1)
        assert!(!qt.intersects(&pt(2.5, 2.5).shape(), query_kinds1));
        Ok(())
    }

    #[test]
    fn query_dist_respects_filter() -> Result<()> {
        let tag1 = Tag(1);
        let tag2 = Tag(2);
        let mut qt = QuadTree::new(vec![
            ShapeInfo::new(rt(0.0, 0.0, 1.0, 1.0).shape(), tag1, Kinds(DenseBitSet::new())),
            ShapeInfo::new(rt(5.0, 5.0, 6.0, 6.0).shape(), tag2, Kinds(DenseBitSet::new())),
        ])?;

        let query_tag2 = Query(TagQuery::Tag(tag2), KindsQuery::All);

        // Distance from origin to tag2 shape should be > 0 (first shape excluded)
        let dist = qt.dist(&pt(0.0, 0.0).shape(), query_tag2).unwrap();
        assert!(dist > 0.0);

        // Distance from inside tag2 shape should be 0
        let dist = qt.dist(&pt(5.5, 5.5).shape(), query_tag2).unwrap();
        assert_relative_eq!(dist, 0.0);
        Ok(())
    }

    #[test]
    fn query_contains_respects_filter() -> Result<()> {
        let tag1 = Tag(1);
        let tag2 = Tag(2);
        let mut qt = QuadTree::new(vec![
            ShapeInfo::new(
                poly(&[pt(0.0, 0.0), pt(10.0, 0.0), pt(10.0, 10.0), pt(0.0, 10.0)]).shape(),
                tag1,
                Kinds(DenseBitSet::new()),
            ),
            ShapeInfo::new(
                poly(&[pt(20.0, 20.0), pt(30.0, 20.0), pt(30.0, 30.0), pt(20.0, 30.0)]).shape(),
                tag2,
                Kinds(DenseBitSet::new()),
            ),
        ])?;

        let query_tag2 = Query(TagQuery::Tag(tag2), KindsQuery::All);

        // Point at (5, 5) is inside tag1 polygon but not tag2
        assert!(!qt.contains(&pt(5.0, 5.0).shape(), query_tag2));
        // Point at (25, 25) is inside tag2 polygon
        assert!(qt.contains(&pt(25.0, 25.0).shape(), query_tag2));
        Ok(())
    }
}
