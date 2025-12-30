use rust_dense_bitset::DenseBitSet;

use crate::geom::qt::quadtree::ShapeIdx;
use crate::primitive::ShapeOps;
use crate::primitive::shape::Shape;

#[must_use]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Tag(pub usize);

#[must_use]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Kinds(pub DenseBitSet);

pub const NO_TAG: Tag = Tag(usize::MAX);
pub const ALL: Query = Query(TagQuery::All, KindsQuery::All);

#[must_use]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum TagQuery {
    All,
    Tag(Tag),
    Except(Tag),
}

#[must_use]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum KindsQuery {
    All,
    HasCommon(Kinds), // Query all shapes who have a common kind with the query value.
}

#[must_use]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Query(pub TagQuery, pub KindsQuery);

fn matches_tag_query(s: &ShapeInfo, q: TagQuery) -> bool {
    match q {
        TagQuery::All => true,
        TagQuery::Tag(tag) => tag == s.tag,
        TagQuery::Except(tag) => tag != s.tag,
    }
}

fn matches_kinds_query(s: &ShapeInfo, q: KindsQuery) -> bool {
    match q {
        KindsQuery::All => true,
        KindsQuery::HasCommon(kinds) => (kinds.0 & s.kinds.0).any(),
    }
}

#[must_use]
pub fn matches_query(s: &ShapeInfo, q: Query) -> bool {
    matches_tag_query(s, q.0) && matches_kinds_query(s, q.1)
}

#[must_use]
#[derive(Debug, Clone)]
pub struct ShapeInfo {
    shape: Shape,
    tag: Tag,
    kinds: Kinds, // A bitmask.
}

impl ShapeInfo {
    pub fn new(shape: Shape, tag: Tag, kinds: Kinds) -> Self {
        Self { shape, tag, kinds }
    }

    pub fn anon(shape: Shape) -> Self {
        Self { shape, tag: NO_TAG, kinds: Kinds(DenseBitSet::new()) }
    }

    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    pub fn tag(&self) -> Tag {
        self.tag
    }

    pub fn kinds(&self) -> Kinds {
        self.kinds
    }
}

// Split paths up so they are spread out more.
// Split compound shapes up.
pub fn decompose_shape(s: ShapeInfo) -> Vec<ShapeInfo> {
    let shapes = match s.shape {
        Shape::Compound(c) => c.quadtree().shapes().map(|v| v.shape.clone()).collect(),
        Shape::Path(p) => p.caps().map(ShapeOps::shape).collect(),
        Shape::PathExcl(p) => p.caps().map(ShapeOps::shape).collect(),
        _ => vec![s.shape.clone()],
    };
    let tag = s.tag;
    let kinds = s.kinds;
    shapes.into_iter().map(|shape| ShapeInfo { shape, tag, kinds }).collect()
}

pub fn cached_intersects<S: ::std::hash::BuildHasher>(
    cache: &mut std::collections::HashMap<ShapeIdx, bool, S>,
    idx: ShapeIdx,
    shape_info: &ShapeInfo,
    s: &Shape,
    q: Query,
) -> bool {
    if !matches_query(shape_info, q) {
        return false;
    }
    if let Some(res) = cache.get(&idx) {
        return *res;
    }
    let res = shape_info.shape().intersects_shape(s);
    cache.insert(idx, res);
    res
}

pub fn cached_contains<S: ::std::hash::BuildHasher>(
    cache: &mut std::collections::HashMap<ShapeIdx, bool, S>,
    idx: ShapeIdx,
    shape_info: &ShapeInfo,
    s: &Shape,
    q: Query,
) -> bool {
    if !matches_query(shape_info, q) {
        return false;
    }
    if let Some(res) = cache.get(&idx) {
        return *res;
    }
    let res = shape_info.shape().contains_shape(s);
    cache.insert(idx, res);
    res
}

pub fn cached_dist<S: ::std::hash::BuildHasher>(
    cache: &mut std::collections::HashMap<ShapeIdx, Option<f64>, S>,
    idx: ShapeIdx,
    shape_info: &ShapeInfo,
    s: &Shape,
    q: Query,
) -> Option<f64> {
    if !matches_query(shape_info, q) {
        return None;
    }
    if let Some(res) = cache.get(&idx) {
        return *res;
    }
    let res = shape_info.shape().dist_to_shape(s);
    cache.insert(idx, res);
    res
}
