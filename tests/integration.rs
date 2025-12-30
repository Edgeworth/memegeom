use approx::assert_relative_eq;
use memegeom::Result;
use memegeom::geom::qt::quadtree::QuadTree;
use memegeom::geom::qt::query::{ALL, ShapeInfo};
use memegeom::primitive::{
    Boundary, Capsule, CapsuleExcl, Circle, CircleExcl, Compound, Line, Path, PathExcl, Poly,
    PolyExcl, Pt, PtI, Rt, RtExcl, RtI, Segment, Shape, ShapeOps, Tri, TriExcl, cap, circ, path,
    poly, pt, rt, seg,
};
use memegeom::tf::Tf;

const EP: f64 = 1e-6;

#[test]
fn shape_intersection_chain() {
    // Test that intersection is transitive across different shape types
    let circle = circ(pt(0.0, 0.0), 2.0);
    let rect = rt(-1.0, -1.0, 1.0, 1.0);
    let polygon = poly(&[pt(0.0, 0.0), pt(2.0, 0.0), pt(1.0, 2.0)]);

    // All should intersect with a common interior point
    let interior_pt = pt(0.5, 0.5);
    assert!(circle.intersects_shape(&interior_pt.shape()));
    assert!(rect.intersects_shape(&interior_pt.shape()));
    assert!(polygon.intersects_shape(&interior_pt.shape()));

    // All should intersect with each other
    assert!(circle.intersects_shape(&rect.shape()));
    assert!(rect.intersects_shape(&polygon.clone().shape()));
    assert!(circle.intersects_shape(&polygon.shape()));
}

#[test]
fn shape_containment_chain() {
    // Test containment relationships
    let outer_rect = rt(-10.0, -10.0, 10.0, 10.0);
    let inner_rect = rt(-5.0, -5.0, 5.0, 5.0);
    let inner_circle = circ(pt(0.0, 0.0), 3.0);
    let inner_point = pt(0.0, 0.0);

    assert!(outer_rect.contains_shape(&inner_rect.shape()));
    assert!(outer_rect.contains_shape(&inner_circle.shape()));
    assert!(outer_rect.contains_shape(&inner_point.shape()));
    assert!(inner_rect.contains_shape(&inner_circle.shape()));
    assert!(inner_circle.contains_shape(&inner_point.shape()));
}

#[test]
fn polygon_operations() {
    // Test complex polygon operations
    let square = poly(&[pt(0.0, 0.0), pt(10.0, 0.0), pt(10.0, 10.0), pt(0.0, 10.0)]);

    // Test point containment
    assert!(square.contains_shape(&pt(5.0, 5.0).shape()));
    assert!(!square.contains_shape(&pt(15.0, 5.0).shape()));

    // Test capsule containment
    let inner_capsule = cap(pt(3.0, 5.0), pt(7.0, 5.0), 1.0);
    assert!(square.contains_shape(&inner_capsule.shape()));

    let outer_capsule = cap(pt(3.0, 5.0), pt(15.0, 5.0), 1.0);
    assert!(!square.contains_shape(&outer_capsule.shape()));

    // Test path containment
    let inner_path = path(&[pt(2.0, 5.0), pt(8.0, 5.0)], 1.0);
    assert!(square.contains_shape(&inner_path.shape()));
}

#[test]
fn quadtree_operations() -> Result<()> {
    // Test quadtree with multiple shape types (using shapes that support distance)
    let shapes = vec![
        ShapeInfo::anon(rt(0.0, 0.0, 5.0, 5.0).shape()),
        ShapeInfo::anon(circ(pt(10.0, 10.0), 2.0).shape()),
        ShapeInfo::anon(poly(&[pt(20.0, 0.0), pt(25.0, 0.0), pt(22.5, 5.0)]).shape()),
    ];

    let mut qt = QuadTree::new(shapes)?;

    // Test intersection queries
    assert!(qt.intersects(&pt(2.5, 2.5).shape(), ALL));
    assert!(qt.intersects(&pt(10.0, 10.0).shape(), ALL));
    assert!(qt.intersects(&pt(22.5, 2.5).shape(), ALL));
    assert!(!qt.intersects(&pt(15.0, 15.0).shape(), ALL));

    // Test distance queries
    let dist_to_rect = qt.dist(&pt(-2.0, 2.5).shape(), ALL);
    assert!(dist_to_rect.is_some());
    assert_relative_eq!(dist_to_rect.unwrap(), 2.0, epsilon = EP);
    Ok(())
}

#[test]
fn transformation_pipeline() {
    // Test chaining transformations
    let original = rt(0.0, 0.0, 1.0, 1.0);

    let translate = Tf::translate(pt(5.0, 5.0));
    let scale = Tf::scale(pt(2.0, 2.0));
    let _rotate = Tf::rotate(45.0); // Prefixed with _ to silence unused warning

    // Apply transformations
    let combined = translate * scale;
    let result = combined.rt(&original);

    // The result should be at (5, 5) to (7, 7) after translate(5,5) * scale(2,2)
    if let memegeom::primitive::shape::Shape::Rect(r) = result {
        assert_relative_eq!(r.l(), 5.0, epsilon = EP);
        assert_relative_eq!(r.b(), 5.0, epsilon = EP);
        assert_relative_eq!(r.r(), 7.0, epsilon = EP);
        assert_relative_eq!(r.t(), 7.0, epsilon = EP);
    } else {
        panic!("Expected Rect shape");
    }

    // Test inverse transformation
    let tf = Tf::translate(pt(3.0, 4.0));
    let inv = tf.inv().expect("Translation should be invertible");
    let composed = tf * inv;
    let p = pt(10.0, 20.0);
    let result = composed.pt(p);
    assert_relative_eq!(result.x, p.x, epsilon = EP);
    assert_relative_eq!(result.y, p.y, epsilon = EP);
}

#[test]
fn distance_calculations() {
    // Test distance calculations between various shapes
    let rect = rt(0.0, 0.0, 1.0, 1.0);
    let point = pt(3.0, 0.5);

    // Point to rect distance
    let dist = rect.dist_to_shape(&point.shape());
    assert!(dist.is_some());
    assert_relative_eq!(dist.unwrap(), 2.0, epsilon = EP);

    // Capsule to rect distance
    let capsule = cap(pt(5.0, 0.0), pt(5.0, 1.0), 1.0);
    let cap_dist = rect.dist_to_shape(&capsule.shape());
    assert!(cap_dist.is_some());
    assert_relative_eq!(cap_dist.unwrap(), 3.0, epsilon = EP);

    // Overlapping shapes should have zero distance
    let overlapping_rect = rt(0.5, 0.5, 1.5, 1.5);
    let overlap_dist = rect.dist_to_shape(&overlapping_rect.shape());
    assert!(overlap_dist.is_some());
    assert_relative_eq!(overlap_dist.unwrap(), 0.0, epsilon = EP);
}

#[test]
fn segment_operations() {
    // Test segment operations
    let seg1 = seg(pt(0.0, 0.0), pt(10.0, 0.0));
    let seg2 = seg(pt(5.0, -5.0), pt(5.0, 5.0));

    // Segments should intersect
    assert!(seg1.intersects_shape(&seg2.shape()));

    // Test containment on segment
    assert!(seg1.contains(pt(5.0, 0.0)));
    assert!(!seg1.contains(pt(5.0, 1.0)));

    // Test bounds
    let bounds = seg1.bounds();
    assert!(bounds.is_some());
    let b = bounds.unwrap();
    assert_relative_eq!(b.l(), 0.0, epsilon = EP);
    assert_relative_eq!(b.r(), 10.0, epsilon = EP);
}

#[test]
fn capsule_operations() {
    // Test capsule operations
    let cap1 = cap(pt(0.0, 0.0), pt(10.0, 0.0), 2.0);
    let cap2 = cap(pt(5.0, 2.0), pt(5.0, 5.0), 1.0); // Moved closer so they intersect

    // Test intersection - cap1 extends to y=2 at x=5, cap2 starts at y=2 with r=1
    assert!(cap1.intersects_shape(&cap2.shape()));

    // Non-intersecting capsules
    let cap3 = cap(pt(5.0, 5.0), pt(5.0, 10.0), 1.0);
    assert!(!cap1.intersects_shape(&cap3.shape()));

    // Test bounds
    let bounds = cap1.bounds();
    assert!(bounds.is_some());
    let b = bounds.unwrap();
    assert_relative_eq!(b.l(), -2.0, epsilon = EP);
    assert_relative_eq!(b.r(), 12.0, epsilon = EP);
    assert_relative_eq!(b.b(), -2.0, epsilon = EP);
    assert_relative_eq!(b.t(), 2.0, epsilon = EP);
}

#[test]
fn path_operations() {
    // Test path operations
    let p = path(&[pt(0.0, 0.0), pt(5.0, 0.0), pt(10.0, 0.0)], 1.0);

    // Collinear points should be reduced
    assert_eq!(p.pts().len(), 2);

    // Test intersection with rect
    let rect = rt(4.0, -2.0, 6.0, 2.0);
    assert!(p.intersects_shape(&rect.shape()));

    // Test bounds
    let bounds = p.bounds();
    assert!(bounds.is_some());
    let b = bounds.unwrap();
    assert_relative_eq!(b.l(), -1.0, epsilon = EP);
    assert_relative_eq!(b.r(), 11.0, epsilon = EP);
}

#[test]
fn boundary_exclusion() {
    use memegeom::primitive::{cap_excl, circ_excl};

    // Test exclusive boundary shapes
    let c1 = circ(pt(0.0, 0.0), 1.0);
    let c2 = circ_excl(pt(2.0, 0.0), 1.0);

    // Touching circles: inclusive should intersect, exclusive should not
    assert!(!c1.intersects_shape(&c2.shape()));

    let cap1 = cap(pt(0.0, 0.0), pt(1.0, 0.0), 1.0);
    let cap2 = cap_excl(pt(3.0, 0.0), pt(4.0, 0.0), 1.0);

    // Touching capsules
    assert!(!cap1.intersects_shape(&cap2.shape()));
}

#[test]
fn primitive_defaults_construct() {
    let _ = Boundary::default();
    let _ = Pt::default();
    let _ = PtI::default();
    let _ = RtI::default();
    let _ = Capsule::default();
    let _ = CapsuleExcl::default();
    let _ = Circle::default();
    let _ = CircleExcl::default();
    let _ = Line::default();
    let _ = Path::default();
    let _ = PathExcl::default();
    let _ = Poly::default();
    let _ = PolyExcl::default();
    let _ = Rt::default();
    let _ = RtExcl::default();
    let _ = Segment::default();
    let _ = Tri::default();
    let _ = TriExcl::default();
    let _ = Compound::default();
    let _ = Shape::default();
}
