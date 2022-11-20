
#[cfg(test)]
use super::*;

#[test]
fn circle_doesnt_overlap_circle() {
    let a = Circle::new(
        Vector2f::new(0.0,0.0),
        5.0);
    
    let b = Circle::new(
        Vector2f::new(10.0,0.0),
        2.0);

    let result = a.test_against_circle(&b);
    assert!( result.is_none() );
}

#[test]
fn circle_does_overlap_circle() {
    let a = Circle::new(Vector2f::new(0.0,0.0), 6.0);
    let b = Circle::new(Vector2f::new(10.0,0.0), 6.0);

    let result = a.test_against_circle(&b);
    assert!(result.is_some());
}

#[test]
fn edges_correct_in_convex_poly() {
    let mut poly = ConvexPoly::default();
    let vertices: Vec<Vector2f> = vec![Vector2f::new(0.0, 0.0), Vector2f::new(1.0, 0.0),
				       Vector2f::new(1.0, 1.0)];
    poly.set_vertices(&vertices);
    assert!(poly.edges[0].point1 == vertices[0]);
    assert!(poly.edges.last().unwrap().point2 == vertices[0]);
}

#[test]
fn calc_min_depth_test1() {
    let mut poly1 = ConvexPoly::default();
    let mut poly2 = ConvexPoly::default();

    poly1.pos = Vector2f::new(0.0,0.0);
    poly1.set_vertices(&vec![Vector2f::new(-1.0, -1.0), Vector2f::new(1.0, -1.0),
			     Vector2f::new(1.0, 1.0), Vector2f::new(-1.0, 1.0)]);

    poly2.pos = Vector2f::new(1.0,0.0);
    poly2.set_vertices(&vec![Vector2f::new(-1.0, -1.0), Vector2f::new(1.0, -1.0),
			     Vector2f::new(1.0, 1.0), Vector2f::new(-1.0, 1.0)]);

    let result = poly1.calc_min_depth_wrt_own_axes(&poly2);
    let (depth, edge) = result;
    assert!((depth - 1.0).abs() < 0.001f32);
    assert!(edge.normal == Vector2f::new(1.0, 0.0));
}

#[test]
fn calc_min_depth_test2() {
    let mut poly1 = ConvexPoly::default();
    let mut poly2 = ConvexPoly::default();

    poly1.pos = Vector2f::new(-2.0,0.0);
    poly1.set_vertices(&vec![Vector2f::new(-1.0, -1.0), Vector2f::new(1.0, -1.0),
			     Vector2f::new(1.0, 1.0), Vector2f::new(-1.0, 1.0)]);

    poly2.pos = Vector2f::new(2.0,0.0);
    poly2.set_vertices(&vec![Vector2f::new(-1.0, -1.0), Vector2f::new(1.0, -1.0),
			     Vector2f::new(1.0, 1.0), Vector2f::new(-1.0, 1.0)]);

    let result = poly1.calc_min_depth_wrt_own_axes(&poly2);
    let (depth, edge) = result;
    assert!(depth < 0.0);
    assert!(edge.normal == Vector2f::new(1.0, 0.0));
}

#[test]
fn convex_poly_transform_test() {
    let mut poly = ConvexPoly::new_box(10.0, 10.0);
    poly.pos = Vector2f::new(1.0, 1.0);
    let transformed_poly = poly.transform(&Matrix3f::identity());

    assert!(transformed_poly.vertices == poly.vertices);
    assert!(transformed_poly.pos == poly.pos);
}

