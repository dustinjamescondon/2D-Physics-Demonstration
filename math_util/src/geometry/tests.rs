#[cfg(test)]
use super::*;

#[test]
fn clip_test1() {
    let edge = Edge::new(Vector2f::new(0.0, 0.0), Vector2f::new(10.0, 0.0));
    let clipped_edge = edge.clip(&Vector2f::new(5.0, 0.0), &Vector2f::new(1.0, 0.0));
    let expected_p2 = Vector2f::new(5.0, 0.0);
    assert!((clipped_edge.unwrap().point2 - expected_p2).norm() < 1.0e-5);
}

#[test]
fn clip_away() {
    let edge = Edge::new(Vector2f::new(0.0, 0.0), Vector2f::new(1.0, 0.0));
    {
	let clipped_edge = edge.clip(&Vector2f::new(1.5, 10.0), &Vector2f::new(-1.0, 0.0));
	assert!(clipped_edge.is_none());
    }
    {
	let clipped_edge = edge.clip(&Vector2f::new(-1.5, 10.0), &Vector2f::new(-1.0, 0.0));
	assert!(clipped_edge.is_some());
	assert!(clipped_edge.unwrap() == edge);
    }
}

#[test]
fn clip_test2() {
    let edge = Edge::new(Vector2f::new(0.0, 0.0), Vector2f::new(10.0, 0.0));
    let clipped_edge = edge.clip(&Vector2f::new(5.0, 0.0), &Vector2f::new(-1.0, 0.0));
    let expected_p1 = Vector2f::new(5.0, 0.0);
    assert!((clipped_edge.unwrap().point1 - expected_p1).norm() < 1.0e-5);
}

