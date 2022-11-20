use crate::CollisionShape;

use super::shapes::Shape;
use math_util::Vector2f;

#[derive(Clone, Debug)]
pub struct AABB {
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
}

impl AABB {
    pub fn new(x: f32, y: f32, width: f32, height: f32) -> AABB {
	AABB {
	    x,
	    y,
	    width,
	    height,
	}
    }
    
    pub fn overlapping(&self, other: &AABB) -> bool {
	let overlap_on_x  = f32::abs(self.x - other.x) * 2.0 <= (self.width + other.width);
        let overlap_on_y = f32::abs(self.y - other.y) * 2.0 <= (self.height + other.height);

	overlap_on_x && overlap_on_y
    }

    pub fn from_shape(shape: &impl Shape) -> AABB {
	let x_axis = Vector2f::new(1.0, 0.0);
	let y_axis = Vector2f::new(0.0, 1.0);
	let min_x = shape.pos().x - shape.project(&-x_axis);
	let max_x = shape.pos().x + shape.project(&x_axis);
	let min_y = shape.pos().y - shape.project(&-y_axis);
	let max_y = shape.pos().y + shape.project(&y_axis);

	AABB {
	    x: (max_x + min_x) / 2.0,
	    y: (max_y + min_y) / 2.0,
	    width: max_x - min_x,
	    height: max_y - min_y,
	}
    }

    pub fn from_collision_shape(coll_shape: &CollisionShape) -> AABB {
	match coll_shape {
	    CollisionShape::Circle(s) => AABB::from_shape(s),
	    CollisionShape::LineSegment(s) => AABB::from_shape(s),
	    CollisionShape::ConvexPoly(s) => AABB::from_shape(s),
	}
    }
}


#[cfg(test)]
mod aabb_tests {
    use super::*;
    
    #[test]
    fn overlapping() {
	let aabb1 = AABB::new(0.0, 0.0, 10.0, 10.0);
	let aabb2 = AABB::new(4.0, 0.0, 10.0, 10.0);

	assert!(aabb1.overlapping(&aabb2));
    }

    #[test]
    fn not_overlapping() {
	let aabb1 = AABB::new(0.0, 0.0, 10.0, 10.0);
	let aabb2 = AABB::new(11.0, 0.0, 10.0, 10.0);

	assert!(!aabb1.overlapping(&aabb2));
    }
}
