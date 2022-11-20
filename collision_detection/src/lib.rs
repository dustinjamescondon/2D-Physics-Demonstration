//! This crate defines a number of primitive shapes--namely
//! ConvexPoly, Circle, and LineSegment--and the collision testing
//! routines between them.
//!
//! The collision checking routines return more than just if two
//! objects are colliding/overlapping; instead, the collision normal
//! and collision point are returned in the form of a collision
//! "Contact", which itself contains "ContactPoints"
//!
//! Note that when testing for a collision between two objects, the
//! collision normal will always point towards the first (or self)
//! object used in the test.

mod info;
pub mod shapes;
pub mod aabb;
extern crate math_util;
pub use shapes::*;
pub use info::*;
pub use aabb::*;
mod collision_tests;

impl CollisionShape {
    /// Create a transformed version of this shape
    pub fn transform(&self, transformation: &Matrix3f) -> CollisionShape {
	match self {
	    CollisionShape::Circle(c) => { c.transform(transformation).to_shape() },
	    CollisionShape::ConvexPoly(p) => { p.transform(transformation).to_shape() },
	    CollisionShape::LineSegment(l) => {l.transform(transformation).to_shape() },
	}
    }

    /// Create a rotated version of this shape
    pub fn rotate(&self, radians: f32) -> CollisionShape {
	match self {
	    CollisionShape::Circle(c) => { c.rotate(radians).to_shape() },
	    CollisionShape::ConvexPoly(p) => { p.rotate(radians).to_shape() },
	    CollisionShape::LineSegment(l) => {l.rotate(radians).to_shape() },
	}
    }

    /// Performs a collision test between CollisionShapes, returning a contact
    /// if they are overlapping, and None otherwise.
    pub fn test_against(&self, other:& CollisionShape) -> Option<Contact> {
	
	// Since the abstraction that happens with polymorphism only
	// hinders the specifics of collision detection between
	// different shapes (although the "visitor pattern" might
	// help) we instead wrap the different shapes into a rust enum
	// and handle things case-by-case
        match (self, other) {
            (CollisionShape::Circle(circ), CollisionShape::LineSegment(line)) => circ.test_against_line(line),
            (CollisionShape::Circle(circ1), CollisionShape::Circle(circ2)) => circ1.test_against_circle(circ2),
            (CollisionShape::Circle(circ), CollisionShape::ConvexPoly(poly)) => circ.test_against_poly(poly),
            (CollisionShape::LineSegment(line), CollisionShape::Circle(circ)) => line.test_against_circle(circ),
	    (CollisionShape::LineSegment(line), CollisionShape::ConvexPoly(poly)) => line.test_against_poly(poly),
	    (CollisionShape::LineSegment(line1), CollisionShape::LineSegment(line2)) => line1.test_against_line(line2),	    
	    (CollisionShape::ConvexPoly(poly1), CollisionShape::ConvexPoly(poly2)) => poly1.test_against_poly(poly2),
	    (CollisionShape::ConvexPoly(poly), CollisionShape::Circle(circ)) => poly.test_against_circle(circ),
	    (CollisionShape::ConvexPoly(poly), CollisionShape::LineSegment(line)) => poly.test_against_line(line),
        } 
    }

    /// Returns whether the passed pt is inside this shape
    pub fn is_pt_inside(&self, pt: &Vector2f) -> bool {
        match self {
            CollisionShape::Circle(c) => c.is_pt_inside(pt),
            CollisionShape::LineSegment(line) => line.is_pt_inside(pt),
            CollisionShape::ConvexPoly(poly) => poly.is_pt_inside(pt),
        }
    }
}
