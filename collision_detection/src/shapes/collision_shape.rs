//! This module just wraps up the different shapes into one generic
//! "collision shape"

use super::circle::*;
use super::line_segment::*;
use super::convex_poly::*;


/// This is a way of abstracting the shapes.
///
/// Note we're not using polymorphism because it obfuscates the
/// details that we need when testing shapes for collision
#[derive(Clone, Debug)]
pub enum CollisionShape {
    Circle(Circle),
    ConvexPoly(ConvexPoly),
    LineSegment(LineSegment),
}

impl std::fmt::Display for CollisionShape {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
	match self
	{
	    CollisionShape::Circle(c) => {
		c.fmt(f)
	    },
	    CollisionShape::ConvexPoly(p) => {
		p.fmt(f)
	    },
	    CollisionShape::LineSegment(l) => {
		l.fmt(f)
	    },
	}
    }
}
