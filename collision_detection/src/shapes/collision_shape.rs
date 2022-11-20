use super::circle::*;
use super::line_segment::*;
use super::convex_poly::*;


// TODO might be good to still keep these just to store shapes,
// because once we define collision fixtures, we use it instead
// of the raw shapes
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
