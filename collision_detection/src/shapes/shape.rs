use super::*;

pub trait Volume {
    fn is_pt_inside(&self, _pt: &Vector2f) -> bool {
        false
    }
}

pub trait Shape {
    /// Returns the position of the shape
    fn pos(&self) -> Vector2f;
    
    /// Returns how far the shape extends in the axis direction
    /// relative to the shape's "pos()"
    fn project(&self, axis: &Vector2f) -> f32;

    /// Applies the transformation to the shape returning a new
    /// transformed shape
    fn transform(&self, trans: &Matrix3f) -> Self;

    /// Applies just a rotation transformation to the shape, returning
    /// a new transformed shape
    fn rotate(&self, radians: f32) -> Self;

    /// Tests this shape against the given circle. If they are
    /// colliding/overlapping then a contact is returned, which
    /// specificed the contact normal and points
    fn test_against_circle(&self, c: &Circle) -> Option<Contact>;

    /// Tests this shape against the given LineSegment. If they are
    /// colliding/overlapping then a contact is returned, which
    /// specificed the contact normal and points
    fn test_against_line(&self, l: &LineSegment) -> Option<Contact>;

    /// Tests this shape against the given ConvexPoly. If they are
    /// colliding/overlapping then a contact is returned, which
    /// specificed the contact normal and points
    fn test_against_poly(&self, p: &ConvexPoly) -> Option<Contact>;
}


/// Applies to any shape that contains edges, like a polygon or line
/// segment
pub trait Edged {
    /// Returns all the edges contained in the object
    fn edges(&self) -> Vec<Edge>;
}

/// Finds the axis (represented by an edge and its normal) with the minimum
/// penetration when both shapes are projected onto it. Note these axis/edges
/// are only taken from shape a
pub fn calc_min_penetration_axis(edges: &Vec<Edge>, a: &impl Shape, b: &impl Shape) -> PenetrationInfo
{
    let position_difference = b.pos() - a.pos();

    let mut min_penetration: Option<f32> = None;
    let mut min_edge: Option<&Edge> = None;
    
    for edge in edges {
	let position_difference_dot_edge_normal = position_difference.dot(&edge.normal);
	
	// Don't use this edge as a seperating one, because it's facing away from the other object
	if position_difference_dot_edge_normal < 0.0 { continue; }

	let depth = (a.project(&edge.normal) + b.project(&-edge.normal)) -
	    position_difference_dot_edge_normal;

	// NOTE: Relying on early exit of condition here
	if min_penetration.is_none() || depth < min_penetration.unwrap() {
	    min_penetration = Some(depth);
	    min_edge = Some(edge);
	}
    }

    assert!(min_edge.is_some());
    PenetrationInfo::new(min_penetration.unwrap(), min_edge.unwrap())
}
