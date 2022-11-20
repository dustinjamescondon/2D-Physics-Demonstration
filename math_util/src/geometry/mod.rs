pub use super::*;
mod tests;

#[derive(Clone, PartialEq, Debug)]
pub struct Edge {
    pub normal: Vector2f,
    pub point1: Vector2f,
    pub point2: Vector2f,
}

impl std::fmt::Display for Edge {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
	write!(f, "normal:{}point1:{}point2{}", self.normal, self.point1, self.point2)
    }
}

impl Edge {
    pub fn new(point1: Vector2f, point2: Vector2f) -> Edge {
	Edge {
	    point1,
	    point2,
	    normal: perp_counter_clockwise(&(point2 - point1).normalize()), // TODO check this is right direction
	}
    }

    // Clips the edge given a point and normal, representing a
    // line. The segment of the edge that fall on the side of the line that
    // the normal points to is removed from the edge.
    pub fn clip(&self, clip_point: &Vector2f, line_normal: &Vector2f) -> Option<Edge> {
	let clip_line_direction = perp_clockwise(line_normal);
	let point1_distance = (self.point1-clip_point).dot(line_normal);
	let point2_distance = (self.point2-clip_point).dot(line_normal);

	// first check if the two points of the line segement are on either side of the
	// clipping line
	if point1_distance * point2_distance > 0.0  {
	    // If they're both on the side pointed to by the clipping line normal, then
	    // we clip the entire edge away
	    if point1_distance > 0.0 {
		return None
	    }
	    // Otherwise both points are on the other side, in which case we keep the entire edge
	    else {
		return Some(self.clone());
	    }
	}
	// otherwise, find the intersection point
	let numerator = (self.point1 - clip_point).dot(&self.normal);
	let denominator = self.normal.dot(&clip_line_direction);

	// TODO keep the edge or not?
	if denominator.abs() < 1.0e-5 {
	    return Some(self.clone());
	}

	let t_prime = numerator/denominator;
	let intersection_point = clip_point + (t_prime * clip_line_direction);
	if point1_distance > 0.0 {
	    Some(Edge::new(intersection_point, self.point2))
	}
	else {
	    Some(Edge::new(self.point1, intersection_point))
	}
    }
}

impl Default for Edge {
    fn default() -> Edge {
	Edge::new(Vector2f::zeros(), Vector2f::new(1.0, 1.0))
    }
}
