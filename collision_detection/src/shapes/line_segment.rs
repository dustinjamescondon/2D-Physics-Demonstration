use math_util::*;
use super::*;

#[derive(Clone, Debug)]
pub struct LineSegment {
    pub vertex1 : Vector2f,
    pub vertex2 : Vector2f,
    pub normal : Vector2f,
    pub direction : Vector2f,
    pub length : f32,
}

impl std::fmt::Display for LineSegment {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
	write!(f, "vertex1: {}vertex2: {}", self.vertex1, self.vertex2)
    }
}

impl Volume for LineSegment {}

impl LineSegment {
    pub fn new(point1: Vector2f, point2: Vector2f) -> LineSegment {
	let direction = (point2 - point1).normalize();
	LineSegment {
	    vertex1: point1,
	    vertex2: point2,
	    direction,
	    normal: perp_counter_clockwise(&direction),
	    length: (point2 - point1).norm(),
	}
    }
   
    pub fn midpoint(&self) -> Vector2f {
	(self.point1() + self.point2()) / 2.0
    }

    pub fn as_edge(&self) -> Edge {
	Edge::new(self.point1(), self.point2())
    }

    pub fn point1(&self) -> Vector2f {
	self.vertex1
    }

    pub fn point2(&self) -> Vector2f {
	self.vertex2
    }

    pub fn to_shape(self) -> CollisionShape {
	CollisionShape::LineSegment(self)
    }

    /// 
    pub fn calc_min_depth(&self, p: &ConvexPoly) -> (f32, Edge) {
	// The only potential seperating axis has the projection
	// direction of the line normal
	let position_difference = p.pos() - self.pos();

	let position_difference_dot_edge_normal = position_difference.dot(&self.normal);
	
	// NOTE: this doesn't project the line segment because its projection will be zero
	// in the normal direction
	let depth_option1:f32 = p.project(&-self.normal) - position_difference_dot_edge_normal;
	let depth_option2:f32 = p.project(&self.normal) + position_difference_dot_edge_normal;

	
	if depth_option1 < depth_option2 {
	    (depth_option1, self.as_edge())
	}
	else {
	    (depth_option2, self.as_edge())
	}
    }
}

impl Shape for LineSegment {
    fn pos(&self) -> Vector2f {
	self.midpoint()
    }

    fn project(&self, dir: &Vector2f) -> f32 {
	maximum((self.vertex1 - self.pos()).dot(dir), (self.vertex2 - self.pos()).dot(dir))
    }
    
    fn transform(&self, transformation: &Matrix3f) -> LineSegment {
	LineSegment {
	    vertex1: transform_vector(transformation, &self.vertex1),
	    vertex2: transform_vector(transformation, &self.vertex2),
	    direction: transform_vector_rotation_only(transformation, &self.direction),
	    normal: perp_counter_clockwise(&self.direction),
	    length: self.length,
	}
    }

    fn rotate(&self, radians: f32) -> LineSegment {
	self.transform(&create_homogenous(Vector2f::zeros(), radians))
    }

    fn test_against_circle(&self, circle: &Circle) -> Option<Contact> {
	flip_contact_normal(circle.test_against_line(self))
    }

    fn test_against_line(&self, _line: &LineSegment) -> Option<Contact> {
	panic!("Line test against line not implemented yet!");

	/*let clipped_line = line.as_edge().clip(&self.point1(), &-self.direction)
	    .and_then(|edge| edge.clip(&self.point2(), &self.direction))?;

	let p1_depth = self.normal.dot(&(self.point1() - clipped_line.point1));
	let p2_depth = self.normal.dot(&(self.point1() - clipped_line.point2));

	// First check if the end points of the clipped line are on
	// different sides of this linesegment.
	if p1_depth * p2_depth < 0.0 {
	    if p1_depth > 0.0 {
		Contact {
		    normal
		}
	    }
	    else {
		
	    }
	}
	else {
	    None
	}*/
    }

    // Normal points towards self object
    fn test_against_poly(&self, p: &ConvexPoly) -> Option<Contact> {

	//--------------------------------------------------
	// First get penetration using line segment's axis
	//..................................................
	let (line_pen_depth, line_pen_edge) = self.calc_min_depth(p);
	//--------------------------------------------------
	// Then get min penetration using poly's axis
	//..................................................
	let poly_axis_info = p.calc_min_depth_line(self, line_pen_depth);


	// If either projection isn't penetrating, then the objects
	// aren't penetrating, by the seperating axis theorem.
	if line_pen_depth < 0.0 || poly_axis_info.depth < 0.0 {
	    return None;
	}

	let reference_face: &Edge;
	let incident_face: &Edge;

	let sign: f32;

	// If the line has less penetration, then make it the reference face, and
	// since it's the "self" object, and the line normal is away from self, then
	// flip the sign of the normal
	if line_pen_depth < poly_axis_info.depth {
	    reference_face = &line_pen_edge;
	    incident_face = &poly_axis_info.edge; // I'm assuming the incident face is this one
	    sign = -1.0;
	}
	else {
	    reference_face = &poly_axis_info.edge;
	    incident_face = &line_pen_edge; // Otherwise I'm assuming the incident face is this one
	    sign = 1.0;
	}

	
	/*------------------------------------------
	 * Clip the incident
	 * face against the sides of the Reference face
	 *
	 * If we've clipped away the incident face, then we aren't intersecting
	 * (note the ? operator)
	 *..........................................*/
	let clipped_incident_face = incident_face.clip(
	    &reference_face.point1,
	    &perp_counter_clockwise(&reference_face.normal))
	    .and_then(|r| r.clip(&r.point2,
		  &perp_clockwise(&r.normal)))?;

	/*------------------------------------------*/
	/* Either of the vertices that are part of the clipped
	 * incident face that are behind the Reference face are
	 * contact points
	 *..........................................*/

	// is the first point in the clipped incident face behind the reference face?
	let v1_depth = -(clipped_incident_face.point1 - reference_face.point1).dot(&reference_face.normal);
	let v2_depth = -(clipped_incident_face.point2 - reference_face.point1).dot(&reference_face.normal);

	let mut result = Contact::default();

	if v1_depth >= 0.0 {
	    result.points.push(
	    ContactPoint {
		point: clipped_incident_face.point1,
		depth: v1_depth,
	    });
	}
	
	if v2_depth >= 0.0 {
	    result.points.push(
		ContactPoint{
		    point: clipped_incident_face.point2,
		    depth: v2_depth,
		});
	}

	if result.points.is_empty() {
	    None
	} else {
	    result.normal = reference_face.normal * sign;
	    Some(result)
	}
    }

}
