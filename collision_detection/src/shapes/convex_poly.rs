use super::*;
pub use crate::info::*;

#[derive(Clone, Debug)]
pub struct ConvexPoly{
    pub pos : Vector2f,
    
    // Note: these are the local vertices, which are independent of pos
    // in order to get the global vertices, call ConvexPoly::global_vertices()
    vertices : Vec<Vector2f>, 
    pub edges: Vec<Edge>,
}

impl std::fmt::Display for ConvexPoly {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {

	let vertices: Vec<String> = self.vertices.iter().map(|v| format!("{v}")).collect();
	let vertices_string = vertices.join("");
	write!(f, "position: {}vertices: {}", self.pos, vertices_string)
    }
}

impl Default for ConvexPoly {
    fn default() -> ConvexPoly {
	ConvexPoly {
	    pos: Vector2f::zeros(),
	    vertices: Vec::<Vector2f>::new(),
	    edges: Vec::<Edge>::new(),
	}
    }
}

impl Edged for ConvexPoly {
    fn edges(&self) -> Vec<Edge> {
	self.edges.iter().map(|e| {
	    Edge::new(self.pos + e.point1, self.pos + e.point2)
	}).collect()
    }
}

impl ConvexPoly {
    pub fn set_vertices(&mut self, vertices: &Vec<Vector2f>) -> &mut Self {
	self.vertices = vertices.clone();
	self.edges = vec![Edge::default(); vertices.len()];
	for i in 0..vertices.len() {
	    self.edges[i] = Edge::new(self.vertices[i],
				      self.vertices[(i+1) % self.vertices.len()]);
	}
	self
    }

    pub fn new_box(width: f32, height: f32) -> ConvexPoly {
	let mut poly = ConvexPoly::default();
	let width_2 = width / 2.0;
	let height_2 = height / 2.0;
	poly.set_vertices(&vec![Vector2f::new(-width_2, -height_2),
				Vector2f::new(width_2, -height_2),
				Vector2f::new(width_2, height_2),
				Vector2f::new(-width_2, height_2)]);
	poly
    }
    
    pub fn to_shape(self) -> CollisionShape {
	CollisionShape::ConvexPoly(self)
    }

    pub fn global_vertices(&self) -> Vec<Vector2f> {
	self.vertices.iter().map(|v| {
	    self.pos + v
	}).collect()
    }

    /// Returns the line segments making up the convex polygon
    pub fn line_segments(&self) -> Vec<LineSegment> {
	self.edges.iter().map(|e| {
	    LineSegment::new(self.pos + e.point1, self.pos + e.point2)
	}).collect()
    }

    /// Returns the edge whose normal is opposing the reference face's
    /// normal most directly
    pub fn find_incident_face(&self, reference_face: &Edge) -> Edge {
	let mut max_opposition: f32 = 0.0;
	let mut incident_face: Option<Edge> = None;
	for edge in self.edges() {
	    let proj = reference_face.normal.dot(&edge.normal);
	    if -proj > max_opposition {
		max_opposition = -proj;
		incident_face = Some(edge);
	    }
	}
	assert!(incident_face.is_some());
	incident_face.unwrap()
    }

    // TODO clean this up
    /// Returns the minimum projection penetration between all edges
    /// in this convex polygon and the edge containing the axis it was
    /// determined with.
    pub fn calc_min_penetration_axis_wrt_line(&self, line: &LineSegment, line_axis_overlap: f32) -> PenetrationInfo {
	let position_difference = line.pos() - self.pos();
	let mut min_penetration: Option<f32> = None;
	let mut min_edge: Option<Edge> = None;
	
	for edge in self.edges() {
	    let position_difference_dot_edge_normal = position_difference.dot(&edge.normal);
	    //--------------------------------------------------
	    // First, see how far we have to move the this object in
	    // the backwards normal direction of this edge to make it
	    // not overlap in the line's axis direction
	    //..................................................
	    let cos_theta = (-edge.normal).dot(&line.normal);
	    let mut option1 = f32::MAX;

	    // If the normal's are opposing, then do actually consider
	    // this overlapping distance Otherwise, just leave option1
	    // as None
	    if cos_theta > 0.0 {
		option1 = line_axis_overlap / cos_theta;
	    }
	    

	    // Second, see how far what the penetration of the polygon
	    // and line is wrt this edge
	    let option2 = (self.project(&edge.normal) + line.project(&-edge.normal)) -
		position_difference_dot_edge_normal;

	    // Return the axis with the smallest penetration
	    if option1 < option2 {
		if min_penetration.is_none() || option1 < min_penetration.unwrap() {
		    min_penetration = Some(option1);
		    min_edge = Some(edge.clone());
		}
	    } else if min_penetration.is_none() || option2 < min_penetration.unwrap() {
		min_penetration = Some(option2);
		min_edge = Some(edge.clone());
	    }
	}

	assert!(min_penetration.is_some());
	PenetrationInfo::new(min_penetration.unwrap(), &min_edge.unwrap())
    }
}

impl Shape for ConvexPoly {
    fn pos(&self) -> Vector2f {
	self.pos
    }

    fn project(&self, dir: &Vector2f) -> f32 {
	let mut max_so_far = 0.0;
	for v in &self.vertices {
	    let proj = v.dot(dir);
	    if proj > max_so_far { max_so_far = proj; }
	}
	max_so_far
    }
    
    fn transform(&self, transformation: &Matrix3f) -> ConvexPoly {
	let vertices: Vec<Vector2f> = self.vertices.iter().map(
	    |v| transform_vector_rotation_only(transformation, v)).collect();
	let mut result = ConvexPoly {
	    pos: transform_vector(transformation, &self.pos),
	    ..Default::default()
	};
	result.set_vertices(&vertices);
	result
    }

    fn rotate(&self, radians: f32) -> ConvexPoly {
	let mut result = self.clone();
	let rotated_vertices: Vec<Vector2f> = self.vertices.iter().map(|v| rotate_about_point(v, radians, &Vector2f::zeros())).collect();
	result.set_vertices(&rotated_vertices);
	result
    }

    fn test_against_circle(&self, c: &Circle) -> Option<Contact>
    {
	let line_segments = self.line_segments();
	for line_seg in line_segments {
	    let contact = line_seg.test_against_circle(c);
	    if contact.is_some() {
		return contact;
	    }
	}
	None
    }

    fn test_against_line(&self, l: &LineSegment) -> Option<Contact>
    {
	flip_contact_normal(l.test_against_poly(self))
    }

    fn test_against_poly(&self, p: &ConvexPoly) -> Option<Contact> {

	// Find which axis from object a (self) has minimum penetration
	let a_pen_info = calc_min_penetration_axis(&self.edges(), self, p);

	// Find which axis from object b has minimum penetration
	let b_pen_info = calc_min_penetration_axis(&p.edges(), p, self);
	
	// Now see which result contains the reference face (axis of min penetration depth)

	// If either isn't penetrationg, then by the seperating-axis theorem,
	// they aren' colliding/overlapping
	if a_pen_info.depth < 0.0 || b_pen_info.depth < 0.0 {
	    return None;
	}

	let reference_face: Edge;
	let incident_face: Edge;

	let sign: f32;

	if a_pen_info.depth < b_pen_info.depth {
	    // shape a (self) is the object containing the axis of
	    // minimum penetration, and therefore the "reference
	    // face", and shape b contains the "incident face"
	    reference_face = a_pen_info.edge;
	    incident_face = p.find_incident_face(&reference_face);
	    sign = 1.0;
	} else {
	    reference_face = b_pen_info.edge;
	    incident_face = self.find_incident_face(&reference_face);

	    // since we want the normal to be away from "this" object, we will eventually need
	    // to flip it (because we're using the edge normal from object2)
	    sign = -1.0;
	}

	// Clip the incident face against the sides of the Reference face
	let clipped_incident_face_result = incident_face.clip(
	    &reference_face.point1, &perp_counter_clockwise(&reference_face.normal)).and_then(|r|
	    r.clip(&reference_face.point2, &perp_clockwise(&reference_face.normal)));
	// If we've clipped away the incident face, then we aren't intersecting
	let clipped_incident_face = clipped_incident_face_result?;

	// Either of the vertices that are part of the clipped
	// incident face that are behind the Reference face are
	// contact points

	// Having a positive depth means the clipped point is behind
	// the reference face, meaning penetration
	let v1_depth = -(clipped_incident_face.point1 - reference_face.point1).dot(&reference_face.normal);
	let v2_depth = -(clipped_incident_face.point2 - reference_face.point1).dot(&reference_face.normal);

	let mut result = Contact {
	    normal: -reference_face.normal * sign, // flip the sign if we need to
	    .. Default::default()
	};
        
	// if first clipped point is behind the reference face, then
	// it's penetrating
	if v1_depth >= 0.0 {
	    result.points.push(
		ContactPoint{
		    point: clipped_incident_face.point1,
		    depth: v1_depth,
		});
	}
	
	// if second clipped point is behind the reference face, then
	// it's penetrating
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
	    Some(result)
	}
    }
}

impl Volume for ConvexPoly {
    fn is_pt_inside(&self, pt: &Vector2f) -> bool {
	// if there's a single edge which the point lying on the
	// side the normal points, then the point isn't in the shape
	// (because it's convex)
	for edge in &self.edges {
	    let delta_pos = pt - (edge.point1 + self.pos);
	    let proj = delta_pos.dot(&edge.normal);
	    if proj > 0.0 { return false; }
	}

	// Otherwise, the point must be inside the shape
	true
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_print() {
	let c = ConvexPoly::new_box(10.0, 10.0);
	println!("{}", c);
    }
}
