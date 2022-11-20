use crate::math_util::*;
use super::collision_shape::*;
use super::shape::*;
use crate::contact::*;
use super::line_segment::*;
use super::convex_poly::*;

#[derive(Clone, Debug)]
pub struct Circle {
    pub pos : Vector2f,
    pub radius : f32,
}

impl std::fmt::Display for Circle {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
	write!(f, "position: {}\nradius: {}", self.pos, self.radius)
    }
}

impl Circle {
    pub fn new(pos: Vector2f, radius: f32) -> Circle {
	Circle {
	    pos,
	    radius,
	}
    }
    
    pub fn is_pt_inside(&self, pt: &Vector2f) -> bool {
        (self.pos - pt).norm() - self.radius < 0.0
    }

    pub fn to_shape(self) -> CollisionShape {
	CollisionShape::Circle(self)
    }
}

impl Volume for Circle {
    fn is_pt_inside(&self, _pt: &Vector2f) -> bool {
	false
    }
}

impl Shape for Circle {
    fn pos(&self) -> Vector2f {
	self.pos
    }

    fn project(&self, _dir: &Vector2f) -> f32 {
	self.radius
    }
    
    fn transform(&self, transformation: &Matrix3f) -> Circle {
	let pos_prime = transform_vector(transformation, &self.pos);
	Circle {
	    pos: pos_prime,
	    radius: self.radius,
	}
    }

    fn rotate(&self, radians: f32) -> Circle {
	self.transform(&create_homogenous(Vector2f::zeros(), radians))
    }

    fn test_against_circle(&self, circ: &Circle) -> Option<Contact> {
	let dist : Vector2f = self.pos - circ.pos;
	let depth = self.radius + circ.radius - dist.norm();
	let normal = dist.normalize();

	if depth >= 0.0 {
	    Some(Contact {
		normal,
		points: vec![ContactPoint {
		    point: circ.pos + ((self.radius - depth * 0.5) * normal),
		    depth,
		}],
	    })
	}
	else {
	    None
	}
    }
    
    fn test_against_line(&self, line: &LineSegment) -> Option<Contact> {
	// Note: Want normal always facing towards the self
	
	// Use this to define a strip orthog to the line connecting the two points
	let line_point1 = line.point1();
	let line_point2 = line.point2();
	let dir_p1_to_p2 = (line_point2 - line_point1).normalize();
	let line_normal = perp_counter_clockwise(&dir_p1_to_p2);
	let delta_c_to_point1 = line_point1 - self.pos;
	let delta_c_to_point2 = line_point2 - self.pos;
	let is_off_lhs_strip = delta_c_to_point1.dot(&dir_p1_to_p2) > 0.0;
	let is_off_rhs_strip = delta_c_to_point2.dot(&(-dir_p1_to_p2)) > 0.0;

	let mut distance : f32;
	let normal : Option<Vector2f>;
	let point : Option<Vector2f>;

	match(is_off_lhs_strip, is_off_rhs_strip) {
	    (true, false) =>  {
		distance = delta_c_to_point1.norm();
		normal = Some(-delta_c_to_point1.normalize());
		point = Some(line_point1);
	    }
	    (false, true) => {
		distance = delta_c_to_point2.norm();
		normal = Some(-delta_c_to_point2.normalize());
		point = Some(line_point2);
	    },
	    (false, false) => {
		distance = delta_c_to_point1.dot(&(-line_normal));
		// Make sure normal is pointing the right way
		normal = Some(
		    if distance < 0.0 { -line_normal }
		    else { line_normal }
		);
		// Make sure it's always positive
		distance = distance.abs();
		point = Some(line_point1 + proj_onto(&(-delta_c_to_point1), &dir_p1_to_p2));
	    }
	    (true, true) => {
		panic!("This isn't possible! So probably need an epsilon somewhere...")
	    }
	};

	let depth = self.radius - distance;
	// Now use our results to return a result
	if depth >= 0.0 {
	    Some(Contact {
		normal: normal.unwrap(),
		points: vec![ContactPoint {
		    point: point.unwrap(),
		    depth,
		}],
	    })
	}
	else { None }
    }

    fn test_against_poly(&self, p: &ConvexPoly) -> Option<Contact> {
	flip_contact_normal(p.test_against_circle(self))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sandbox() {
	let c = Circle::new(Vector2f::new(0.0, 0.0), 10.0);
	println!("{}", c);
    }
}


