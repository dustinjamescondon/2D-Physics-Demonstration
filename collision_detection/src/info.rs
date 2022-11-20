pub use crate::math_util::*;

/// Contains the information about how much two object are overlapping
/// when projected along an axis taken from the normal of the edge
pub struct PenetrationInfo {
    pub depth: f32,
    pub edge: Edge,
}

impl PenetrationInfo {
    pub fn new(depth: f32, edge: &Edge) -> Self{
	PenetrationInfo {
	    depth,
	    edge: edge.clone()
	}
    }
}

pub struct ContactPoint {
    pub point: Vector2f,
    pub depth: f32,
}

pub struct Contact {
    pub normal : Vector2f,
    pub points : Vec<ContactPoint>,
}


/// Modifies the contact by reversing the normal
pub fn flip_contact_normal(mut contact: Option<Contact>) -> Option<Contact>{
    if contact.is_some() {
	contact.as_mut().unwrap().normal *= -1.0
    }
    contact
}

impl Default for Contact {
    fn default() -> Contact {
	Contact {
	    normal: Vector2f::new(1.0, 0.0),
	    points: vec![],
	}
    }
}
