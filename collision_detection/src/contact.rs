pub use crate::math_util::*;

pub struct ContactPoint {
    pub point: Vector2f,
    pub depth: f32,
}

pub struct Contact {
    pub normal : Vector2f,
    pub points : Vec<ContactPoint>,
}

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
