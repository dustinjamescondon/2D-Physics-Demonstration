//! A collision hull represents a collection of collision shapes

use crate::RigidBody;
pub use collision_detection::{CollisionShape, Contact, Shape, AABB};
pub use math_util::*;

/// Represents a CollisionShape that is attached to a RigidBody's
/// CollisionHull
#[derive(Clone)]
pub struct CollisionFixture {
    pub shape: CollisionShape,
    pub aabb: AABB,
    // the rigid body that owns this fixture (can just make this a
    // void pointer instead)
    pub body: *mut RigidBody
}

/// Maintains all the collision fixtures attached to a RigidBody
#[derive(Clone, Default)]
pub struct CollisionHull {
    // fixtures that remain untransformed with the body position and
    // orientation
    pub local_fixtures: Vec<CollisionFixture>,
    // fixtures that we transformed with the body position and
    // orientation
    pub global_fixtures: Vec<CollisionFixture>,
}

impl CollisionHull {
    pub fn new() -> CollisionHull {
	CollisionHull::default()
    }

    // transforms the local_fixtures into global_fixtures based on the
    // current position and orientation of the body
    pub fn update_fixtures(&mut self, local_to_global: Matrix3f) {
	for i in 0..self.local_fixtures.len() {
	    self.global_fixtures[i].shape = self.local_fixtures[i].shape.transform(&local_to_global);
	    self.global_fixtures[i].aabb = AABB::from_collision_shape(&self.global_fixtures[i].shape);
	}
    }
}

/// Tests all the fixtures of collision hull "a" against those in "b",
/// performing an AABB collision culling test first
pub fn test_collision_between_hulls(hull_a:& CollisionHull,
				    hull_b:& CollisionHull) -> Vec::<Contact> {
    let mut contacts = Vec::<Contact>::new();

    for i in 0..hull_a.global_fixtures.len() {
	for j in 0..hull_b.global_fixtures.len() {
	    let global_fixture_a = &hull_a.global_fixtures[i];
	    let global_fixture_b = &hull_b.global_fixtures[j];

	    // Only perform more precise collision detection routine
	    // if the aabb's are overlapping
	    if global_fixture_a.aabb.overlapping(&global_fixture_b.aabb) {
		if let Some(c) = global_fixture_a.shape.test_against(&global_fixture_b.shape) {
		    contacts.push(c);
		}
	    }
	}
    }
    contacts
}
