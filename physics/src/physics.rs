
pub use crate::collision_hull::*;
pub use crate::math_util::*;
pub use crate::collision_detection::*;
use std::fmt;

const CONSTRAINT_SLOP: f32 = 0.05;
const CORRECTIVE_FACTOR: f32 = 2.0;
const GRAV_CONSTANT:f32 = 3.0e-1_f32;
const SOLVER_ITERATIONS: i32 = 7;

pub enum GravityType {
    Newton,
    Down,
}

/// The data structure for running a simulation
pub struct World {
    gravity_type: GravityType,
    pub contacts: Vec<Contact>,
    pub constraints: Vec<Constraint>,
}

impl World {
    /// Initializes a new world/simulation along with the gravity type
    ///
    /// # Example
    /// ``` let world = World::new(GravityType::Newton) ```

    pub fn new(gravity_type: GravityType) -> World {
	World {
	    gravity_type,
	    contacts: Vec::<Contact>::new(),
	    constraints: Vec::<Constraint>::new(),
	}
    }


    /// Returns a list of all the contact generated from the last call
    /// to [simulate]
    pub fn get_contacts(&self) -> &Vec<Contact> {
	&self.contacts
    }

    /// Returns the constraints generated from the last call to [simulate]
    pub fn get_constraints(&self) -> &Vec<Constraint> {
	&self.constraints
    }
}

/// The main physics object that is updated by [World::simulate]
#[derive(Clone)]
pub struct RigidBody {
    pub pos: Vector2f,
    pub vel: Vector2f,
    pub acc: Vector2f,
    pub rotation: f32,
    pub rotation_vel: f32,
    pub rotation_acc: f32,
    pub inv_mass: f32,
    pub inv_moi: f32,
    pub cor: f32,
    pub cof: f32,
    pub hull: CollisionHull,
}

impl Default for RigidBody {
    fn default() -> Self {
	Self::new()
    }
}
    
impl RigidBody {
    pub fn new() -> RigidBody {
	RigidBody {
	    pos: Vector2f::new(0.0,0.0),
	    vel: Vector2f::new(0.0,0.0),
	    acc: Vector2f::new(0.0,0.0),
            rotation:0.0,
            rotation_vel:0.0,
	    rotation_acc:0.0,
	    inv_mass: 1.0,
	    inv_moi: 1.0,
	    cor: 0.0,
	    cof: 0.0,
	    hull: CollisionHull::new(),
	    }
	}
    
    pub fn set_mass(&mut self, m:f32) -> &mut Self {
	if m == f32::INFINITY {
	    self.inv_mass = 0.0;
	}
	else {
	    self.inv_mass = 1.0 / m;
	}
	self
    }

    pub fn set_moi(&mut self, moi:f32) -> &mut Self {
	if moi == f32::INFINITY {
	    self.inv_moi = 0.0;
	}
	else {
	    self.inv_moi = 1.0 / moi;
	}
	self
    }

    pub fn add_fixture(&mut self, shape: CollisionShape) -> &mut Self {
	let fixture = CollisionFixture {
	    body: self,
	    aabb: AABB::from_collision_shape(&shape),
	    shape,
	};
	self.hull.local_fixtures.push(fixture.clone());
	self.hull.global_fixtures.push(fixture);
	self
    }

    /// Generates a series of [LineSegments] that form a closed loop
    /// based on the points argument and adds them to the RigidBody
    pub fn add_fixtures_from_line_loop(&mut self, points: &Vec<Vector2f>) -> &mut Self {
	for i in 0..points.len() {
	    let i_next = (i + 1) % points.len();
	    let line_seg = LineSegment::new(points[i_next], points[i]); // TODO debug
	    self.add_fixture(line_seg.to_shape());
	}
	self
    }

    pub fn apply_impulse(&mut self, impulse: &Vector2f, global_offset: &Vector2f) -> &mut Self {
	// TODO what was this for?
	//let projected_impulse = global_offset * global_offset.dot(impulse)/global_offset.dot(global_offset);
	let projected_impulse = impulse;

	self.vel += projected_impulse * self.inv_mass; 

	let ortho_impulse = perp_counter_clockwise(global_offset).dot(impulse);
	self.rotation_vel += ortho_impulse * self.inv_moi;
	self
    }

    pub fn apply_central_force(&mut self, force: Vector2f) -> &mut Self {
	self.acc += self.inv_mass * force;
	self
    }

    pub fn get_mass(&self) -> f32 {
	if self.inv_mass != 0.0 {
	    1.0 / self.inv_mass
	} else {
	    f32::INFINITY
	}
    }   
}

pub struct ConstraintPoint {
    depth: f32,
    velocity_bias: f32,
    contact_point: Vector2f,
    normal_impulse: f32,
    tangent_impulse: f32,
    normal_mass: f32,
    tangent_mass: f32,
}

impl fmt::Display for ConstraintPoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
	write!(f, "Depth: {}\nVel Bias: {}\nNormal impulse: {}\n Tangent impulse: {}\nNormal mass:{}\n Tangent mass:{}\n", self.depth, self.velocity_bias, self.normal_impulse, self.tangent_impulse, self.normal_mass, self.tangent_mass)
    }
}

pub struct Constraint {
    body_a: *mut RigidBody,
    body_b: *mut RigidBody,
    normal: Vector2f,
    tangent: Vector2f,
    points: Vec<ConstraintPoint>, // TODO make this an array of 2 later
}

impl fmt::Display for Constraint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
	let rslt = write!(f, "Normal: {}\n Tangent: {}\n", self.normal, self.tangent);
	self.points.iter().for_each(|point| write!(f, "{}", point).expect("Yell"));
	rslt
    }
}

impl World {
    /// Updates kinematics and dynamics for the bodies over the timestep of dt seconds
    pub fn simulate(&mut self, _time: f32, dt: f32, bodies: &mut Vec<&mut RigidBody>) {
	self.update_kinematics(bodies, dt);
	self.update_collision_hulls(bodies);
	self.solve_constraints(bodies);
	self.correct_constraints();
    }

    fn update_kinematics(&self, bodies: &mut Vec<&mut RigidBody>, dt: f32) {
	self.calc_accelerations(bodies);
	bodies.iter_mut().for_each(|body| {
	    body.pos += body.vel * dt;
	    body.vel += body.acc * dt;
	    body.rotation += body.rotation_vel * dt;
	    body.rotation_vel += body.rotation_acc * dt;
	});
    }

    /// Update the fixtures based on the current rigid body position and orientation
    fn update_collision_hulls(&self, bodies: &mut Vec<&mut RigidBody>) {
	for body in bodies {
	    body.hull.update_fixtures(create_homogenous(body.pos, body.rotation)
	    );
	}	
    }

    fn solve_constraints(&mut self, bodies: &mut Vec<&mut RigidBody>)
    {
	self.generate_contact_constraints(bodies);
	
	//--------------------------------------------------
	// First, figure out the target velocity in the normal direction
	// ..................................................
	for c in &mut self.constraints {
	    let a: &mut RigidBody;
	    let b: &mut RigidBody;
	    unsafe {
		a = &mut(*c.body_a);
		b = &mut(*c.body_b);
	    }
	    //--------------------------------------------------
	    // Warm start: Not implementing this yet, because I'm not sure how exactly
	    // to save the constraints after doing the integration. How do I map an old
	    // constraint to a new one?
	    // ..................................................
	    // ....
	    //--------------------------------------------------
	    
	    for p in &mut c.points {
		let coeff_of_restitution = maximum(a.cor, b.cor);
		//--------------------------------------------------
		// Calculate the offset of the contact point from the objects' position
		//..................................................
		let d_a = p.contact_point - a.pos;
		let d_b = p.contact_point - b.pos;
		
		let d_perp_a = perp_counter_clockwise(&d_a);
		let d_perp_b = perp_counter_clockwise(&d_b);
		//--------------------------------------------------

		// Linear velocity in the normal direction
		let linear_vel_a = a.vel;
		let linear_vel_b = b.vel;

		// Linear velocity in the normal direction  of the point as caused by rotational velocity
		let angular_vel_a = a.rotation_vel * d_perp_a;
		let angular_vel_b = b.rotation_vel * d_perp_b;

		let rel_vel_along_normal = (linear_vel_b + angular_vel_b - linear_vel_a - angular_vel_a).dot(&c.normal);

		// zeroifying small values here reduces jitter due to non-zero coefficient of restitutions
		p.velocity_bias = zeroify_small_value(rel_vel_along_normal * coeff_of_restitution, 1.0);

		//--------------------------------------------------
		// "Cold" starting, i.e. starting off the iterative
		// process with zeros
		//..................................................
		p.normal_impulse = 0.0;
		p.tangent_impulse = 0.0;
		//--------------------------------------------------
	    }
	}


	//--------------------------------------------------
	// Then solve the constraints
	// ..................................................
	for _ in 0..SOLVER_ITERATIONS {
	    for c in &mut self.constraints {
		let a : &mut RigidBody;
		let b : &mut RigidBody;

		unsafe {
		    a = &mut (*c.body_a);
		    b = &mut (*c.body_b);
		}
		
		for cp in &mut c.points {
		    //--------------------------------------------------
		    // Apply friction impulse
		    //..................................................

		    let mut lambda: f32;
		    //--------------------------------------------------
		    // Compute friction impulse
		    //..................................................
		    {
			//..................................................
			// Calculate the offset of the contact point from the objects' position
			//..................................................

			// The distance vector from object positions to the contact point
			let r_a_vec = cp.contact_point - a.pos;
			let r_b_vec = cp.contact_point - b.pos;

			let r_perp_a = perp_counter_clockwise(&r_a_vec).dot(&c.tangent);
			let r_perp_b = perp_counter_clockwise(&r_b_vec).dot(&c.tangent);
			//..................................................

			// Linear velocity in the normal direction
			let linear_vel_a = a.vel;
			let linear_vel_b = b.vel;

			// Linear velocity in the tangent direction  of the point as caused by rotational velocity
			let angular_vel_a = a.rotation_vel * perp_counter_clockwise(&r_a_vec);
			let angular_vel_b = b.rotation_vel * perp_counter_clockwise(&r_b_vec);

			let rel_vel_along_tangent = (linear_vel_b + angular_vel_b -
						     linear_vel_a - angular_vel_a).dot(&c.tangent);

			cp.tangent_mass = 1.0 / (a.inv_mass + b.inv_mass +
						 (r_perp_a * r_perp_a * a.inv_moi) +
						 (r_perp_b * r_perp_b * b.inv_moi));

			lambda = cp.tangent_mass * (-rel_vel_along_tangent);
		    }
		    //--------------------------------------------------

		    let coeff_of_friction = (a.cof * b.cof).sqrt();

		    let max_friction = (coeff_of_friction * cp.normal_impulse).abs();
		    let new_impulse = clamp(cp.tangent_impulse + lambda, -max_friction, max_friction);
		    lambda = new_impulse - cp.tangent_impulse;
		    cp.tangent_impulse = new_impulse;
		    let j =  lambda * c.tangent;
		    //cp.tangent_impulse = lambda; // Why not like this?
		    a.apply_impulse(&-j, &(cp.contact_point - a.pos));
		    b.apply_impulse(&j, &(cp.contact_point - b.pos));
		}

		//--------------------------------------------------
		// Apply normal impulse
		//..................................................
		for cp in &mut c.points {
		    let old_impulse: f32 = cp.normal_impulse;
		    let mut delta_impulse : f32;
		    {
			//--------------------------------------------------
			// Calculate the offset of the contact point from the objects' position
			//..................................................
			// The distance vector from object positions to the contact point
			let r_a_vec = cp.contact_point - a.pos;
			let r_b_vec = cp.contact_point - b.pos;

			// distance in the normal direction
			//float r_a = r_a_vec.dot(normal);
			//float r_b = r_b_vec.dot(normal);

			// TODO make sure this direction is correct for this application
			// distance in the tangent direction
			let r_perp_a = perp_counter_clockwise(&r_a_vec).dot(&c.normal);
			let r_perp_b = perp_counter_clockwise(&r_b_vec).dot(&c.normal);
			//--------------------------------------------------

			// Linear velocity in the normal direction
			let linear_vel_a = a.vel;
			let linear_vel_b = b.vel;

			// Linear velocity in the normal direction  of the point as caused by rotational velocity
			let angular_vel_a = a.rotation_vel * perp_counter_clockwise(&r_a_vec);
			let angular_vel_b = b.rotation_vel * perp_counter_clockwise(&r_b_vec);

			let rel_vel_along_normal = (linear_vel_b + angular_vel_b -
						    linear_vel_a - angular_vel_a).dot(&c.normal);

			let target_vel = cp.velocity_bias + rel_vel_along_normal;

			cp.normal_mass = 1.0 /(a.inv_mass + b.inv_mass +
					       r_perp_a * r_perp_a * a.inv_moi +
					       r_perp_b * r_perp_b * b.inv_moi);
			delta_impulse = cp.normal_mass * target_vel;
		    }

		    cp.normal_impulse += delta_impulse;
		    if cp.normal_impulse < 0.0 {
			cp.normal_impulse = 0.0;
		    }
		    delta_impulse = cp.normal_impulse - old_impulse;
		    let j = delta_impulse * c.normal;
		    a.apply_impulse(&j, &(cp.contact_point -  a.pos));
		    b.apply_impulse(&-j, &(cp.contact_point - b.pos));
		}
	    }
	}
	
	// TODO
	// report_impulses();
    }

    fn correct_constraints(&mut self) {
	for c in &mut self.constraints {
	    let body_a: &mut RigidBody;
	    let body_b: &mut RigidBody;
	    unsafe {
		body_a = &mut(*c.body_a);
		body_b = &mut(*c.body_b);
	    }
	    for cp in &mut c.points {
		// if the constraint is past the allowable amount of slop then correct it
		if cp.depth > CONSTRAINT_SLOP {
		    let j = CORRECTIVE_FACTOR * cp.depth * c.normal;
		    body_a.apply_impulse(&j, &(cp.contact_point -  body_a.pos));
		    body_b.apply_impulse(&-j, &(cp.contact_point - body_b.pos));
		}
	    }
	}
    }
    
    fn calc_accelerations(&self, bodies: &mut Vec<&mut RigidBody>) {
	// First zero out the existing accelerations
	bodies.iter_mut().for_each(|body| {
	    body.acc = Vector2f::zeros();
	    body.rotation_acc = 0.0;
	});

	match &self.gravity_type {
	    /*--------------------------------------------------
	     * Calculate Newtonian gravity between all bodies
	     *..................................................*/
	    GravityType::Newton => {
		for i in 0..bodies.len()-1 {
		    for j in i+1..bodies.len() {
			let grav_force = calc_grav_between(
			    bodies[i].pos, bodies[j].pos,
			    bodies[i].get_mass(), bodies[j].get_mass());

			(bodies[i]).apply_central_force(grav_force);
			(bodies[j]).apply_central_force(-grav_force);
		    }
		}
	    }
	    /*--------------------------------------------------
	     * Apply a downward (positive y) gravitational force
	     *..................................................*/
	    GravityType::Down => {
		for body in bodies {
		    if body.inv_mass != 0.0 {
			body.acc = Vector2f::new(0.0, 9.81);
		    }
		}
	    }
	}
    }

    /// Tests all the fixtures of this rigid body to those of the other rigid body
    fn generate_contact_constraints(&mut self, bodies: &mut Vec<&mut RigidBody>) {
	self.constraints = Vec::<Constraint>::new();

	if bodies.len() <= 1 {
	    return;
	}

	// Clear out the other contacts
	self.contacts = Vec::<Contact>::new();
	
	for i in 0..bodies.len() - 1 {
	    for j in (i + 1)..bodies.len() {

		let mut contacts = test_collision_between_hulls(&bodies[i].hull, &bodies[j].hull);
		// Now translate the contact to a constraint
		for contact in &contacts {
		    let mut constraint = make_contact_into_constraint(contact);
		    constraint.body_a = &mut(*bodies[i]) as *mut RigidBody;
		    constraint.body_b = &mut(*bodies[j]) as *mut RigidBody;
		    self.constraints.push(constraint);
		}
		self.contacts.append(&mut contacts);
	    }
	}
    }
}

pub fn calc_grav_at_point(point: Vector2f, bodies: &Vec<&RigidBody>) -> Vector2f {
    let mut grav = Vector2f::zeros();
    for body in bodies {
	grav += calc_grav_between(point, body.pos, 1.0, body.get_mass());
    }
    grav
}

/// Calculates the graviational force between two objects by their
/// positions and masses
fn calc_grav_between(pos_a: Vector2f, pos_b: Vector2f, mass_a: f32, mass_b: f32) -> Vector2f {
    // TODO could simplify calculation maybe
    let normal = (pos_b - pos_a).normalize();
    let mag = GRAV_CONSTANT * mass_a  * mass_b / (pos_b - pos_a).norm_squared();
    mag * normal
}

fn make_contact_into_constraint(contact: &Contact) -> Constraint {
    let mut cps = Vec::<ConstraintPoint>::new();
    for i in 0..contact.points.len() {
	cps.push( ConstraintPoint {
	    depth: contact.points[i].depth,
	    normal_impulse: 0.0,
	    tangent_impulse: 0.0,
	    contact_point: contact.points[i].point,
	    normal_mass: 0.0,
	    tangent_mass: 0.0,
	    velocity_bias: 0.0,
	});
    }
    Constraint {
	body_a: std::ptr::null_mut::<RigidBody>(),
	body_b: std::ptr::null_mut::<RigidBody>(),
	normal: contact.normal,
	tangent: perp_clockwise(&contact.normal),
	points: cps, 
    }
}

