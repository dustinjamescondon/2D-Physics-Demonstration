pub mod geometry;

use nalgebra::{Vector2, Vector3, Matrix3, Matrix2};

pub type Vector2f = Vector2<f32>;
pub type Vector3f = Vector3<f32>;
pub type Matrix3f = Matrix3<f32>;
pub type Matrix2f = Matrix2<f32>;

pub struct Transformation {
    pub translation: Vector2f,
    pub rotation: f32,
}

pub fn transform_vector(m: &Matrix3f, v: &Vector2f) -> Vector2f {
    let mut v_ext = Vector3f::new(v.x, v.y, 1.0); // set 3rd component to 1 so translation is applied
    v_ext = m * v_ext; // apply the transformation
    Vector2f::new(v_ext.x, v_ext.y)
}

pub fn transform_vector_rotation_only(m: &Matrix3f, v: &Vector2f) -> Vector2f {
    let mut v_ext = Vector3f::new(v.x, v.y, 0.0); // set 3rd component to 0 so translation is not applied
    v_ext = m * v_ext; // apply the transformation
    Vector2f::new(v_ext.x, v_ext.y)
}

pub fn maximum(a: f32, b: f32) -> f32 {
    if a > b { a } else { b }
}

pub fn zeroify_small_value(x: f32, epsilon: f32) -> f32 {
    if f32::abs(x) < epsilon { 0.0 } else { x }
}

impl Transformation {
    pub fn to_homogeneous(&self) -> Matrix3f {
	create_homogenous(self.translation, self.rotation)
    }
}

pub fn create_homogenous(offset: Vector2f, angle: f32) -> Matrix3f {

    let a_cos = angle.cos();
    let a_sin = angle.sin();

    Matrix3f::new(a_cos, a_sin, offset.x,
                  -a_sin, a_cos, offset.y,
                  0.0, 0.0, 1.0)
}

pub fn create_rotation(radians: f32) -> Matrix2f {
    let a_cos = radians.cos();
    let a_sin = radians.sin();
    Matrix2f::new(a_cos, a_sin,
		  -a_sin, a_cos)
}

pub fn rotate_about_point(point: &Vector2f, radians: f32, rotation_point: &Vector2f) -> Vector2f {
    let delta = point - rotation_point;
    let m = create_rotation(radians);
    (m * delta) + rotation_point
}

pub fn invert_homogenous(m: &Matrix3f) -> Matrix3f {
    
    let rot_t = Matrix2f::new(m.m11, m.m12, m.m21, m.m22).transpose();
    let m_trans = Vector2f::new(m.m13, m.m23);
    let m_inv_trans = -(rot_t * m_trans);
    Matrix3f::new(rot_t.m11, rot_t.m12, m_inv_trans.x,
		  rot_t.m21, rot_t.m22, m_inv_trans.y,
		  0.0,       0.0,       1.0)
}

/// NOTE: this is clockwise only with positive-y-down
pub fn perp_clockwise(v : &Vector2f) -> Vector2f {
    Vector2f::new(-v.y, v.x)
}

pub fn perp_counter_clockwise(v: &Vector2f) -> Vector2f {
    -perp_clockwise(v)
}

pub fn proj_onto(v : &Vector2f, unit_v : &Vector2f) -> Vector2f {
    v.dot(unit_v)*unit_v
}

pub fn clamp(val: f32, min:f32, max:f32) -> f32 {
    if val < min { min }
    else if val > max { max }
    else { val }
}
