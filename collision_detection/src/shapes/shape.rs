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
