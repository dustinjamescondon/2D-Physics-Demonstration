extern crate math_util;
extern crate collision_detection;

use math_util::*;
use collision_detection::*;

use sdl2::{pixels::Color, gfx::primitives::DrawRenderer};
use sdl2::render::Canvas;
use sdl2::video::Window;

pub fn render_aabb(canvas: &mut Canvas<Window>, aabb: &AABB) {
    canvas.set_draw_color(Color::RGBA(0, 255, 0, 0));
    let half_w: f32 = aabb.width / 2.0;
    let half_h: f32 = aabb.height / 2.0;
    
    canvas.draw_line(
	((aabb.x - half_w) as i32, (aabb.y - half_h) as i32),
	((aabb.x + half_w) as i32, (aabb.y - half_h) as i32),
    ).expect("Couldn't draw line");
    canvas.draw_line(
	((aabb.x + half_w) as i32, (aabb.y - half_h) as i32),
	((aabb.x + half_w) as i32, (aabb.y + half_h) as i32),
    ).expect("Couldn't draw line");
    canvas.draw_line(
	((aabb.x + half_w) as i32, (aabb.y + half_h) as i32),
	((aabb.x - half_w) as i32, (aabb.y + half_h) as i32),
    ).expect("Couldn't draw line");
    canvas.draw_line(
	((aabb.x - half_w) as i32, (aabb.y + half_h) as i32),
	((aabb.x - half_w) as i32, (aabb.y - half_h) as i32),
    ).expect("Couldn't draw line");
}

pub fn render_shape(canvas: &mut Canvas<Window>, shape: &CollisionShape, trans: &Matrix3f) {
    match shape {
	CollisionShape::Circle(c_) => {
	    let c = c_.transform(trans);
	    canvas.circle(c.pos.x as i16, c.pos.y as i16,
			  c.radius as i16, Color::RGB(0,255,0)).expect("Couldn't draw!")
	},
	CollisionShape::LineSegment(l_) => {
	    let l = l_.transform(trans);
	    canvas.set_draw_color(Color::BLUE);
	    let point1 = l.point1;
	    let point2 = l.point2;
	    canvas.draw_line(
		(point1.x as i32, point1.y as i32),
		(point2.x as i32, point2.y as i32)).expect("Couldn't draw line!");

	    let endpt_of_normal = l.midpoint() + 10.0 * l.normal;
	    canvas.draw_line(
		(l.midpoint().x as i32, l.midpoint().y as i32),
		(endpt_of_normal.x as i32, endpt_of_normal.y as i32)).expect("Yell");
	},
	CollisionShape::ConvexPoly(p_) => {
	    let p = p_.transform(trans);
	    canvas.set_draw_color(Color::BLUE);
	    p.edges.iter().for_each(|edge| {
		let point1 = edge.point1 + p.pos;
		let point2 = edge.point2 + p.pos;

		{ // Draw the normal
		    let line_seg = LineSegment::new(point1, point2);
		    let point1 = line_seg.midpoint();
		    let point2 = line_seg.midpoint() + line_seg.normal * 10.0;
		    canvas.draw_line(
			(point1.x as i32, point1.y as i32),
			(point2.x as i32, point2.y as i32)).expect("Couldn't draw line!");
		}

		{ // Draw the edge
		    canvas.set_draw_color(Color::RED);
		    canvas.draw_line(
			(point1.x as i32, point1.y as i32),
			(point2.x as i32, point2.y as i32)).expect("Couldn't draw line!")
		}
	    });
	    
	},
    }
}
