//! This sets up a basic scene with static walls, ramps, and dynamic
//! boxes and balls to demo the collision detection and physics system


extern crate physics;
extern crate render_utils;
extern crate math_util;

use math_util::*;
use render_utils::*;
use physics::physics::*;
use sdl2::{pixels::Color, gfx::primitives::DrawRenderer};
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::render::Canvas;
use sdl2::video::Window;
use std::time::Duration;

const WINDOW_W: u32 = 1200;
const WINDOW_H: u32 = 800;

fn render_bodies(bodies: &Vec<RigidBody>, canvas: &mut Canvas<Window>) {
    for body in bodies {
	canvas.circle(body.pos.x as i16, body.pos.y as i16, 10.0 as i16, Color::RED).expect("Yell");
	for fixture in &body.hull.global_fixtures {
	    //render_aabb(canvas, &fixture.aabb);
	    render_shape(canvas, &fixture.shape, &Matrix3f::identity());
	}
    }
}

fn render_contacts(canvas: &mut Canvas<Window>, contacts: &Vec<Contact>) {
    for contact in contacts {
	render_contact(canvas, contact);
    }
    
    fn render_contact(canvas: &mut Canvas<Window>, contact: &Contact) {
        let contact_pt_radius = 6i16;
        for point in &contact.points {
	    let end_pt = point.point + 10.0 * contact.normal;
	    canvas.draw_line((point.point.x as i32, point.point.y as i32),
	    (end_pt.x as i32, end_pt.y as i32)).expect("Couldn't draw!");
            canvas.circle(point.point.x as i16, point.point.y as i16,
			  contact_pt_radius, Color::RGB(0,255,0)).expect("Couldn't draw!");
        }
    }
}

fn main() {
    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
    let mut world = World::new(GravityType::Down);
    let mut rigid_bodies = vec![
	//--------------------------------------------------
	// Create dynamic boxes/balls
	//..................................................
	create_box((WINDOW_W / 2) as f32, (WINDOW_H / 2) as f32),
	create_ball((WINDOW_W / 2) as f32, (WINDOW_H / 2) as f32 - 100.0),
	create_box((WINDOW_W / 2) as f32, (WINDOW_H / 2) as f32 - 200.0),
	create_box((WINDOW_W / 2) as f32, (WINDOW_H / 2) as f32 - 300.0),
	create_ball((WINDOW_W / 2) as f32 + 100.0, (WINDOW_H / 2) as f32),
	create_box((WINDOW_W / 2) as f32 + 100.0, (WINDOW_H / 2) as f32 - 100.0),
	create_box((WINDOW_W / 2) as f32 + 100.0, (WINDOW_H / 2) as f32 - 200.0),
	create_box((WINDOW_W / 2) as f32 + 100.0, (WINDOW_H / 2) as f32 - 300.0),
	create_ball(150.0, 70.0),
	//--------------------------------------------------
	// Create static walls/ramps
	//..................................................
	create_ramp(100.0, 100.0, 500.0, 500.0),
	create_wall(Direction::UP, 50.0, WINDOW_W as f32, WINDOW_H as f32),
	create_wall(Direction::DOWN, 50.0, WINDOW_W as f32, WINDOW_H as f32),
	create_wall(Direction::LEFT, 50.0, WINDOW_W as f32, WINDOW_H as f32),
	create_wall(Direction::RIGHT, 50.0, WINDOW_W as f32, WINDOW_H as f32)];
    
    let window = video_subsystem.window("Physics system demo", WINDOW_W, WINDOW_H)
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().build().unwrap();

    canvas.set_draw_color(Color::RGB(0, 255, 255));
    canvas.clear();
    let mut event_pump = sdl_context.event_pump().unwrap();
    let mut i = 0;
    'running: loop {
        i = (i + 1) % 255;
        canvas.set_draw_color(Color::RGB(0, 0, 0));
        canvas.clear();
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} |
                Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    break 'running
                },
                Event::MouseMotion { timestamp, window_id, which, mousestate, x, y, xrel, yrel } => {
                },
                Event::MouseButtonDown { timestamp, window_id, which, mouse_btn, clicks, x, y } => {
                },
		Event::MouseButtonUp { timestamp, window_id, which, mouse_btn, clicks, x, y } => {
		},
                _ => {}
            }
        }

	let mut rbs = Vec::<&mut RigidBody>::new();
	rigid_bodies.iter_mut().for_each(|body| rbs.push(body));
	
        world.simulate(0.0, 0.05, &mut rbs);
        render_bodies(&rigid_bodies, &mut canvas);
	render_contacts(&mut canvas, world.get_contacts());
        // The rest of the game loop goes here...

        canvas.present();
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
    }
}

fn create_box(x: f32, y: f32) -> RigidBody {
    let mut rb = RigidBody::new();
    rb.cor = 0.4;
    rb.cof = 0.8;
    rb.pos = Vector2f::new(x, y);
    rb.rotation = 0.8;
    rb.set_mass(2.0);
    rb.set_moi(1000.0);
    rb.add_fixture(
	ConvexPoly::new_box(50.0, 50.0).to_shape(),
    );
    
    rb
}

fn create_ball(x: f32, y: f32) -> RigidBody {
    let mut rb = RigidBody::new();
    rb.cor = 0.4;
    rb.cof = 0.8;
    rb.pos = Vector2f::new(x, y);
    rb.rotation = 0.8;
    rb.set_mass(5.0);
    rb.set_moi(1000.0);
    rb.add_fixture(
	Circle::new(Vector2f::zeros(), 25.0).to_shape(),
    );
    
    rb
}

fn create_ramp(x0: f32, y0: f32, x1: f32, y1: f32) -> RigidBody {
    let mut rb = RigidBody::new();
    rb.inv_mass = 0.0;
    rb.inv_moi = 0.0;
    rb.cor = 0.0;
    rb.cof = 0.5;

    // make the rb pos the midpoint of the line
    rb.pos = Vector2f::new((x1 + x0) / 2.0, (y1 + y0) /2.0);
    rb.add_fixture(
	LineSegment::new(Vector2f::new(x0, y0) - rb.pos, Vector2f::new(x1, y1) - rb.pos).to_shape()
    );
    
    rb
}

// Helper function for creating walls
enum Direction { UP, DOWN, LEFT, RIGHT }
fn create_wall(dir: Direction, thickness: f32, room_w: f32, room_h: f32) -> RigidBody {
    let mut rb = RigidBody::new();
    rb.inv_mass = 0.0;
    rb.inv_moi = 0.0;
    rb.cor = 0.0;
    rb.cof = 0.5;

    let half_room_h = room_h / 2.0;
    let half_room_w = room_w / 2.0;

    match dir {
	Direction::UP => {
	    rb.pos = Vector2f::new(half_room_w, thickness / 2.0);
	    rb.add_fixture(
		ConvexPoly::new_box(room_w, thickness).to_shape()
	    );
	},
	Direction::DOWN => {
	    rb.pos = Vector2f::new(half_room_w, room_h - (thickness / 2.0));
	    rb.add_fixture(
		ConvexPoly::new_box(room_w, thickness).to_shape()
	    );
	},
	Direction::LEFT => {
	    rb.pos = Vector2f::new(thickness / 2.0, half_room_h);
	    rb.add_fixture(
		ConvexPoly::new_box(thickness, room_h - (2.0 * thickness) - 1.0).to_shape()
	    );
	},
	Direction::RIGHT => {
	    rb.pos = Vector2f::new(room_w - (thickness / 2.0), half_room_h);
	    rb.add_fixture(
		ConvexPoly::new_box(thickness, room_h - (2.0 * thickness) - 1.0).to_shape()
	    );
	},
    }

    rb
}

