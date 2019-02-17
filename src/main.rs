use core::f32::consts::PI;
use log::info;
use nalgebra::{Point3};
use nphysics_testbed3d::Testbed;

const TWO_PI: f32 = 2.0 * PI;


mod logger;
mod panic_hook;

mod physics;

use physics::Simulate;

// pub struct Servo {
//     target: f32,
//     joint: Rc<RevoluteJoint<f32>>,
// }

// impl Servo {
//     pub fn new(joint: Rc<RevoluteJoint<f32>>) -> Servo {
//         joint.enable_angular_motor();
//         Servo {
//             target: 0.0,
//             joint
//         }
//     }

//     pub fn step(&mut self) {
//         self.joint.set_desired_angular_motor_velocity(self.target - self.joint.angle());
//     }
// }

// pub struct HexapodMultibody {
// }

// impl HexapodMultibody {
//     pub fn set_v(&self, world: &mut World<f32>, leg: usize, joint: usize) {
//         let mb = world.multibody_mut(self.body_handle.0).unwrap();
//         let joint = mb
//             .link_mut(self.legs[leg][joint].1)
//             .unwrap()
//             .joint_mut()
//             .downcast_mut::<RevoluteJoint<f32>>()
//             .unwrap();
//         joint.set_desired_angular_motor_velocity(0.1);
//         joint.set_max_angular_motor_torque(100.0);
//         joint.enable_angular_motor();
//     }
// }

use physics::{init_world, HexapodModel, HexapodGeometry};

fn main() {
    // console!(log, "Initializing simulation...");
    // panic_hook::set_once();
    logger::init();
    info!("Initializing simulation...");

    let mut world = init_world();
    let hexapod = HexapodModel::create(&HexapodGeometry::default(), &mut world);

    let mut testbed = Testbed::new_empty();


    testbed.set_body_color(
        hexapod.body(),
        Point3::new(255.0 / 255.0, 96.0 / 255.0, 33.0 / 255.0),
    );

    testbed.set_world(world);
    testbed.look_at(Point3::new(-10.0, 4.0, -10.0), Point3::new(0.0, 0.0, 0.0));
    testbed.add_callback(move |world, _, _| {
        let mut world = world.get_mut();
        hexapod.simulation_step(world.deref_mut());
    });

    info!("Starting testbed...");
    testbed.run();
}
