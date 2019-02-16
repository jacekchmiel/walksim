use std::rc::Rc;
use core::f32::consts::PI;
use nalgebra::{Point3, Vector3};
use nalgebra::geometry::Isometry3;
use ncollide3d::shape::{ConvexHull, Cuboid, Cylinder, ShapeHandle};
use nphysics3d::joint::{RevoluteJoint, FreeJoint};
use nphysics3d::object::{MultibodyDesc, ColliderDesc, RigidBodyDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;
use nphysics3d::object::BodyPartHandle;

const TWO_PI: f32 = 2.0 * PI;

const BODY_HEIGHT: f32 = 4.0;
const BODY_RADIUS: f32 = 12.0;

const LEG_R: f32 = 1.0;
const LEG_COXA: f32 = 4.0;
const LEG_FEMUR: f32 = 4.0;
const LEG_TIBIA: f32 = 10.0;
const LEG_COLLIDER_MARGIN: f32 = 0.35;


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

struct Hexapod {
    body_r: f32,
    body_h: f32,
    coxa_len: f32,
    femur_len: f32,
    tibia_len: f32,
}

impl Hexapod {
    pub fn new() -> Self {
        Hexapod {
            body_r: BODY_RADIUS,
            body_h: BODY_HEIGHT,
            coxa_len: LEG_COXA,
            femur_len: LEG_FEMUR,
            tibia_len: LEG_TIBIA,
        }
    }

    pub fn leg_origin(&self, idx: usize) -> Vector3<f32> {
        let angle = (idx as f32) / 6.0 * TWO_PI;
        Vector3::new(
            self.body_r * angle.cos(),
            0.0,
            self.body_r * angle.sin()
        )
    }

    pub fn base_leg_angle(&self, idx: usize) -> f32 {
        (idx as f32) / 6.0 * TWO_PI
    }

    pub fn make_step_callback(self) -> impl FnMut() -> () {
        || {

        }
    }
}

fn make_leg_collider(len: f32) -> ColliderDesc<f32> {
    let shape = ShapeHandle::new(Cuboid::new(Vector3::new(len/2.0 - LEG_COLLIDER_MARGIN * 2.0, LEG_R/2.0, LEG_R/2.0)));
    ColliderDesc::new(shape).density(1.0)
}

fn make_body(world: &mut World<f32>, testbed: &mut Testbed) -> BodyPartHandle {
    let hexapod = Hexapod::new();
    let angles: Vec<_> = (0..6).map(|v| (v as f32) / 6.0 * TWO_PI).collect();
    let points_low = angles.iter().map(|a| {
        Point3::new(
            BODY_RADIUS * a.cos(),
            -BODY_HEIGHT / 2.0,
            BODY_RADIUS * a.sin(),
        )
    });
    let points_high = angles.iter().map(|a| {
        Point3::new(
            BODY_RADIUS * a.cos(),
            BODY_HEIGHT / 2.0,
            BODY_RADIUS * a.sin(),
        )
    });
    let points: Vec<_> = points_low.chain(points_high).collect();
    let shape = ShapeHandle::new(ConvexHull::try_from_points(&points).expect("Body hull computation failed."));

    let body_collider = ColliderDesc::new(shape).density(1.0);
    let body_joint = FreeJoint::new(Isometry3::translation(0.0, BODY_HEIGHT, 0.0));
    let mut multibody_desc = MultibodyDesc::new(body_joint)
        .parent_shift(Vector3::y() * BODY_HEIGHT)
        .collider(&body_collider);

    let coxa_collider = make_leg_collider(LEG_COXA);
    let femur_collider = make_leg_collider(LEG_FEMUR);
    let tibia_collider = make_leg_collider(LEG_TIBIA);

    for i in 0..6 {
        let body_joint = RevoluteJoint::new(Vector3::y_axis(), -hexapod.base_leg_angle(i));
        let coxa_mb = multibody_desc.add_child(body_joint)
            .set_body_shift(Vector3::x() * LEG_COXA/2.0)
            .set_parent_shift(-hexapod.leg_origin(i))
            .add_collider(&coxa_collider);

        let coxa_joint = RevoluteJoint::new(Vector3::z_axis(), 0.0);
        let femur_mb = coxa_mb.add_child(coxa_joint)
            .set_body_shift(Vector3::x() * LEG_FEMUR/2.0)
            .set_parent_shift(-Vector3::x() * LEG_COXA/2.0)
            .add_collider(&femur_collider);

        let mut tibia_joint = RevoluteJoint::new(Vector3::z_axis(), 0.0);
        tibia_joint.set_desired_angular_motor_velocity(1.0);
        tibia_joint.set_max_angular_motor_torque(10.0);
        tibia_joint.enable_angular_motor();
        femur_mb.add_child(tibia_joint)
            .set_body_shift(Vector3::x() * LEG_TIBIA/2.0)
            .set_parent_shift(-Vector3::x() * LEG_FEMUR/2.0)
            .add_collider(&tibia_collider);
    }

    multibody_desc.build(world).root().part_handle()
}

fn main() {
    let mut testbed = Testbed::new_empty();

    let mut world = World::new();
    world.set_gravity(Vector3::y() * -9.81);

    let ground_size = 50.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(ground_size, 0.1, ground_size)));

    ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -1.0)
        .build(&mut world);

    let hexapod_handle = make_body(&mut world, &mut testbed);
    testbed.set_body_color(hexapod_handle.0, Point3::new(255.0/255.0,96.0/255.0, 33.0/255.0));
    // let num = 6; // We create 6 × 6 × 6 boxes.
    // let rad = 0.1;

    // First create a collider builder that will be reused by every rigid body.
    // let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    // let collider_desc = ColliderDesc::new(cuboid).density(1.0);

    // // Then a rigid body builder to which we give the collider builder. That
    // // way, each time this rigid body builder will be used to `.build(&mut world)`,
    // // the corresponding collider will be created as well.
    // let mut rb_desc = RigidBodyDesc::new().collider(&collider_desc);

    // let shift = (rad + collider_desc.get_margin()) * 2.0;
    // let centerx = shift * (num as f32) / 2.0;
    // let centery = shift / 2.0;
    // let centerz = shift * (num as f32) / 2.0;
    // let height = 3.0;

    // for i in 0usize..num {
    //     for j in 0usize..num {
    //         for k in 0usize..num {
    //             let x = i as f32 * shift - centerx;
    //             let y = j as f32 * shift + centery + height;
    //             let z = k as f32 * shift - centerz;

    //             // Build the rigid body and its collider.
    //             // Note that we re-use the same RigidBodyDesc every time after
    //             // changing its position.
    //             rb_desc
    //                 .set_translation(Vector3::new(x, y, z))
    //                 .build(&mut world);
    //         }
    //     }
    // }

    testbed.set_world(world);
    testbed.look_at(Point3::new(-100.0, 40.0, -100.0), Point3::new(0.0, 0.0, 0.0));
    testbed.add_callback(move |world, _, _| {
        let mut world = world.get_mut();
        let link = world.multibody_mut(hexapod_handle.0).unwrap();
    });
    testbed.run();
}
