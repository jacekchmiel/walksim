use core::f32::consts::PI;
use nalgebra::geometry::Isometry3;
use nalgebra::{Point3, Vector3};
use ncollide3d::shape::{ConvexHull, Cuboid, ShapeHandle};
use nphysics3d::joint::{FreeJoint, RevoluteJoint};
use nphysics3d::object::BodyPartHandle;
use nphysics3d::object::{ColliderDesc, MultibodyDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;

const TWO_PI: f32 = 2.0 * PI;

const BODY_HEIGHT: f32 = 0.4;
const BODY_RADIUS: f32 = 1.2;

const LEG_R: f32 = 0.1;
const LEG_COXA: f32 = 0.5;
const LEG_FEMUR: f32 = 0.5;
const LEG_TIBIA: f32 = 1.0;
const LEG_COLLIDER_MARGIN: f32 = 0.035;

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

struct HexapodGeometry {
    body_r: f32,
    body_h: f32,
    coxa_len: f32,
    femur_len: f32,
    tibia_len: f32,
}

impl HexapodGeometry {
    pub fn new() -> Self {
        HexapodGeometry {
            body_r: BODY_RADIUS,
            body_h: BODY_HEIGHT,
            coxa_len: LEG_COXA,
            femur_len: LEG_FEMUR,
            tibia_len: LEG_TIBIA,
        }
    }

    pub fn leg_origin(&self, idx: usize) -> Vector3<f32> {
        let angle = (idx as f32) / 6.0 * TWO_PI;
        Vector3::new(self.body_r * angle.cos(), 0.0, self.body_r * angle.sin())
    }

    pub fn base_leg_angle(&self, idx: usize) -> f32 {
        (idx as f32) / 6.0 * TWO_PI
    }

    pub fn body_points(&self) -> Vec<Point3<f32>> {
        let angles: Vec<_> = (0..6).map(|v| (v as f32) / 6.0 * TWO_PI).collect();
        let points_low = angles.iter().map(|a| {
            Point3::new(
                self.body_r * a.cos(),
                -self.body_h / 2.0,
                self.body_r * a.sin(),
            )
        });
        let points_high = angles.iter().map(|a| {
            Point3::new(
                self.body_r * a.cos(),
                self.body_h / 2.0,
                self.body_r * a.sin(),
            )
        });
        points_low.chain(points_high).collect()
    }
}

pub struct HexapodMultibody {
    pub body_handle: BodyPartHandle,
    pub legs: Vec<Vec<BodyPartHandle>>,
}

impl HexapodMultibody {
    pub fn set_v(&self, world: &mut World<f32>, leg: usize, joint: usize) {
        let mb = world.multibody_mut(self.body_handle.0).unwrap();
        let joint = mb
            .link_mut(self.legs[leg][joint].1)
            .unwrap()
            .joint_mut()
            .downcast_mut::<RevoluteJoint<f32>>()
            .unwrap();
        joint.set_desired_angular_motor_velocity(0.1);
        joint.set_max_angular_motor_torque(100.0);
        joint.enable_angular_motor();
    }
}

fn make_leg_collider(len: f32) -> ColliderDesc<f32> {
    let shape = ShapeHandle::new(Cuboid::new(Vector3::new(
        len / 2.0 - LEG_COLLIDER_MARGIN * 2.0,
        LEG_R / 2.0,
        LEG_R / 2.0,
    )));
    ColliderDesc::new(shape).density(1.0)
}

fn make_body(world: &mut World<f32>) -> HexapodMultibody {
    let hexapod = HexapodGeometry::new();
    let shape = ShapeHandle::new(
        ConvexHull::try_from_points(hexapod.body_points().as_slice())
            .expect("Body hull computation failed."),
    );

    let body_collider = ColliderDesc::new(shape).density(1.0);
    let free_joint = FreeJoint::new(Isometry3::translation(0.0, hexapod.femur_len * 2.0, 0.0));
    let body_handle = MultibodyDesc::new(free_joint)
        .parent_shift(Vector3::y() * hexapod.body_h)
        .collider(&body_collider)
        .build(world)
        .root()
        .part_handle();

    let coxa_collider = make_leg_collider(hexapod.coxa_len);
    let femur_collider = make_leg_collider(hexapod.femur_len);
    let tibia_collider = make_leg_collider(hexapod.tibia_len);

    let mut legs = Vec::new();
    for i in 0..6 {
        let coxa_joint = RevoluteJoint::new(Vector3::y_axis(), -hexapod.base_leg_angle(i));
        let coxa_handle = MultibodyDesc::new(coxa_joint)
            .set_body_shift(Vector3::x() * LEG_COXA / 2.0)
            .set_parent_shift(-hexapod.leg_origin(i))
            .add_collider(&coxa_collider)
            .build_with_parent(body_handle, world)
            .unwrap()
            .part_handle();

        let femur_joint = RevoluteJoint::new(Vector3::z_axis(), 0.0);
        let femur_handle = MultibodyDesc::new(femur_joint)
            .set_body_shift(Vector3::x() * LEG_FEMUR / 2.0)
            .set_parent_shift(-Vector3::x() * LEG_COXA / 2.0)
            .add_collider(&femur_collider)
            .build_with_parent(coxa_handle, world)
            .unwrap()
            .part_handle();

        let tibia_joint = RevoluteJoint::new(Vector3::z_axis(), 0.0);
        let tibia_handle = MultibodyDesc::new(tibia_joint)
            .set_body_shift(Vector3::x() * LEG_TIBIA / 2.0)
            .set_parent_shift(-Vector3::x() * LEG_FEMUR / 2.0)
            .add_collider(&tibia_collider)
            .build_with_parent(femur_handle, world)
            .unwrap()
            .part_handle();

        legs.push(vec![coxa_handle, femur_handle, tibia_handle]);
    }

    HexapodMultibody { body_handle, legs }
}

fn main() {
    let mut testbed = Testbed::new_empty();

    let mut world = World::new();
    world.set_gravity(Vector3::y() * -9.81);

    let ground_size = 50.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(ground_size, 2.0, ground_size)));

    ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -2.0)
        .build(&mut world);

    let hexapod = make_body(&mut world);
    testbed.set_body_color(
        hexapod.body_handle.0,
        Point3::new(255.0 / 255.0, 96.0 / 255.0, 33.0 / 255.0),
    );
    testbed.set_world(world);
    testbed.look_at(Point3::new(-10.0, 4.0, -10.0), Point3::new(0.0, 0.0, 0.0));
    testbed.add_callback(move |world, _, _| {
        let mut world = world.get_mut();
        for i in 0..6 {
            for j in 0..3 {
                hexapod.set_v(world.deref_mut(), i, j);
            }
        }
    });
    testbed.run();
}
