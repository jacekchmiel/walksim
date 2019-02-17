use ncollide3d::shape::{ConvexHull, Cuboid, ShapeHandle};
use nphysics3d::joint::{FreeJoint, RevoluteJoint};
use nphysics3d::object::{BodyPartHandle,BodyHandle};
use nphysics3d::object::{ColliderDesc, MultibodyDesc};
use nphysics3d::world::World;
use core::f32::consts::PI;
use nalgebra::geometry::Isometry3;
use nalgebra::{Point3, Vector3};
use super::servo::ServoModel;
use super::Simulate;

const DEFAULT_BODY_HEIGHT: f32 = 0.4;
const DEFAULT_BODY_RADIUS: f32 = 1.2;

const DEFAULT_LEG_RADIUS: f32 = 0.1;
const DEFAULT_LEG_COXA: f32 = 0.5;
const DEFAULT_LEG_FEMUR: f32 = 0.5;
const DEFAULT_LEG_TIBIA: f32 = 1.0;

const TWO_PI: f32 = PI * 2.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Leg {
    LeftFront,
    LeftMiddle,
    LeftBack,
    RightFront,
    RightMiddle,
    RightBack,
}

impl Leg {
    pub fn as_usize(&self) -> usize {
        match self {
            Leg::LeftFront => 0,
            Leg::LeftMiddle => 1,
            Leg::LeftBack => 2,
            Leg::RightBack => 3,
            Leg::RightMiddle => 4,
            Leg::RightFront => 5,
        }
    }

    pub fn from_usize(n: usize) -> Option<Self> {
        match n {
            0 => Some(Leg::LeftFront),
            1 => Some(Leg::LeftMiddle),
            2 => Some(Leg::LeftBack),
            3 => Some(Leg::RightBack),
            4 => Some(Leg::RightMiddle),
            5 => Some(Leg::RightFront),
            _ => None
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LegSegment {
    Coxa,
    Femur,
    Tibia,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HexapodGeometry {
    pub body_r: f32,
    pub body_h: f32,
    pub coxa_len: f32,
    pub femur_len: f32,
    pub tibia_len: f32,
    pub leg_r: f32,
}

impl HexapodGeometry {
    pub fn new() -> Self {
        HexapodGeometry {
            body_r: DEFAULT_BODY_RADIUS,
            body_h: DEFAULT_BODY_HEIGHT,
            coxa_len: DEFAULT_LEG_COXA,
            femur_len: DEFAULT_LEG_FEMUR,
            tibia_len: DEFAULT_LEG_TIBIA,
            leg_r: DEFAULT_LEG_RADIUS,
        }
    }
    pub fn builder() -> HexapodGeometryBuilder {
        HexapodGeometryBuilder::new()
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

    pub fn segment_len(&self, s: LegSegment) -> f32 {
        match s {
            LegSegment::Coxa => self.coxa_len,
            LegSegment::Femur => self.femur_len,
            LegSegment::Tibia => self.tibia_len,
        }
    }
}

impl Default for HexapodGeometry {
    fn default() -> Self {
        HexapodGeometry::new()
    }
}

#[derive(Clone, Copy, Default, PartialEq)]
pub struct HexapodGeometryBuilder(HexapodGeometry);

impl HexapodGeometryBuilder {
    pub fn new() -> Self {
        HexapodGeometryBuilder(
            HexapodGeometry::default()
        )
    }

    pub fn body_radius(mut self, v: f32) -> Self {
        self.0.body_r = v;

        self
    }

    pub fn body_height(mut self, v: f32) -> Self {
        self.0.body_h = v;

        self
    }

    pub fn coxa_len(mut self, v: f32) -> Self {
        self.0.coxa_len = v;

        self
    }

    pub fn femur_len(mut self, v: f32) -> Self {
        self.0.femur_len = v;

        self
    }

    pub fn tibia_len(mut self, v: f32) -> Self {
        self.0.tibia_len = v;

        self
    }

    pub fn build(self) -> HexapodGeometry {
        self.0
    }
}

pub struct HexapodModel {
    body_handle: BodyPartHandle,
    legs: Vec<Vec<BodyPartHandle>>,
    servos: Vec<Vec<ServoModel>>,
}

impl HexapodModel {
    pub fn create(hexapod: &HexapodGeometry, world: &mut World<f32>) -> Self {
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

        let coxa_collider = Self::make_leg_collider(hexapod, LegSegment::Coxa);
        let femur_collider = Self::make_leg_collider(hexapod, LegSegment::Femur);
        let tibia_collider = Self::make_leg_collider(hexapod, LegSegment::Tibia);

        let mut legs = Vec::new();
        let mut servos = Vec::new();
        for i in 0..6 {
            let coxa_joint = RevoluteJoint::new(Vector3::y_axis(), -hexapod.base_leg_angle(i));
            let coxa_handle = MultibodyDesc::new(coxa_joint)
                .set_body_shift(Vector3::x() * hexapod.coxa_len / 2.0)
                .set_parent_shift(-hexapod.leg_origin(i))
                .add_collider(&coxa_collider)
                .build_with_parent(body_handle, world)
                .unwrap()
                .part_handle();

            let femur_joint = RevoluteJoint::new(Vector3::z_axis(), 0.0);
            let femur_handle = MultibodyDesc::new(femur_joint)
                .set_body_shift(Vector3::x() * hexapod.femur_len / 2.0)
                .set_parent_shift(-Vector3::x() * hexapod.coxa_len / 2.0)
                .add_collider(&femur_collider)
                .build_with_parent(coxa_handle, world)
                .unwrap()
                .part_handle();

            let tibia_joint = RevoluteJoint::new(Vector3::z_axis(), 0.0);
            let tibia_handle = MultibodyDesc::new(tibia_joint)
                .set_body_shift(Vector3::x() * hexapod.tibia_len / 2.0)
                .set_parent_shift(-Vector3::x() * hexapod.femur_len / 2.0)
                .add_collider(&tibia_collider)
                .build_with_parent(femur_handle, world)
                .unwrap()
                .part_handle();

            legs.push(vec![coxa_handle, femur_handle, tibia_handle]);
            let c = ServoModel::new(body_handle.0, coxa_handle.1, -hexapod.base_leg_angle(i));
            let f = ServoModel::new(body_handle.0, femur_handle.1, 0.0).with_target(-3.14 / 4.0);
            let t = ServoModel::new(body_handle.0, tibia_handle.1, 0.0).with_target(3.14 * 0.75);
            servos.push(vec![c, f, t]);
        }

        HexapodModel { body_handle, legs, servos }
    }

    fn make_leg_collider(h: &HexapodGeometry, segment: LegSegment) -> ColliderDesc<f32> {
        // Margin required so leg segments will not collide with each other
        let margin = h.leg_r * 1.5;

        let len = h.segment_len(segment);

        let shape = ShapeHandle::new(Cuboid::new(Vector3::new(
            len / 2.0 - margin,
            h.leg_r / 2.0,
            h.leg_r / 2.0,
        )));
        ColliderDesc::new(shape).density(1.0)
    }

    pub fn body(&self) -> BodyHandle {
        self.body_handle.0
    }

    pub fn servo(&self, leg: Leg, segment: LegSegment) -> &ServoModel {
        let s = match segment {
            LegSegment::Coxa => 0,
            LegSegment::Femur => 1,
            LegSegment::Tibia => 2,
        };
        &self.servos[leg.as_usize()][s]
    }

    pub fn servo_mut(&mut self, leg: Leg, segment: LegSegment) -> &mut ServoModel {
        let s = match segment {
            LegSegment::Coxa => 0,
            LegSegment::Femur => 1,
            LegSegment::Tibia => 2,
        };
        &mut self.servos[leg.as_usize()][s]
    }
}

impl Simulate for HexapodModel {
    fn simulation_step(&self, world: &mut World<f32>) {
        for i in 0..6 {
            self.servo(Leg::from_usize(i).unwrap(), LegSegment::Coxa).simulation_step(world);
            self.servo(Leg::from_usize(i).unwrap(), LegSegment::Femur).simulation_step(world);
            self.servo(Leg::from_usize(i).unwrap(), LegSegment::Tibia).simulation_step(world);
        }
    }

}