use core::cell::Cell;
use core::f32::consts::PI;
use nphysics3d::joint::RevoluteJoint;
use std::cmp::min;
use nphysics3d::object::BodyHandle;
use nphysics3d::world::World;
use super::Simulate;

const TWO_PI: f32 = PI * 2.0;

#[derive(Debug)]
pub struct ServoModel {
    base_handle: BodyHandle,
    link_id: usize,

    target: Cell<f32>,
    origin: f32,
}

impl ServoModel {
    pub fn new(base_handle: BodyHandle, link_id: usize, origin: f32) -> Self {
        ServoModel {
            base_handle,
            link_id,

            target: Cell::new(0.0),
            origin
        }
    }

    pub fn set_target(&self, v: f32) {
        self.target.set(v);
    }

    pub fn with_target(mut self, v: f32) -> Self {
        self.set_target(v);

        self
    }
}

impl Simulate for ServoModel {
    fn simulation_step(&self, world: &mut World<f32>) {
        const K_TQ: f32 = 25.0;
        const K_SPEED: f32 = 4.0;
        const MAX_TQ: f32 = 100.0;
        const MAX_SPEED: f32 = TWO_PI; // 60 rpm
        const DEADZONE: f32 = 0.01;

        let base = world.multibody_mut(self.base_handle).unwrap();
        let joint = base
            .link_mut(self.link_id)
            .unwrap()
            .joint_mut()
            .downcast_mut::<RevoluteJoint<f32>>()
            .unwrap();

        let target = self.target.get();
        let raw_angle_diff = (target + self.origin) - joint.angle();

        // normalize angle diff
        let mut norm_angle_diff = raw_angle_diff;
        while norm_angle_diff < 0.0 {
            norm_angle_diff += TWO_PI;
        }
        while norm_angle_diff > TWO_PI {
            norm_angle_diff -= TWO_PI;
        }


        // flip direction if diff is over half rotation
        let mut angle_diff = if norm_angle_diff > PI {
            norm_angle_diff - TWO_PI
        } else {
            norm_angle_diff
        };

        //deadzone
        if norm_angle_diff.abs() < DEADZONE {
            angle_diff = 0.0;
        }

        joint.set_desired_angular_motor_velocity(MAX_SPEED.min(angle_diff)*K_SPEED);
        joint.set_max_angular_motor_torque(MAX_TQ.min(angle_diff.abs()*K_TQ));
        joint.enable_angular_motor();
    }
}