use nphysics3d::joint::RevoluteJoint;
use nphysics3d::object::BodyHandle;
use nphysics3d::world::World;
use super::Simulate;

#[derive(Debug)]
pub struct ServoModel {
    base_handle: BodyHandle,
    link_id: usize,
}

impl ServoModel {
    pub fn new(world: &World<f32>, base_handle: BodyHandle, link_id: usize) -> Self {
        ServoModel {
            base_handle,
            link_id,
        }
    }
}

impl Simulate for ServoModel {
    fn simulation_step(&self, world: &mut World<f32>) {
        let base = world.multibody_mut(self.base_handle).unwrap();
        let joint = base
            .link_mut(self.link_id)
            .unwrap()
            .joint_mut()
            .downcast_mut::<RevoluteJoint<f32>>()
            .unwrap();
        joint.set_desired_angular_motor_velocity(0.1);
        joint.set_max_angular_motor_torque(100.0);
        joint.enable_angular_motor();
    }
}