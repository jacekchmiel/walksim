use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::ColliderDesc;
use nphysics3d::world::World;
use log::info;
use nalgebra::{ Vector3};

pub fn init_world() -> World<f32> {
    let mut world = World::new();
    world.set_gravity(Vector3::y() * -9.81);

    let ground_size = 50.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(ground_size, 2.0, ground_size)));

    ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -2.0)
        .build(&mut world);

    info!("World initialized");
    world
}

pub trait Simulate {
    fn simulation_step(&self, world: &mut World<f32>);
}

pub mod hexapod;
pub mod servo;

