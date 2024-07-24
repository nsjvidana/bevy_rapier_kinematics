mod math_utils;
mod ik;
mod arm;
mod physics;
mod testing;

use bevy_flycam::{FlyCam, MovementSettings, NoCameraPlayerPlugin};
use bevy_rapier3d::plugin::RapierPhysicsPlugin;
use bevy_rapier3d::prelude::{Collider, FixedJointBuilder, MultibodyJoint, RevoluteJointBuilder, RigidBody};
use bevy_rapier3d::render::RapierDebugRenderPlugin;
use bevy::math::Vec3;
use bevy::prelude::*;

fn main() {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<()>::default(),
        RapierDebugRenderPlugin::default(),
    ))
        .add_systems(Startup, startup);

    //flycam stuff
    app.add_plugins(NoCameraPlayerPlugin);
    app.world_mut().resource_mut::<MovementSettings>()
        .speed = 2.5;

    app.run();
}

pub fn startup(
    mut commands: Commands
) {
    //camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(2., 2., 2.)
                .looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        FlyCam,
        Collider::ball(0.1)
    ));

    let fixed = commands.spawn((
        RigidBody::Fixed,
        Transform::default()
    )).id();

    macro_rules! mb_joint {
        ($parent:expr, $joint:expr) => {
            (
                RigidBody::Dynamic,
                Transform::default(),
                MultibodyJoint::new($parent, $joint)
            )
        };
    }

    let half_height = 0.3;
    let radius = 0.05;
    let size = (half_height+radius) * 2.;
    let half_size = size/2.;

    let target_vel = 10f32.to_radians();
    let stiffness = 1.;
    let damping = 0.05;


    let shld_pitch = commands.spawn(mb_joint!(fixed, RevoluteJointBuilder::new(Vec3::X).into())).id();
    let shld_roll = commands.spawn(mb_joint!(shld_pitch, RevoluteJointBuilder::new(Vec3::Y).into())).id();
    let shld_yaw = commands.spawn(mb_joint!(shld_roll, RevoluteJointBuilder::new(Vec3::Z).into())).id();

    let upper_arm_collider = commands.spawn((
        RigidBody::Dynamic,
        Collider::capsule_y(half_height, radius),
        MultibodyJoint::new(shld_yaw, FixedJointBuilder::new().local_anchor2(Vec3::Y * half_size).into()),
    )).id();

    let elb_pitch = commands.spawn(mb_joint!(upper_arm_collider, RevoluteJointBuilder::new(Vec3::X)
        .local_anchor1(Vec3::Y * -size)
        .into()
    )).id();
    let elb_roll = commands.spawn(mb_joint!(elb_pitch, RevoluteJointBuilder::new(Vec3::Y).into()
    )).id();

    let lower_arm_collider = commands.spawn((
        RigidBody::Dynamic,
        Collider::capsule_y(half_height, radius),
        MultibodyJoint::new(elb_roll, FixedJointBuilder::new().local_anchor2(Vec3::Y * half_size).into()),
    ));
}
