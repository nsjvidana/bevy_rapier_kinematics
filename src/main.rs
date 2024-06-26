mod math_utils;
mod ik;
mod ik_systems;
mod arm;
mod physics;
mod k_chain;
mod testing;

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

    let shld_pitch = commands.spawn(mb_joint!(fixed, RevoluteJointBuilder::new(Vec3::X))).id();
    let shld_roll = commands.spawn(mb_joint!(shld_pitch, RevoluteJointBuilder::new(Vec3::Y))).id();
    let shld_yaw = commands.spawn(mb_joint!(shld_roll, RevoluteJointBuilder::new(Vec3::Z))).id();

    let upper_arm_collider = commands.spawn((
        RigidBody::Dynamic,
        Collider::capsule_y(half_height, radius),
        MultibodyJoint::new(shld_yaw, FixedJointBuilder::new().local_anchor2(Vec3::Y * half_size))
    )).id();

    //removing this code prevents the crash
    {
        let elbow_pitch = commands.spawn(mb_joint!(shld_yaw, RevoluteJointBuilder::new(Vec3::X)
            .local_anchor1(Vec3::Y * -size)
        )).id();
    }
}
