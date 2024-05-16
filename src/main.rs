mod math_utils;

use crate::math_utils::{FRAC_PI_12, vec3_y};
use bevy::math::primitives;
use bevy::prelude::*;
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin, PlayerPlugin};
use bevy_rapier3d::parry::shape::ShapeType::Cuboid;
use bevy_rapier3d::plugin::{PhysicsSet, RapierContext, RapierPhysicsPlugin};
use bevy_rapier3d::prelude::{Collider, FixedJoint, GenericJointBuilder, ImpulseJoint, JointAxesMask, JointAxis, MotorModel, RapierMultibodyJointHandle, RigidBody, SphericalJointBuilder};
use bevy_rapier3d::render::{DebugRenderMode, RapierDebugRenderPlugin};
use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, FRAC_PI_6, FRAC_PI_8, PI, TAU};
use std::ops::Mul;
use bevy_rapier3d::dynamics::MultibodyJoint;
use bevy_rapier3d::na::Vector;

fn main() {
    let mut app = App::new();
    app.add_plugins((
            DefaultPlugins,
            NoCameraPlayerPlugin,
            RapierPhysicsPlugin::<()>::default(),
            RapierDebugRenderPlugin {
                style: default(),
                mode: {
                    let mut mode = DebugRenderMode::default();
                    mode.set(DebugRenderMode::JOINTS, true);
                    mode
                },
                enabled: true,
            },
        ))
        .add_systems(Startup, test_startup)
        .add_systems(Update, test_update);

    let mut movement_settings = app.world.get_resource_mut::<bevy_flycam::MovementSettings>().unwrap();
    movement_settings.speed = 3.;


    app.run();
}

fn test_startup (
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    //camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0., 0., 5.)
                .looking_at(Vec3::NEG_Z, Vec3::Y),
            ..default()
        },
        FlyCam
    ));

    //spawn ground
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(10., 0.1, 10.),
        PbrBundle {
            mesh: meshes.add(bevy::prelude::Cuboid {
                half_size: Vec3::new(10., 0.1, 10.),
            }),
            material: materials.add(Color::rgb(128., 128., 128.)),
            transform: Transform::from_xyz(0., -1., 0.),
            ..default()
        },
    ));

    let radius = 0.05;

    let segment_shape = Capsule3d {
        half_length: 0.3 - radius,
        radius,
    };
    let torso_shape = Capsule3d {
        half_length: 0.5 - radius,
        radius,
    };
    let segment_mesh = meshes.add(segment_shape);
    let torso_mesh = meshes.add(torso_shape);

    let material = materials.add(Color::rgb(1., 0., 0.));
    let fixed_entity = commands.spawn((
        RigidBody::Dynamic,
        Transform::default(),
    )).id();
    let torso_cmd = commands.spawn((
        RigidBody::Dynamic,
        PbrBundle {
            mesh: torso_mesh,
            material,
            transform: Transform::default(),
            ..default()
        },
        ImpulseJoint::new(fixed_entity, FixedJoint::default()),
        Collider::capsule_y(torso_shape.half_length, torso_shape.radius),
    ));
    let torso = torso_cmd.id();

    let joint_builder = GenericJointBuilder::new(JointAxesMask::LOCKED_SPHERICAL_AXES);

    //TODO: use generic joint for more freedom over joint control
    let r_shoulder_builder = SphericalJointBuilder::new()
        .local_anchor1(vec3_y(torso_shape.half_length + torso_shape.radius))
        .local_anchor2(vec3_y(segment_shape.half_length + segment_shape.radius))
        .motor(JointAxis::AngZ, FRAC_PI_2, PI/10., 0.5, 0.05)
        .limits(JointAxis::AngZ, [FRAC_PI_12, PI-FRAC_PI_12]);
    let mut r_shoulder = MultibodyJoint::new(torso, r_shoulder_builder);
    r_shoulder.data.set_contacts_enabled(false);
    let r_upper_cmd = commands.spawn((
        RigidBody::Dynamic,
        Collider::capsule_y(segment_shape.half_length, segment_shape.radius),
        Transform::default(),
        r_shoulder,
    ));
    let r_upper = r_upper_cmd.id();
}

fn test_update(
    mut rapier_context: ResMut<RapierContext>,
    mut gizmos: Gizmos,
    query: Query<(&RapierMultibodyJointHandle)>
) {
    let mb_joints = &mut rapier_context.multibody_joints;
    for (handle) in query.iter() {
        let joint = mb_joints.get_mut(handle.0).unwrap().0;
        if joint.num_links() == 0 { continue; }
        let joint_world_mat = joint.link(0).unwrap().local_to_world();
        let rot = joint_world_mat.rotation;
        let fwd = rot.mul(Vector::new(0., 0., 1.));
        gizmos.ray(
            Vec3::new(joint_world_mat.translation.x, joint_world_mat.translation.y, joint_world_mat.translation.z),
            fwd,
            Color::rgb(1., 1., 1.)
        );
    }
}
