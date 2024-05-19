mod math_utils;

pub use bevy_rapier3d::na as nalgebra;

use crate::math_utils::{FRAC_PI_12, rotation_from_fwd, vec3_y};
use bevy::prelude::*;
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_rapier3d::plugin::{RapierContext, RapierPhysicsPlugin};
use bevy_rapier3d::prelude::{Collider, FixedJoint, ImpulseJoint, JointAxis, RapierMultibodyJointHandle, RigidBody, Sleeping, SphericalJointBuilder};
use bevy_rapier3d::render::{DebugRenderMode, RapierDebugRenderPlugin};
use std::ops::Mul;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::dynamics::MultibodyJoint;
use bevy_rapier3d::na::{Matrix4, UnitQuaternion};
use crate::nalgebra::{Isometry, Translation, UnitVector3, Vector, Vector3};

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
            WorldInspectorPlugin::default()
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
            material: materials.add(Color::rgb(0.95, 0.95, 0.95)),
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
    let torso_cmd = commands.spawn((
        RigidBody::Fixed,
        PbrBundle {
            mesh: torso_mesh,
            material,
            transform: Transform::default(),
            ..default()
        },
        Collider::capsule_y(torso_shape.half_length, torso_shape.radius),
        bevy::core::Name::new("Torso"),
    ));
    let torso = torso_cmd.id();

    //TODO: use generic joint for more freedom over joint control
    let r_shoulder_builder = SphericalJointBuilder::new()
        .local_anchor1(vec3_y(torso_shape.half_length + torso_shape.radius))
        .local_anchor2(vec3_y(segment_shape.half_length + segment_shape.radius))
        .motor(JointAxis::AngX, 90_f32.to_radians(), 5_f32.to_radians() , 1., 0.0)
        .motor(JointAxis::AngY, 0., 5_f32.to_radians() , 1., 0.0)
        .motor(JointAxis::AngZ, 0., 5_f32.to_radians() , 1., 0.0);
        // .limits(JointAxis::AngZ, [FRAC_PI_12, PI-FRAC_PI_12]);
    let mut r_shoulder = MultibodyJoint::new(torso, r_shoulder_builder);
    r_shoulder.data.set_contacts_enabled(false);
    let r_upper_cmd = commands.spawn((
        RigidBody::Dynamic,
        Collider::capsule_y(segment_shape.half_length, segment_shape.radius),
        Transform::default(),
        r_shoulder,
        bevy::core::Name::new("Upper"),
        Sleeping::disabled()
    ));
    let r_upper = r_upper_cmd.id();
}

fn test_update(
    mut rapier_context: ResMut<RapierContext>,
    mut gizmos: Gizmos,
    mut joint_q: Query<(&RapierMultibodyJointHandle, &mut MultibodyJoint, &Transform)>,
    transform_q: Query<&Transform>,
    cam_q: Query<&Transform, With<FlyCam>>
) {
    let op = cam_q.get_single();
    if op.is_err() { return; }
    let cam_pos = Vector3::<f32>::from(op.unwrap().translation);

    let multibodies= &mut rapier_context.multibody_joints;
    for (mb_handle, mut joint_cmp, transform) in joint_q.iter_mut() {

        let mb = multibodies.get(mb_handle.0).unwrap();
        if mb.0.num_links() == 0 { continue; }
        let mut link = mb.0.link(0).unwrap();
        let mut found_link = false;
        for l in mb.0.links() {
            if l.joint.data.as_spherical().is_some() {
                link = l;
                found_link = true;
                break;
            }
        }
        if !found_link { continue; }

        let pos = link.local_to_world().translation.vector;
        let rapier_joint = &link.joint.data;
        let joint_pos = link.local_to_world().rotation.mul(rapier_joint.local_frame2.translation.vector) + pos;

        //link.local_to_world().rotation.inverse().mul(link.local_to_world() * joint.local_axis2()).into()

        let fwd = UnitVector3::new_normalize(cam_pos - joint_pos);
        let rot = rotation_from_fwd(&fwd);
        let axis = rot.axis().unwrap();

        let par_transform = transform_q.get(joint_cmp.parent).unwrap();
        let axis_vec3: Vec3 = axis.into();
        joint_cmp.data.set_local_axis1(par_transform.rotation.inverse().mul(axis_vec3));

        let mut stiffness= 0.;
        let mut damping = 0.;
        {
            let motor = joint_cmp.data.motor(JointAxis::AngX).unwrap();
            stiffness = motor.stiffness;
            damping = motor.damping;
        }
        joint_cmp.data.set_motor_position(JointAxis::AngX, rot.angle(), stiffness, damping);
        {
            let motor = joint_cmp.data.motor(JointAxis::AngY).unwrap();
            stiffness = motor.stiffness;
            damping = motor.damping;
        }
        joint_cmp.data.set_motor_position(JointAxis::AngY, 0., stiffness, damping);
        {
            let motor = joint_cmp.data.motor(JointAxis::AngZ).unwrap();
            stiffness = motor.stiffness;
            damping = motor.damping;
        }
        joint_cmp.data.set_motor_position(JointAxis::AngZ, 0., stiffness, damping);


        gizmos.ray(
            joint_pos.into(),
            axis.into(),
            Color::WHITE
        );

    }
}
