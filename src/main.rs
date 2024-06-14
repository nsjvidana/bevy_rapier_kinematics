mod math_utils;
mod ik;
mod ik_systems;
mod arm;

use std::f32::consts::{FRAC_2_PI, FRAC_PI_2};
use crate::math_utils::{get_rot_axes, rotation_from_fwd, vec3_y};
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_rapier3d::plugin::{RapierContext, RapierPhysicsPlugin};
use bevy_rapier3d::prelude::{Collider, FixedJointBuilder, GenericJoint, GenericJointBuilder, ImpulseJoint, JointAxis, RapierMultibodyJointHandle, Real, RevoluteJointBuilder, RigidBody, Rot, Sleeping, Vect};
use bevy_rapier3d::render::{DebugRenderMode, RapierDebugRenderPlugin};
use std::ops::{Deref, Mul};
use bevy::DefaultPlugins;
use bevy::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::dynamics::{JointAxesMask, MultibodyJoint};
use bevy_rapier3d::na::{UnitQuaternion, Matrix3, Rotation3, UnitVector3, Vector, Vector3};
use crate::arm::ArmChain;
use crate::ik::JacobianIKArmBundle;
use crate::ik_systems::set_ik_arm_positions;

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
        // .add_systems(Startup, test_startup)
        // .add_systems(Update, test_update)
        .add_systems(Startup, test_startup2)
        .add_systems(PostUpdate, set_ik_arm_positions)
        ;

    let mut movement_settings = app.world.get_resource_mut::<bevy_flycam::MovementSettings>().unwrap();
    movement_settings.speed = 3.;


    app.run();
}

fn test_startup(
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
        FlyCam,
        Collider::ball(0.1)
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

    let fwd = UnitVector3::new_normalize(Vector3::new(1., 1., 1.));
    let rot = rotation_from_fwd(&fwd);

    //TODO: use generic joint for more freedom over joint control
    let r_shoulder_builder = GenericJointBuilder::new(JointAxesMask::LOCKED_SPHERICAL_AXES)
        .local_anchor1(vec3_y(torso_shape.half_length + torso_shape.radius))
        .local_anchor2(vec3_y(segment_shape.half_length + segment_shape.radius))
        .local_basis1(rot.into())
        .set_motor(JointAxis::AngX, -90_f32.to_radians(), 0.01_f32.to_radians(), 1., 0.01)
        .set_motor(JointAxis::AngY, 0., 0.01_f32.to_radians(), 1., 0.01)
        .set_motor(JointAxis::AngZ, 0., 0.01_f32.to_radians(), 1., 0.01);
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
    mut joint_q: Query<(&RapierMultibodyJointHandle, &mut MultibodyJoint)>,
    cam_q: Query<&Transform, With<FlyCam>>
) {
    let op = cam_q.get_single();
    if op.is_err() { return; }
    let cam_pos = Vector3::<f32>::from(op.unwrap().translation);

    let multibodies= &mut rapier_context.multibody_joints;
    for (mb_handle, mut joint_cmp) in joint_q.iter_mut() {

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


        let fwd = UnitVector3::new_normalize(cam_pos - joint_pos);
        let rot = UnitQuaternion::face_towards(&fwd, &Vector::y());
        let par_rot = link.local_to_world().rotation * link.local_to_parent().rotation.inverse();
        joint_cmp.data.set_local_basis1(par_rot.inverse().into());
        joint_cmp.data.set_local_basis2(par_rot.inverse().into());
        let (x, y, z) = rot.euler_angles();
        let mut stiff = 0.;
        let mut damp = 0.;
        if let Some(x_m) = joint_cmp.data.motor(JointAxis::AngX) {
            stiff = x_m.stiffness;
            damp = x_m.damping;
        }
        joint_cmp.data.set_motor_position(JointAxis::AngX, x - FRAC_PI_2, stiff, damp);
        if let Some(y_m) = joint_cmp.data.motor(JointAxis::AngY) {
            stiff = y_m.stiffness;
            damp = y_m.damping;
        }
        joint_cmp.data.set_motor_position(JointAxis::AngY, y, stiff, damp);
        if let Some(z_m) = joint_cmp.data.motor(JointAxis::AngZ) {
            stiff = z_m.stiffness;
            damp = z_m.damping;
        }
        joint_cmp.data.set_motor_position(JointAxis::AngZ, z, stiff, damp);
    }
}


fn test_update2(
    mut commands: Commands,
    rapier_context: Res<RapierContext>,
    mut joint_q: Query<(&RapierMultibodyJointHandle, &mut MultibodyJoint)>,
) {
    let multibodies= &rapier_context.multibody_joints;

    for (mb_handle, mut joint_cmp) in joint_q.iter_mut() {
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

        let par_rot = link.local_to_world().rotation * link.local_to_parent().rotation.inverse();
        joint_cmp.data.set_local_basis1(par_rot.inverse().into());
    }
}

fn test_startup2(
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
        FlyCam,
        Collider::ball(0.1)
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
            transform: Transform::from_xyz(0., -5., 0.),
            ..default()
        },
    ));

    let node_mesh = meshes.add(Sphere::new(0.05).mesh());

    let root = commands.spawn((
        RigidBody::Fixed,
        PbrBundle {
            mesh: node_mesh.clone(),
            material: materials.add(Color::BLUE),
            ..default()
        }
    )).id();
    let shoulder_x = commands.spawn((
        RigidBody::Dynamic,
        MultibodyJoint::new(root, RevoluteJointBuilder::new(Vec3::X))
    )).id();
    let shoulder_y = commands.spawn((
        RigidBody::Dynamic,
        MultibodyJoint::new(shoulder_x, RevoluteJointBuilder::new(Vec3::Y))
    )).id();
    let shoulder_z = commands.spawn((
        RigidBody::Dynamic,
        MultibodyJoint::new(shoulder_y, RevoluteJointBuilder::new(Vec3::Z))
    )).id();

    let radius = 0.05;

    let segment_shape = Capsule3d {
        half_length: 0.3 - radius,
        radius,
    };
    let upper_arm = commands.spawn((
        RigidBody::Dynamic,
        MultibodyJoint::new(shoulder_z, *FixedJointBuilder::new()
            .local_anchor2(vec3_y(segment_shape.half_length+radius))
            .build()
            .set_contacts_enabled(false)
        ),
        Collider::capsule_y(segment_shape.half_length, radius),
    )).id();

    let imp_j = commands.spawn((
        RigidBody::Dynamic,
        ImpulseJoint::new(upper_arm, FixedJointBuilder::new()
            .local_anchor1(vec3_y(-segment_shape.half_length-radius))),
    )).id();


    let elb_node_mat = materials.add(Color::GREEN);
    let elb_x = commands.spawn((
        RigidBody::Dynamic,
        MultibodyJoint::new(imp_j, RevoluteJointBuilder::new(Vec3::Y)),
        PbrBundle {
            mesh: node_mesh.clone(),
            material: elb_node_mat.clone(),
            ..default()
        },
    )).id();
    let elb_z = commands.spawn((
        RigidBody::Dynamic,
        MultibodyJoint::new(elb_x, RevoluteJointBuilder::new(Vec3::X)),
        PbrBundle {
            mesh: node_mesh.clone(),
            material: elb_node_mat.clone(),
            ..default()
        },
    )).id();

    let lower_arm = commands.spawn((
        RigidBody::Dynamic,
        MultibodyJoint::new(elb_z, *FixedJointBuilder::new()
            .local_anchor2(vec3_y(segment_shape.half_length+radius))
            .build()
            .set_contacts_enabled(false)
        ),
        Collider::capsule_y(segment_shape.half_length, radius),
    )).id();

    let arm = commands.spawn(JacobianIKArmBundle::<Real>::new(
        ArmChain::new_attached(
            root,
            [shoulder_x, shoulder_y, shoulder_z],
            upper_arm,
            [elb_x, elb_z],
            lower_arm
        ),
        None
    ));
}