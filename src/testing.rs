use core::panic;

use bevy::{app::{App, PostUpdate}, math::Vec3, prelude::{default, Camera3dBundle, Commands, Component, Entity, IntoSystemConfigs, Local, Query, Res}, transform::components::Transform, DefaultPlugins};
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::{dynamics::{MultibodyJoint, RevoluteJointBuilder, RigidBody, Sleeping, SphericalJointBuilder}, geometry::Collider, math::Real, na::Isometry3, plugin::{PhysicsSet, RapierContext, RapierPhysicsPlugin}, render::{DebugRenderMode, RapierDebugRenderPlugin}};
use k::{InverseKinematicsSolver, SerialChain};

use crate::{math_utils::vec3_y, physics::{toggle_contacts_with, ToggleContactsWith}};

pub fn thing(
    mut commands: Commands
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

    //ground
    

    let _target_vel = 10_f32.to_radians();
    let _stiffness = 1.;
    let _damping = 0.05;

    let _fixed = commands.spawn((
        RigidBody::Fixed,
        Sleeping::default(),
        Transform::default()
    )).id();

    let mut last_body = commands.spawn(RigidBody::Fixed).id();
    let half_length = 0.1;
    let radius = 0.05;
    let size = (half_length + radius)*2.;
    let axes = vec![Vec3::X, Vec3::Y, Vec3::Z];
    for i in 0..10 {
        let revolute = *RevoluteJointBuilder::new(axes[i%3])
            .local_anchor1(vec3_y(size / 2. * (i != 0) as usize as f32))
            .local_anchor2(vec3_y(-size / 2.))
            .build()
            .set_contacts_enabled(false);
        let spherical = *SphericalJointBuilder::new()
            .local_anchor1(vec3_y(size / 2. * (i != 0) as usize as f32))
            .local_anchor2(vec3_y(-size / 2.))
            .build()
            .set_contacts_enabled(false);

        let mb_joint = if i%2 == 0 {
            MultibodyJoint::new(last_body, revolute.into())
        }
        else {
            MultibodyJoint::new(last_body, revolute.into())
        };

        last_body = commands.spawn((
            RigidBody::Dynamic,
            Collider::capsule_y(half_length, radius),
            // Sensor,
            mb_joint
        )).id();
    }
}

fn update(
    mut arm_q: Query<&Arm>,
    mut first_frame_check: Local<bool>,
    rapier_context: Res<RapierContext>
) {
    if !first_frame_check.clone() { *first_frame_check = true; return; }
    
    let arm_result = arm_q.get_single_mut();
    if arm_result.is_err() { return; }

    let arm = arm_result.unwrap();

    let [
        fixed,
        shoulder_x_entity,
        _shoulder_y_entity,
        _shoulder_z_entity,
        elbow_pitch_entity,
        _elbow_roll_entity,
        wrist_pitch_entity,
        _wrist_yaw_entity
    ] = arm.entities;
    let (shoulder_mb, shld_x_id) = rapier_context.multibody_joints.get(
        *rapier_context.entity2multibody_joint().get(&shoulder_x_entity).unwrap()
    ).unwrap();
    let (elbow_mb, elb_pitch_id) = rapier_context.multibody_joints.get(
        *rapier_context.entity2multibody_joint().get(&elbow_pitch_entity).unwrap()
    ).unwrap();
    let (wrist_mb, wrist_pitch_id) = rapier_context.multibody_joints.get(
        *rapier_context.entity2multibody_joint().get(&wrist_pitch_entity).unwrap()
    ).unwrap();

    let fixed_body = rapier_context.bodies.get(*rapier_context.entity2body().get(&fixed).unwrap()).unwrap();
    let fixed_transform =  Isometry3 {
        rotation: fixed_body.rotation().clone(),
        translation: fixed_body.translation().clone().into()
    };
    let shld_transforms = [
        shoulder_mb.link(shld_x_id).unwrap().local_to_parent(), //x
        shoulder_mb.link(shld_x_id+1).unwrap().local_to_parent(),//y
        shoulder_mb.link(shld_x_id+2).unwrap().local_to_parent()//z
    ];

    let elb_transforms = [
        elbow_mb.link(elb_pitch_id).unwrap().local_to_parent(), //pitch
        elbow_mb.link(elb_pitch_id+1).unwrap().local_to_parent() //roll
    ];

    let wrist_transforms = [
        wrist_mb.link(wrist_pitch_id).unwrap().local_to_parent(), //pitch
        wrist_mb.link(wrist_pitch_id+1).unwrap().local_to_parent() //yaw
    ];

    let mut shld_index = 0;
    let mut elb_index = 0;
    let mut wrist_index = 0;
    for node in arm.chain.iter() {
        let joint = node.joint();

        #[allow(unused)]
        if !matches!(joint.joint_type, k::JointType::Rotational { axis }) {
            continue;
        }
        
        if joint.name.starts_with("shld") { //shoulder rotations
            //PROBLEM: setting origin would mean having to also zero the joint position, which is not right.
            node.set_joint_position_unchecked(shld_transforms[shld_index].rotation.angle());
            shld_index += 1;
        }
        else if joint.name.starts_with("elb") { //elbow rotations
            node.set_joint_position_unchecked(elb_transforms[elb_index].rotation.angle());
            elb_index += 1;
        }
        else if joint.name.starts_with("wrist") {
            node.set_joint_position_unchecked(wrist_transforms[wrist_index].rotation.angle());
            wrist_index += 1;
        }
        else { //fixed (has to be root node)
            node.set_origin(unsafe { std::mem::transmute(fixed_transform) });
        }
    }
}

fn create_arm<T: k::RealField + k::SubsetOf<f64>>() -> SerialChain<T> {
    use k::*;
    let fixed: Node<T> = NodeBuilder::new()
        .name("fixed")
        .joint_type(JointType::Fixed)
        .finalize().into();
    let l0: Node<T> = NodeBuilder::new()
        .name("shld_x")
        .joint_type(JointType::Rotational {axis: Vector3::x_axis()})
        .finalize().into();
    let l1: Node<T> = NodeBuilder::new()
        .name("shld_y")
        .joint_type(JointType::Rotational {axis: Vector3::y_axis()})
        .finalize().into();
    let l2: Node<T> = NodeBuilder::new()
        .name("shld_z")
        .joint_type(JointType::Rotational {axis: Vector3::z_axis()})
        .finalize().into();

    let l3: Node<T> = NodeBuilder::new()
        .name("elb_pitch")
        .joint_type(JointType::Rotational {axis: Vector3::x_axis()})
        .finalize().into();
    let l4: Node<T> = NodeBuilder::new()
        .name("elb_roll")
        .joint_type(JointType::Rotational {axis: Vector3::y_axis()})
        .finalize().into();

    let l5: Node<T> = NodeBuilder::new()
        .name("wrist_pitch")
        .joint_type(JointType::Rotational {axis: Vector3::x_axis()})
        .finalize().into();
    let l6: Node<T> = NodeBuilder::new()
        .name("wrist_yaw")
        .joint_type(JointType::Rotational {axis: Vector3::z_axis()})
        .finalize().into();
    connect![fixed => l0 => l1 => l2 => l3 => l4 => l5 => l6];

    return SerialChain::new_unchecked(Chain::from_root(fixed));
}

#[derive(Component)]
pub struct Arm {
    pub entities: [Entity; 8],
    pub chain: SerialChain<Real>
}

pub fn test() {

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
        .add_systems(PostUpdate, toggle_contacts_with.after(PhysicsSet::SyncBackend))
        ;


    let mut movement_settings = app.world_mut().get_resource_mut::<bevy_flycam::MovementSettings>().unwrap();
    movement_settings.speed = 3.;

    app.run();
}

