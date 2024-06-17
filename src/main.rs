mod math_utils;
mod ik;
mod ik_systems;
mod arm;
mod physics;

use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_rapier3d::plugin::{RapierContext, RapierPhysicsPlugin};
use bevy_rapier3d::prelude::{Collider, FixedJointBuilder, ImpulseJoint, MultibodyJoint, PhysicsSet, Real, RevoluteJointBuilder, RigidBody, Sleeping, SphericalJointBuilder, Vect};
use bevy_rapier3d::render::{DebugRenderMode, RapierDebugRenderPlugin};
use std::ops::{Deref, Mul};
use bevy::DefaultPlugins;
use bevy::math::Vec3;
use bevy::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use k::{connect, InverseKinematicsSolver, JointType, NodeBuilder, SerialChain};
use crate::math_utils::vec3_y;
use crate::physics::{toggle_contacts_with, ToggleContactsWith};

fn main() {
    use bevy::prelude::*;
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
        .init_resource::<TestResource>()
        .add_systems(Startup, startup)
        .add_systems(PostUpdate, toggle_contacts_with.after(PhysicsSet::Writeback))
        .add_systems(PostUpdate, update.after(PhysicsSet::Writeback))
    ;

    let mut movement_settings = app.world.get_resource_mut::<bevy_flycam::MovementSettings>().unwrap();
    movement_settings.speed = 3.;


    app.run();
}

#[derive(Resource, Default)]
pub struct TestResource {
    pub sleep: bool
}

pub fn startup(
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

    let fixed = commands.spawn((
        RigidBody::Fixed,
        Sleeping::default(),
        Transform::default()
    )).id();

    let shoulder = commands.spawn((
        RigidBody::Dynamic,
        Sleeping::default(),
        Transform::default(),
        MultibodyJoint::new(fixed, SphericalJointBuilder::new())
    )).id();

    let radius = 0.05;
    let segment_shape = Capsule3d {
        half_length: 0.3 - radius,
        radius,
    };
    let upper_arm = commands.spawn((
        RigidBody::Dynamic,
        Sleeping::default(),
        ImpulseJoint::new(shoulder, FixedJointBuilder::new()
            .local_anchor2(vec3_y(segment_shape.half_length+radius))
        ),
        Collider::capsule_y(segment_shape.half_length, radius),
    )).id();

    let elb_z = commands.spawn((
        RigidBody::Dynamic,
        Sleeping::default(),
        Transform::default(),
        MultibodyJoint::new(upper_arm, RevoluteJointBuilder::new(Vect::Z)
            .local_anchor1(vec3_y(-segment_shape.half_length-radius))
        )
    )).id();

    let lower_arm = commands.spawn((
        RigidBody::Dynamic,
        Sleeping::default(),
        ImpulseJoint::new(elb_z, FixedJointBuilder::new()
            .local_anchor2(vec3_y(segment_shape.half_length+radius))
        ),
        ToggleContactsWith {
            entity: upper_arm,
            contacts_enabled: false
        },
        Collider::capsule_y(segment_shape.half_length, radius),
    )).id();


    let wrist_x = commands.spawn((
        RigidBody::Dynamic,
        Sleeping::default(),
        Transform::default(),
        MultibodyJoint::new(lower_arm, RevoluteJointBuilder::new(Vect::X)
            .local_anchor1(vec3_y(-segment_shape.half_length-radius)))
    )).id();
    let wrist_y = commands.spawn((
        RigidBody::Dynamic,
        Sleeping::default(),
        Transform::default(),
        MultibodyJoint::new(wrist_x, RevoluteJointBuilder::new(Vect::Y))
    )).id();

    let arm = commands.spawn(Arm {
        entities: [
            fixed,
            shoulder,
            elb_z,
            wrist_x,
            wrist_y
        ],
        chain: SerialChain::new_unchecked(k::Chain::from_root(create_arm()))
    });
}

pub fn update(
    arm_q: Query<&Arm>,
    joint_q: Query<(&Transform, &MultibodyJoint)>,
    cam_q: Query<&Transform, With<FlyCam>>,
    mut sleeping_q: Query<&mut Sleeping>,

    keys: Res<ButtonInput<KeyCode>>,
    mut sleeping_res: ResMut<TestResource>,
) {
    let r1 = cam_q.get_single();
    let r2 = arm_q.get_single();
    if r1.is_err() || r2.is_err() { return; }

    let cam_pos = r1.unwrap().translation;
    let [
    fixed,
    shoulder,
    elbow_z,
    wrist_x,
    wrist_y,
    ] = r2.unwrap().entities;

    //sleeping control
    {
        if keys.just_pressed(KeyCode::KeyK) {
            sleeping_res.sleep = !sleeping_res.sleep;
            sleeping_q.get_mut(fixed).unwrap().sleeping = sleeping_res.sleep;
            sleeping_q.get_mut(shoulder).unwrap().sleeping = sleeping_res.sleep;
            sleeping_q.get_mut(elbow_z).unwrap().sleeping = sleeping_res.sleep;
            sleeping_q.get_mut(wrist_x).unwrap().sleeping = sleeping_res.sleep;
            sleeping_q.get_mut(wrist_y).unwrap().sleeping = sleeping_res.sleep;
        }
    }

    let arm_root = create_arm();
    let chain = SerialChain::new_unchecked(k::Chain::from_root(arm_root));
    let nodes: Vec<&k::Node<Real>> = chain.iter().collect();
    let shld_pos = joint_q.get(shoulder).unwrap().0.translation;
    let elb_pos = joint_q.get(elbow_z).unwrap().0.translation;
    let wrist_pos = joint_q.get(wrist_x).unwrap().0.translation;

    // println!("\n
    //     shoulder: {shld_pos}\n
    //     elbow: {elb_pos}\n
    //     wrist: {wrist_pos}"
    // );

    //shoulder nodes
    nodes[1].set_origin(k::Isometry3::translation(shld_pos.x, shld_pos.y, shld_pos.z));
    nodes[2].set_origin(k::Isometry3::translation(shld_pos.x, shld_pos.y, shld_pos.z));
    nodes[3].set_origin(k::Isometry3::translation(shld_pos.x, shld_pos.y, shld_pos.z));

    //elbow node
    nodes[4].set_origin(k::Isometry3::translation(elb_pos.x, elb_pos.y, elb_pos.z));

    //wrist node
    nodes[5].set_origin(k::Isometry3::translation(wrist_pos.x, wrist_pos.y, wrist_pos.z));
    nodes[6].set_origin(k::Isometry3::translation(wrist_pos.x, wrist_pos.y, wrist_pos.z));

    let solver = k::JacobianIkSolver::new(
        0.1, 10_f32.to_radians(), 0.5, 10
    );

    use std::time::Instant;
    let start = Instant::now();
    let solver_result = solver.solve(&chain, &k::Isometry3::translation(cam_pos.x, cam_pos.y, cam_pos.z));
    if solver_result.is_err() {
        // println!("{}", solver_result.err().unwrap());
    }
    else {
        println!("Converged! Time to solve: {:.2?}", start.elapsed());
    }
}

#[derive(Component)]
pub struct Arm {
    pub entities: [Entity; 5],
    pub chain: SerialChain<Real>,
}

fn create_arm() -> k::Node<Real> {
    use k::Vector3;
    let fixed: k::Node<f32> = NodeBuilder::new()
        .name("fixed")
        .joint_type(JointType::Fixed)
        .finalize()
        .into();
    let l0: k::Node<f32> = NodeBuilder::new()
        .name("shoulder_x")
        .joint_type(JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .finalize()
        .into();
    let l1: k::Node<f32> = NodeBuilder::new()
        .name("shoulder_y")
        .joint_type(JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .finalize()
        .into();
    let l2: k::Node<f32> = NodeBuilder::new()
        .name("shoulder_z")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .finalize()
        .into();
    let l3: k::Node<f32> = NodeBuilder::new()
        .name("elbow_z")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .finalize()
        .into();
    let l4: k::Node<f32> = NodeBuilder::new()
        .name("wrist_x")
        .joint_type(JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .finalize()
        .into();
    let l5: k::Node<f32> = NodeBuilder::new()
        .name("wrist_y")
        .joint_type(JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .finalize()
        .into();
    connect![fixed => l0 => l1 => l2 => l3 => l4 => l5];
    fixed
}
