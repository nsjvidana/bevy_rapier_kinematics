mod math_utils;
mod ik;
mod ik_systems;
mod arm;
mod physics;

use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_rapier3d::dynamics::JointAxis;
use bevy_rapier3d::plugin::{RapierContext, RapierPhysicsPlugin};
use bevy_rapier3d::prelude::{Collider, FixedJointBuilder, ImpulseJoint, MultibodyJoint, PhysicsSet, Real, RevoluteJointBuilder, RigidBody, Sleeping, SphericalJointBuilder, Vect};
use bevy_rapier3d::render::{DebugRenderMode, RapierDebugRenderPlugin};
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

    let target_vel = 10_f32.to_radians();

    let fixed = commands.spawn((
        RigidBody::Fixed,
        Sleeping::default(),
        Transform::default()
    )).id();

    let shoulder = commands.spawn((
        RigidBody::Dynamic,
        Sleeping::default(),
        Transform::default(),
        MultibodyJoint::new(fixed, SphericalJointBuilder::new()
            .motor(JointAxis::AngX, 0., target_vel, 1., 0.05)
            .motor(JointAxis::AngY, 0., target_vel, 1., 0.05)
            .motor(JointAxis::AngZ, 0., target_vel, 1., 0.05)
        )
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
            .motor(0., target_vel, 1., 0.05)
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
            .local_anchor1(vec3_y(-segment_shape.half_length-radius))
            .motor(0., target_vel, 1., 0.05)
        )
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
    joint_transforms_q: Query<&Transform, With<MultibodyJoint>>,
    mut joint_q: Query<&mut MultibodyJoint>,
    cam_q: Query<&Transform, With<FlyCam>>,
    mut sleeping_q: Query<&mut Sleeping>,

    keys: Res<ButtonInput<KeyCode>>,
    mut sleeping_res: ResMut<TestResource>,
    rapier_context: Res<RapierContext>,
) {
    if !keys.just_pressed(KeyCode::KeyP) { return; }

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

    let arm_root = create_arm();
    let chain = SerialChain::new_unchecked(k::Chain::from_root(arm_root));
    let nodes: Vec<&k::Node<Real>> = chain.iter().collect();

    let shld_pos = joint_transforms_q.get(shoulder).unwrap().translation;
    let elb_pos = joint_transforms_q.get(elbow_z).unwrap().translation;
    let wrist_pos = joint_transforms_q.get(wrist_x).unwrap().translation;

    let shoulder_handle = rapier_context.entity2multibody_joint().get(&shoulder).unwrap();
    let elbow_handle = rapier_context.entity2multibody_joint().get(&elbow_z).unwrap();
    let wrist_x_handle = rapier_context.entity2multibody_joint().get(&wrist_x).unwrap();
    let wrist_y_handle = rapier_context.entity2multibody_joint().get(&wrist_y).unwrap();

    let (mb, link_id) = rapier_context.multibody_joints.get(*shoulder_handle).unwrap();
    let shld_relative = mb.link(link_id).unwrap().joint.body_to_parent();

    let (mb, link_id) = rapier_context.multibody_joints.get(*elbow_handle).unwrap();
    let elb_relative = mb.link(link_id).unwrap().joint.body_to_parent();
    
    let (mb, link_id) = rapier_context.multibody_joints.get(*wrist_x_handle).unwrap();
    let wrist_x_relative = mb.link(link_id).unwrap().joint.body_to_parent();
    let (mb, link_id) = rapier_context.multibody_joints.get(*wrist_y_handle).unwrap();
    let wrist_y_relative = mb.link(link_id).unwrap().joint.body_to_parent();
    


    //shoulder nodes
    let shld_trans = k::Isometry3::from_parts(
        k::Translation3::new(shld_pos.x, shld_pos.y, shld_pos.z),
        unsafe { std::mem::transmute(shld_relative.rotation) }
    );
    nodes[1].set_origin(shld_trans);
    nodes[2].set_origin(shld_trans);
    nodes[3].set_origin(shld_trans);

    //elbow node
    let elb_trans = k::Isometry3::from_parts(
        k::Translation3::new(elb_pos.x, elb_pos.y, elb_pos.z),
        unsafe { std::mem::transmute(elb_relative.rotation) }
    );
    nodes[4].set_origin(elb_trans);

    //wrist node
    let wrist_trans = k::Isometry3::from_parts(
        k::Translation3::new(wrist_pos.x, wrist_pos.y, wrist_pos.z),
        unsafe { std::mem::transmute(wrist_x_relative.rotation * wrist_y_relative.rotation) }
    );
    nodes[5].set_origin(wrist_trans);
    nodes[6].set_origin(wrist_trans);

    let solver = k::JacobianIkSolver::new(
        0.1, 10_f32.to_radians(), 0.5, 10
    );

    use std::time::Instant;
    let start = Instant::now();
    let solver_result = solver.solve(&chain, &k::Isometry3::translation(cam_pos.x, cam_pos.y, cam_pos.z));
    let elapsed = start.elapsed();
    if solver_result.is_err() {
        if keys.pressed(KeyCode::KeyE) {
            println!("{}", solver_result.err().unwrap());
        }
        return;
    }
    
    println!("Converged! Time to solve: {:.2?}", elapsed);
    
    chain.update_transforms();
    
    //shoulder
    let mut shld_joint = joint_q.get_mut(shoulder).unwrap();
    shld_joint.data.raw.motors[JointAxis::AngX as usize].target_pos = nodes[1]
        .world_transform().unwrap().rotation.euler_angles().0;
    shld_joint.data.raw.motors[JointAxis::AngY as usize].target_pos = nodes[2]
        .world_transform().unwrap().rotation.euler_angles().1;
    shld_joint.data.raw.motors[JointAxis::AngZ as usize].target_pos = nodes[3]
        .world_transform().unwrap().rotation.euler_angles().2;

    //elbow (z axis)
    joint_q.get_mut(elbow_z).unwrap().data.raw.motors[JointAxis::AngZ as usize].target_pos = nodes[4]
        .world_transform().unwrap().rotation.euler_angles().2;

    //wrist
    joint_q.get_mut(wrist_x).unwrap().data.raw.motors[JointAxis::AngX as usize].target_pos = nodes[5]
        .world_transform().unwrap().rotation.euler_angles().0;
    joint_q.get_mut(wrist_y).unwrap().data.raw.motors[JointAxis::AngY as usize].target_pos = nodes[6]
        .world_transform().unwrap().rotation.euler_angles().1;
    
    //waking up the joints
    sleeping_res.sleep = !sleeping_res.sleep;
    sleeping_q.get_mut(fixed).unwrap().sleeping = false;
    sleeping_q.get_mut(shoulder).unwrap().sleeping = false;
    sleeping_q.get_mut(elbow_z).unwrap().sleeping = false;
    sleeping_q.get_mut(wrist_x).unwrap().sleeping = false;
    sleeping_q.get_mut(wrist_y).unwrap().sleeping = false;
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
