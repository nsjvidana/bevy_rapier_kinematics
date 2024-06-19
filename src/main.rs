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
use shape::Cube;
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
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
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

    //testing entity
    commands.spawn((
        TestingEntity,
        PbrBundle {
            mesh: meshes.add(Cuboid::new(0.05, 0.05, 0.05).mesh()),
            material: materials.add(Color::YELLOW),
            ..default()
        }
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


    let wrist = commands.spawn((
        RigidBody::Dynamic,
        Sleeping::default(),
        Transform::default(),
        MultibodyJoint::new(lower_arm, SphericalJointBuilder::new()
            .local_anchor1(vec3_y(-segment_shape.half_length-radius))
            .motor(JointAxis::AngX, 0., target_vel, 1., 0.05)
            .motor(JointAxis::AngY, 0., target_vel, 1., 0.05)
            .motor(JointAxis::AngZ, 0., target_vel, 1., 0.05)
        )
    )).id();
    // let wrist_y = commands.spawn((
    //     RigidBody::Dynamic,
    //     Sleeping::default(),
    //     Transform::default(),
    //     MultibodyJoint::new(wrist_x, RevoluteJointBuilder::new(Vect::Y))
    // )).id();

    let arm = commands.spawn(Arm {
        entities: [
            fixed,
            shoulder,
            elb_z,
            wrist,
        ],
        chain: SerialChain::new_unchecked(k::Chain::from_root(create_arm()))
    });
}

pub fn update(
    arm_q: Query<&Arm>,
    joint_transforms_q: Query<&Transform, With<MultibodyJoint>>,
    mut joint_q: Query<&mut MultibodyJoint>,
    mut sleeping_q: Query<&mut Sleeping>,
    mut testing_q: Query<&mut Transform, (With<TestingEntity>, Without<MultibodyJoint>)>,

    keys: Res<ButtonInput<KeyCode>>,
    mut sleeping_res: ResMut<TestResource>,
    rapier_context: Res<RapierContext>,
    time: Res<bevy::prelude::Time>,

    mut gizmos: Gizmos
) {
    let r1 = testing_q.get_single_mut();
    let r2 = arm_q.get_single();
    if r1.is_err() || r2.is_err() { return; }

    let mut targ_transform = r1.unwrap();
    let left = keys.pressed(KeyCode::ArrowLeft) as i8 as f32;
    let right = keys.pressed(KeyCode::ArrowRight) as i8 as f32;
    let fwd = keys.pressed(KeyCode::ArrowUp) as i8 as f32;
    let back = keys.pressed(KeyCode::ArrowDown) as i8 as f32;
    let up = keys.pressed(KeyCode::Numpad8) as i8 as f32;
    let down = keys.pressed(KeyCode::Numpad2) as i8 as f32;

    targ_transform.translation += 0.2 * time.delta_seconds() * Vec3::new(left-right, up-down, fwd-back);

    let targ_pos = targ_transform.translation;
    let [
        fixed,
        shoulder,
        elbow_z,
        wrist,
    ] = r2.unwrap().entities;

    let arm_root = create_arm();
    let chain = SerialChain::new_unchecked(k::Chain::from_root(arm_root));
    let nodes: Vec<&k::Node<Real>> = chain.iter().collect();

    let shld_pos = joint_transforms_q.get(shoulder).unwrap().translation;
    let elb_pos = joint_transforms_q.get(elbow_z).unwrap().translation;
    let wrist_pos = joint_transforms_q.get(wrist).unwrap().translation;

    let shoulder_handle = rapier_context.entity2multibody_joint().get(&shoulder).unwrap();
    let elbow_handle = rapier_context.entity2multibody_joint().get(&elbow_z).unwrap();
    let wrist_handle = rapier_context.entity2multibody_joint().get(&wrist).unwrap();
    // let wrist_x_handle = rapier_context.entity2multibody_joint().get(&wrist_x).unwrap();
    // let wrist_y_handle = rapier_context.entity2multibody_joint().get(&wrist_y).unwrap();

    let (mb, link_id) = rapier_context.multibody_joints.get(*shoulder_handle).unwrap();
    let shld_relative = mb.link(link_id).unwrap().joint.body_to_parent();

    let (mb, link_id) = rapier_context.multibody_joints.get(*elbow_handle).unwrap();
    let elb_relative = mb.link(link_id).unwrap().joint.body_to_parent();
    
    let (mb, link_id) = rapier_context.multibody_joints.get(*wrist_handle).unwrap();
    let wrist_relative = mb.link(link_id).unwrap().joint.body_to_parent();
    


    //shoulder nodes
    let (x, y, z) = shld_relative.rotation.to_rotation_matrix().euler_angles();
    let mut shld_trans = k::Isometry3::from_parts(
        k::Translation3::new(shld_pos.x, shld_pos.y, shld_pos.z),
        k::UnitQuaternion::identity()
    );
    shld_trans.rotation = k::UnitQuaternion::from_euler_angles(x, 0., 0.);
        nodes[1].set_origin(shld_trans);
    shld_trans.rotation = k::UnitQuaternion::from_euler_angles(0., y, 0.);
        nodes[2].set_origin(shld_trans);
    shld_trans.rotation = k::UnitQuaternion::from_euler_angles(0., 0., z);
        nodes[3].set_origin(shld_trans);

    //elbow node
    let elb_trans = k::Isometry3::from_parts(
        k::Translation3::new(elb_pos.x, elb_pos.y, elb_pos.z),
        unsafe { std::mem::transmute(elb_relative.rotation) }
    );
    nodes[4].set_origin(elb_trans);

    //wrist node
    let (x, y, z) = wrist_relative.rotation.to_rotation_matrix().euler_angles();
    let mut wrist_trans = k::Isometry3::from_parts(
        k::Translation3::new(wrist_pos.x, wrist_pos.y, wrist_pos.z),
        k::UnitQuaternion::identity()
    );
    wrist_trans.rotation = k::UnitQuaternion::from_euler_angles(x, 0., 0.);
        nodes[5].set_origin(wrist_trans);
    wrist_trans.rotation = k::UnitQuaternion::from_euler_angles(0., y, 0.);
        nodes[6].set_origin(wrist_trans);
    wrist_trans.rotation = k::UnitQuaternion::from_euler_angles(0., 0., z);
        nodes[7].set_origin(wrist_trans);

    let solver = k::JacobianIkSolver::new(
        0.1, 10_f32.to_radians(), 0.5, 10
    );

    use std::time::Instant;
    let start = Instant::now();
    let solver_result = solver.solve(&chain, &k::Isometry3::translation(targ_pos.x, targ_pos.y, targ_pos.z));
    let elapsed = start.elapsed();
    if solver_result.is_err() {
        if keys.pressed(KeyCode::KeyE) {
            println!("{}", solver_result.err().unwrap());
        }
        return;
    }
    
    println!("Converged! Time to solve: {:.2?}", elapsed);
    
    chain.update_transforms();

    let shld_pos = nodes[3].world_transform().unwrap().translation;
    let elb_pos = nodes[4].world_transform().unwrap().translation;
    let wrist_pos = nodes[6].world_transform().unwrap().translation;

    gizmos.sphere(Vec3::new(shld_pos.x, shld_pos.y, shld_pos.z), Quat::IDENTITY, 0.05, Color::RED);
    gizmos.sphere(Vec3::new(elb_pos.x, elb_pos.y, elb_pos.z), Quat::IDENTITY, 0.05, Color::GREEN);
    gizmos.sphere(Vec3::new(wrist_pos.x, wrist_pos.y, wrist_pos.z), Quat::IDENTITY, 0.05, Color::BLUE);

    return;
    
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
    let wrist_motors = &mut joint_q.get_mut(wrist).unwrap().data.raw.motors;
    wrist_motors[JointAxis::AngX as usize].target_pos = nodes[5]
        .world_transform().unwrap().rotation.euler_angles().0;
    wrist_motors[JointAxis::AngY as usize].target_pos = nodes[6]
        .world_transform().unwrap().rotation.euler_angles().1;
    wrist_motors[JointAxis::AngZ as usize].target_pos = nodes[7]
        .world_transform().unwrap().rotation.euler_angles().2;
    
    //waking up the joints
    sleeping_res.sleep = !sleeping_res.sleep;
    sleeping_q.get_mut(fixed).unwrap().sleeping = false;
    sleeping_q.get_mut(shoulder).unwrap().sleeping = false;
    sleeping_q.get_mut(elbow_z).unwrap().sleeping = false;
    sleeping_q.get_mut(wrist).unwrap().sleeping = false;
    // sleeping_q.get_mut(wrist_x).unwrap().sleeping = false;
    // sleeping_q.get_mut(wrist_y).unwrap().sleeping = false;
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
    let l6: k::Node<f32> = NodeBuilder::new()
        .name("wrist_z")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .finalize()
        .into();
    connect![fixed => l0 => l1 => l2 => l3 => l4 => l5 => l6];
    fixed
}

#[derive(Component)]
pub struct Arm {
    pub entities: [Entity; 4],
    pub chain: SerialChain<Real>,
}

#[derive(Component)]
pub struct TestingEntity;
