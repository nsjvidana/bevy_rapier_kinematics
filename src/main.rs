mod math_utils;
mod ik;
mod arm;
mod physics;
mod testing;
mod chain;
mod node;
mod iterator;

use bevy_flycam::{FlyCam, MovementSettings, NoCameraPlayerPlugin};
use bevy_rapier3d::na::{Isometry3, SimdValue, UnitVector3, Vector, Vector3};
use bevy_rapier3d::plugin::RapierPhysicsPlugin;
use bevy_rapier3d::prelude::{Collider, RigidBody};
use bevy_rapier3d::render::RapierDebugRenderPlugin;
use bevy::math::Vec3;
use bevy::prelude::*;
use chain::SerialKChain;
use ik::CyclicIKSolver;
use math_utils::project_onto_plane;
use node::{KJointType, KNodeBuilder};

fn main() {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<()>::default(),
        RapierDebugRenderPlugin::default(),
    ))
        .add_systems(Startup, startup)
        .add_systems(Update, update);

    //flycam stuff
    app.add_plugins(NoCameraPlayerPlugin);
    app.world_mut().resource_mut::<MovementSettings>()
        .speed = 2.5;

    app.run();
}

pub fn startup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>
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
    //ground
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(10., 0.1, 10.),
        Transform::from_xyz(0., -5., 0.),
    ));
}

pub fn update(
    mut gizmos: Gizmos,
    keys: Res<ButtonInput<KeyCode>>,
    cam_query: Query<&Transform, With<FlyCam>>
) {
    let cam_transform = cam_query.get_single().ok().unwrap();

    let lim = [-60f32.to_radians(), 60f32.to_radians()];

    let fixed = KNodeBuilder::new().joint_type(KJointType::Fixed)
        .build();
    let shoulder_x = KNodeBuilder::new()
        .joint_type(KJointType::Revolute { axis: Vector::x_axis() })
        .limits(lim)
        .build();
    let shoulder_y = KNodeBuilder::new()
        .joint_type(KJointType::Revolute { axis: Vector::y_axis() })
        .limits(lim)
        .build();
    let shoulder_z = KNodeBuilder::new()
        .joint_type(KJointType::Revolute { axis: Vector::z_axis() })
        .limits(lim)
        .build();
    
    let elbow_x = KNodeBuilder::new()
        .joint_type(KJointType::Revolute { axis: Vector::x_axis() })
        .translation(Vector3::new(0., -0.8, 0.).into())
        .limits(lim)
        .build();
    let elbow_y = KNodeBuilder::new()
        .joint_type(KJointType::Revolute { axis: Vector::y_axis() })
        .limits(lim)
        .build();
    
    let wrist_x = KNodeBuilder::new()
        .joint_type(KJointType::Revolute { axis: Vector::x_axis() })
        .translation(Vector3::new(0., -1., 0.).into())
        .limits(lim)
        .build(); 
    let wrist_z = KNodeBuilder::new()
        .joint_type(KJointType::Revolute { axis: Vector::z_axis() })
        .limits(lim)
        .build();

    chain_nodes![fixed => shoulder_x => shoulder_y => shoulder_z => elbow_x => elbow_y => wrist_x => wrist_z];
    let mut chain = SerialKChain::from_root(&fixed);
    let solver = CyclicIKSolver {
        allowable_target_distance: 0.1,
        allowable_target_angle: 1f32.to_radians(),
        max_iterations: 1
    };
    let solver_result = solver.solve(&mut chain, Isometry3 {
        rotation: cam_transform.rotation.into(),
        translation: cam_transform.translation.into(),
    });
    chain.update_world_transforms();

    for joint in chain.iter_joints() {
        gizmos.sphere(
            joint.world_transform().unwrap().translation.into(),
            default(),
            0.05,
            Color::linear_rgb(0., 1., 0.)
        );
    }

    if solver_result.is_err() {
        println!("{}", solver_result.err().unwrap());
    }
}
