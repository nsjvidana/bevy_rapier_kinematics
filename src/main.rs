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
use k::connect;
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

    let base = KNodeBuilder::new()
        .joint_type(KJointType::Revolute { axis: Vector3::y_axis() })
        .build();
    let mut previous = base.clone();
    for _ in 0..5 {
        let new_joint = KNodeBuilder::new()
            .joint_type(KJointType::Revolute { axis: Vector3::x_axis() })
            .translation(Vector3::new(0., -1., 0.).into())
            .build();
        connect![previous => new_joint];
        previous = new_joint.clone();
    }

    let mut chain = SerialKChain::from_root(&base);
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
