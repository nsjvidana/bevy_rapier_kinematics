mod math_utils;

use bevy::prelude::*;
use bevy_flycam::PlayerPlugin;
use bevy_rapier3d::plugin::RapierPhysicsPlugin;
use bevy_rapier3d::prelude::{Collider, RigidBody};
use bevy_rapier3d::render::RapierDebugRenderPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PlayerPlugin,
            RapierPhysicsPlugin::<()>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, test_system)
        .run();
}

fn test_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    //spawn ground
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(10., 0.1, 10.),
        Transform::from_translation(Vec3::new(0., 2039., 0.)),
    ));

    let radius = 0.05;

    let segment_shape = Capsule3d {
        half_length: 0.15 - radius,
        radius,
    };
    let torso_shape = Capsule3d {
        half_length: 0.5 - radius,
        radius
    };
    let segment_mesh = meshes.add(segment_shape);
    let torso_mesh = meshes.add(torso_shape);

    let material = materials.add(Color::rgb(1., 0., 0.));
    commands.spawn((
        RigidBody::Fixed,
        PbrBundle {
            mesh: torso_mesh,
            material,
            transform: Transform::from_translation(Vec3::ZERO),
            ..default()
        },
        Collider::capsule_y(torso_shape.half_length, torso_shape.radius),
    ));

}
