mod math_utils;

use bevy::prelude::*;
use bevy_flycam::{FlyCam, PlayerPlugin};
use crate::math_utils::rotation_from_fwd;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PlayerPlugin)
        .add_systems(Startup, test_system)
        .run();
}

fn test_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>
) {
    let cube = Cuboid {
        half_size: Vec3::new(1.,1.,1.)
    };
    let material = Color::rgb(1., 0., 0.);

    commands.spawn(PbrBundle {
        mesh: meshes.add(cube.mesh()),
        material: materials.add(material),
        ..default()
    });
}
