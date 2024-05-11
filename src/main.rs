use bevy::math;
use bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Update, test_system)
        .run();
}

fn test_system(
    mut commands : Commands,
    mut meshes: Assets<Mesh>,
    mut materials: Assets<StandardMaterial>
) {
    let cube = Cuboid {
        half_size: Vec3::new(1.,1.,1.)
    };
    let material = Color::rgb(1., 0., 0.);

    commands.spawn(PbrBundle {
        mesh: meshes.add(cube.mesh()),
        material: materials.add(material.into()),
        ..default()
    });
}
