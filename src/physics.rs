use bevy::prelude::{Added, Component, Entity, Query, ResMut};
use bevy_rapier3d::prelude::RapierContext;
use bevy_rapier3d::rapier::prelude::GenericJoint;

#[derive(Component)]
pub struct ToggleContactsWith {
    pub entity: Entity,
    pub contacts_enabled: bool
}

pub fn toggle_contacts_with(
    mut rapier_context: ResMut<RapierContext>,
    toggled_contacts_q: Query<(Entity, &ToggleContactsWith), Added<ToggleContactsWith>>
) {
    for (entity, toggled_contact) in toggled_contacts_q.iter() {
        let this_body = *rapier_context.entity2body().get(&entity).unwrap();
        let other_body = *rapier_context.entity2body().get(&toggled_contact.entity).unwrap();
        rapier_context.impulse_joints.insert(
            this_body,
            other_body,
            *GenericJoint::default().set_contacts_enabled(toggled_contact.contacts_enabled),
            true
        );
    }
}
