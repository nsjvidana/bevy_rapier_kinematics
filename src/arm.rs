use bevy::prelude::{Bundle, Capsule3d, Component, Entity, Transform};
use bevy_rapier3d::parry::math::Real;
use bevy_rapier3d::prelude::{Collider, RigidBody};

#[derive(Component)]
pub struct ArmInfo {
    pub upper_arm: Entity,
    pub lower_arm: Entity,
    pub torso: Entity,
}

impl Default for ArmInfo {
    fn default() -> Self {
        Self {
            upper_arm: Entity::PLACEHOLDER,
            lower_arm: Entity::PLACEHOLDER,
            torso: Entity::PLACEHOLDER
        }
    }
}

#[derive(Component)]
pub struct CapsuleSegment {
    pub capsule: Capsule3d
}

impl BodySegment for CapsuleSegment {
    fn get_length(&self) -> Real {
        self.capsule.radius + (self.capsule.half_length*2.)
    }
}

#[derive(Bundle)]
pub struct CapsuleSegmentBundle {
    rb: RigidBody,
    collider: Collider,
    transform: Transform,
    capsule_segment: CapsuleSegment,
}

impl CapsuleSegmentBundle {
    pub fn new(capsule_info: Capsule3d) -> Self {
        return Self {
            rb: RigidBody::Dynamic,
            transform: Transform::default(),
            collider: Collider::capsule_y(capsule_info.half_length, capsule_info.radius),
            capsule_segment: CapsuleSegment {
                capsule: capsule_info
            },
        }
    }
}


#[bevy_trait_query::queryable]
pub trait BodySegment {
    fn get_length(&self) -> Real;
}
