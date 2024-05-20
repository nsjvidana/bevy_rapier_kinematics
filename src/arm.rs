use bevy::prelude::{Bundle, Capsule3d, Component, Entity};
use bevy_rapier3d::prelude::{Collider, RigidBody};

#[derive(Component)]
pub struct ArmInfo {
    pub upper_arm: Entity,
    pub lower_arm: Entity,
}

#[derive(Component)]
pub struct CapsuleSegmentInfo {
    pub capsule: Capsule3d
}

#[derive(Bundle)]
pub struct CapsuleSegmentBundle {
    rb: RigidBody,
    collider: Collider,
    capsule_segment_info: CapsuleSegmentInfo,
}

impl CapsuleSegmentBundle {
    pub fn new(capsule_info: Capsule3d)-> Self {
        return Self {
            rb: RigidBody::Dynamic,
            collider: Collider::capsule_y(capsule_info.half_length, capsule_info.radius),
            capsule_segment_info: CapsuleSegmentInfo {
                capsule: capsule_info
            },
        }
    }
}
