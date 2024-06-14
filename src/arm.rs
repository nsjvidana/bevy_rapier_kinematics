use bevy::prelude::{Bundle, Capsule3d, Component, Entity, Transform};
use bevy_rapier3d::parry::math::Real;
use bevy_rapier3d::prelude::{Collider, RigidBody};

#[derive(Component)]
pub struct EntityChain {
    joints: Vec<Entity>,
    parts: Vec<Entity>
}

impl Default for EntityChain {
    fn default() -> Self {
        Self {
            joints: vec![],
            parts: vec![],
        }
    }
}

pub struct ArmChain {
    raw: EntityChain,
    is_attached: bool
}

impl ArmChain {
    #[inline]
    pub fn new_attached(
        torso: Entity,
        shoulder_joints: [Entity; 3],
        upper_arm: Entity,
        elbow_joints: [Entity; 2],
        lower_arm: Entity
    ) -> Self {
        let raw = EntityChain {
            joints: vec![
                shoulder_joints[0], shoulder_joints[1], shoulder_joints[2],
                elbow_joints[0], elbow_joints[1],
            ],
            parts: vec![
                torso,
                upper_arm,
                lower_arm,
            ],
        };
        Self {
            raw,
            is_attached: true
        }
    }

    pub fn shoulder_joints(&self) -> &[Entity] {
        &self.raw.joints[0..3]
    }

    pub fn elbow_joints(&self) -> &[Entity] {
        &self.raw.joints[3..5]
    }

    pub fn upper_arm(&self) -> Entity {
        if self.is_attached { return self.raw.parts[1] }
        return self.raw.parts[0]
    }

    pub fn lower_arm(&self) -> Entity {
        if self.is_attached { return self.raw.parts[2] }
        return self.raw.parts[1]
    }

    pub fn torso(&self) -> Option<Entity> {
        if self.is_attached { return Some(self.raw.parts[0]) }
        return None
    }

    pub fn is_attached(&self) -> bool {
        self.is_attached
    }
}

impl From<EntityChain> for ArmChain {
    fn from(value: EntityChain) -> Self {
        if value.joints.len() != 5 || value.parts.len() < 2 || value.parts.len() > 3 {
            panic!("This EntityChain is not an ArmChain!")
        }

        Self {
            is_attached: value.parts.len() == 3,
            raw: value,
        }
    }
}

impl Into<EntityChain> for ArmChain {
    fn into(self) -> EntityChain {
        self.raw
    }
}

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
