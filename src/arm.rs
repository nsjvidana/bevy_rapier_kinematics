use bevy::prelude::{Bundle, Capsule3d, Component, Entity, Transform};
use bevy_rapier3d::parry::math::Real;
use bevy_rapier3d::prelude::{Collider, RigidBody};

#[derive(Component)]
pub struct EntityChain {
    joints: Vec<Entity>,
    parts: Vec<Entity>
}

impl EntityChain {
    pub fn as_attached_arm(&self) -> Option<&AttachedArmChain> {
        if self.joints.len() == 5 && self.parts.len() == 4 {
            Some(unsafe { std::mem::transmute(self) })
        }
        else {
            None
        }
    }
}

impl Default for EntityChain {
    fn default() -> Self {
        Self {
            joints: vec![],
            parts: vec![],
        }
    }
}

pub struct AttachedArmChain {
    raw: EntityChain,
}

impl AttachedArmChain {
    #[inline]
    pub fn new(
        torso: Entity,
        shoulder_joints: [Entity; 3],
        upper_arm: Entity,
        elbow_joints: [Entity; 2],
        lower_arm: Entity,
        wrist: Entity
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
                wrist
            ],
        };
        Self {
            raw,
        }
    }

    pub fn shoulder_joints(&self) -> &[Entity] {
        &self.raw.joints[0..3]
    }

    pub fn elbow_joints(&self) -> &[Entity] {
        &self.raw.joints[3..5]
    }

    pub fn upper_arm(&self) -> Entity {
        self.raw.parts[1]
    }

    pub fn lower_arm(&self) -> Entity {
        self.raw.parts[2]
    }
    
    pub fn wrist(&self) -> Entity {
        self.raw.parts[3]
    }

    pub fn torso(&self) -> Option<Entity> {
        Some(self.raw.parts[0])
    }
}

impl Into<EntityChain> for AttachedArmChain {
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
