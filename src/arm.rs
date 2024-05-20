use bevy::prelude::*;
use bevy_rapier3d::na::{Scalar, Vector3};
use bevy_rapier3d::prelude::{Collider, RigidBody};

#[derive(Component)]
pub struct ArmInfo<T> where T: Scalar {
    pub upper_arm: Entity,
    pub lower_arm: Entity,
    pub upper_arm_len: T,
    pub lower_arm_len: T,
}

#[derive(Component)]
pub struct CapsuleSegmentInfo {
    pub capsule: Capsule3d
}

#[derive(Component)]
pub struct FabrikArmInfo<T> where T: Scalar {
    pub arm_info: ArmInfo<T>,
    
    pub shoulder_pos: Vector3<T>,
    pub elbow_pos: Vector3<T>,
    pub arm_pos: Vector3<T>,
    pub elbow_ik_pole_pos: Vector3<T>,
    
    pub num_iterations: usize
}

impl<T> FabrikArmInfo<T> where T: Scalar {
    pub fn solve(target_pos: Vector3<T>) {
        
    }
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
