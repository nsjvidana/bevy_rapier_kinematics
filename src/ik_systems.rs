use std::ops::Mul;
use bevy::prelude::{Commands, Entity, Query, Transform, With};
use bevy_rapier3d::parry::math::Real;
use k::{connect, JointType, NodeBuilder, SerialChain, Translation3};
use crate::arm::{ArmInfo, BodySegment};
use crate::ik::{IKArm};
use crate::math_utils::vec3_y;

pub fn set_ik_arm_positions(
    mut commands: Commands,
    mut ik_arm_q: Query<(&ArmInfo, &mut IKArm<Real>, Entity)>,
    body_segment_q: Query<(&dyn BodySegment, &Transform)>
) {
    for (arm_info, mut ik_arm, arm_entity) in ik_arm_q.iter_mut() {
        let mut cmd = commands.entity(arm_entity);
        
        let (upper_segment, upper_transform) = body_segment_q.get(arm_info.upper_arm).unwrap();
        let (lower_segment, lower_transform) = body_segment_q.get(arm_info.lower_arm).unwrap();
        let upper_arm_len = upper_segment.iter().next().unwrap().get_length();
        let lower_arm_len = lower_segment.iter().next().unwrap().get_length();
    }
}