use std::ops::Mul;
use bevy::prelude::{Commands, Entity, Query, Transform, With};
use bevy_rapier3d::parry::math::Real;
use k::{connect, JointType, NodeBuilder, SerialChain, Translation3};
use crate::arm::{ArmInfo, BodySegment};
use crate::ik::{IKArm, UninitializedIKArm};
use crate::math_utils::vec3_y;

pub fn initialize_ik_arms(
    mut commands: Commands,
    mut ik_arm_q: Query<(&ArmInfo, &mut IKArm<Real>, Entity), With<UninitializedIKArm>>,
    body_segment_q: Query<(&dyn BodySegment, &Transform)>
) {
    for (arm_info, mut ik_arm, arm_entity) in ik_arm_q.iter_mut() {
        let mut cmd = commands.entity(arm_entity);
        
        let (upper_segment, upper_transform) = body_segment_q.get(arm_info.upper_arm).unwrap();
        let (lower_segment, lower_transform) = body_segment_q.get(arm_info.lower_arm).unwrap();
        let upper_arm_len = upper_segment.iter().next().unwrap().get_length();
        let lower_arm_len = lower_segment.iter().next().unwrap().get_length();

        let mut temp_vec3 = upper_transform.mul(vec3_y(upper_arm_len));
        let shoulder_pos: Translation3<f32> = Translation3::new(temp_vec3.x, temp_vec3.y, temp_vec3.z);
        temp_vec3 = lower_transform.mul(vec3_y(lower_arm_len));
        let elbow_pos: Translation3<f32> = Translation3::new(temp_vec3.x, temp_vec3.y, temp_vec3.z);
        temp_vec3 = lower_transform.mul(vec3_y(-lower_arm_len));
        let wrist_pos: Translation3<f32> = Translation3::new(temp_vec3.x, temp_vec3.y, temp_vec3.z);

        let fixed: k::Node<f32> = NodeBuilder::new()
            .name("fixed")
            .joint_type(JointType::Fixed)
            .translation(shoulder_pos)
            .finalize()
            .into();
        let l0: k::Node<f32> = NodeBuilder::new()
            .name("shoulder_x_rot")
            .joint_type(JointType::Rotational {
                axis: k::Vector3::x_axis()
            })
            .translation(shoulder_pos)
            .finalize()
            .into();
        let l1: k::Node<f32> = NodeBuilder::new()
            .name("shoulder_y_rot")
            .joint_type(JointType::Rotational {
                axis: k::Vector3::y_axis()
            })
            .translation(shoulder_pos)
            .finalize()
            .into();
        let l2: k::Node<f32> = NodeBuilder::new()
            .name("shoulder_z_rot")
            .joint_type(JointType::Rotational {
                axis: k::Vector3::z_axis()
            })
            .translation(shoulder_pos)
            .finalize()
            .into();
        let l3: k::Node<f32> = NodeBuilder::new()
            .name("elbow_y_rot")
            .joint_type(JointType::Rotational {
                axis: k::Vector3::y_axis()
            })
            .translation(elbow_pos)
            .finalize()
            .into();
        let l4: k::Node<f32> = NodeBuilder::new()
            .name("wrist")
            .translation(wrist_pos)
            .finalize()
            .into();
        connect![fixed => l0 => l1 => l2 => l3 => l4];
        ik_arm.arm_chain = SerialChain::new_unchecked(k::Chain::from_root(fixed));

        cmd.remove::<UninitializedIKArm>();
    }
}