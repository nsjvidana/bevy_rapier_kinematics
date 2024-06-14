use std::ops::Mul;
use bevy::prelude::{Commands, Entity, Query, Transform, Vec3};
use bevy_rapier3d::na::Vector3;
use bevy_rapier3d::parry::math::Real;
use crate::arm::{ArmInfo, BodySegment};
use crate::ik::{JacobianIKArm};

pub fn set_ik_arm_positions(
    mut ik_arm_q: Query<(&ArmInfo, &mut JacobianIKArm<Real>)>,
    body_segment_q: Query<(&dyn BodySegment, &Transform)>,
    transform_q: Query<&Transform>
) {
    for (arm_info, ik_arm) in ik_arm_q.iter_mut() {
        if ik_arm.target_pos.is_none() { continue; }
        let target_pos = ik_arm.target_pos.unwrap();
        
        let (upper_segment, upper_transform) = body_segment_q.get(arm_info.upper_arm).unwrap();
        let (lower_segment, lower_transform) = body_segment_q.get(arm_info.lower_arm).unwrap();
        let upper_arm_len = upper_segment.iter().next().unwrap().get_length();
        let lower_arm_len = lower_segment.iter().next().unwrap().get_length();

        let wrist_pos = lower_transform.mul(Vec3::new(0., -lower_arm_len/2., 0.));
        let dist_to_target = wrist_pos.distance(target_pos.into());
        if dist_to_target <= ik_arm.ik_solver.allowable_target_distance { continue; }


        if let Some(ik_pole_entity) = ik_arm.elbow_ik_pole {
            let shoulder_pos = Vector3::<Real>::from(
                upper_transform.mul(Vec3::new(0., upper_arm_len/2., 0.))
            );
            let ik_pole = transform_q.get(ik_pole_entity).unwrap().translation;
            let mut dir_to_pole = Vector3::from(ik_pole) - Vector3::<Real>::from(shoulder_pos);
                dir_to_pole.normalize_mut();
            for node in ik_arm.chain.iter() {
                let name = &node.joint().name;
                let node_pos =
                    if name.starts_with("shoulder") { shoulder_pos }
                    else if name.starts_with("elbow") { dir_to_pole.scale(upper_arm_len) }
                    else { dir_to_pole.scale(upper_arm_len + lower_arm_len) };
                node.set_origin(k::Isometry3::<Real> {
                    rotation: k::UnitQuaternion::identity(),
                    translation: k::Translation3::new(
                        node_pos.x,
                        node_pos.y,
                        node_pos.z
                    )
                });
            }
        }
        else {
            
        }
    }
}