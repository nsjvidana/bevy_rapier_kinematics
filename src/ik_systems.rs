use std::ops::Mul;
use bevy::prelude::{Commands, Entity, Query, Transform, Vec3};
use bevy_rapier3d::na::Vector3;
use bevy_rapier3d::parry::math::Real;
use bevy_rapier3d::prelude::GenericJoint;
use crate::arm::{AttachedArmChain, ArmInfo, BodySegment, EntityChain};
use crate::ik::{JacobianIKArm, JacobianIKArmBundle};

pub fn prepare_ik_nodes(
    mut ik_arm_q: Query<(&EntityChain, &mut JacobianIKArm<Real>)>,
    transform_q: Query<&Transform>
) {
    for (entity_chain, ik_arm) in ik_arm_q.iter_mut() {
        let arm_chain = entity_chain.as_attached_arm()
                .expect("An EntityChain was expected to be an AttachedArmChain, but wasn't!");
        let shoulder_joints = arm_chain.shoulder_joints();
        let elbow_joints = arm_chain.elbow_joints();

        let shoulder_loc = transform_q.get(shoulder_joints[0])
            .unwrap()
            .translation;
        let elbow_loc = transform_q.get(elbow_joints[0])
            .unwrap()
            .translation;
        
        let shoulder_iso = k::Isometry3 {
            translation: k::Translation3::new(shoulder_loc.x, shoulder_loc.y, shoulder_loc.z),
            rotation: k::UnitQuaternion::identity()
        };
        let elbow_iso = k::Isometry3 {
            translation: k::Translation3::new(elbow_loc.x, elbow_loc.y, elbow_loc.z),
            rotation: k::UnitQuaternion::identity()
        };

        let chain = &ik_arm.chain;
        
        chain.find("shld_x").unwrap().set_origin(shoulder_iso);
        chain.find("shld_y").unwrap().set_origin(shoulder_iso);
        chain.find("shld_z").unwrap().set_origin(shoulder_iso);

        chain.find("elb_y").unwrap().set_origin(elbow_iso);
        chain.find("elb_x").unwrap().set_origin(elbow_iso);
    }
}