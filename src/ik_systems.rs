use std::ops::Mul;
use bevy::prelude::{Commands, Entity, Query, Transform, Vec3};
use bevy_rapier3d::dynamics::MotorModel;
use bevy_rapier3d::na::Vector3;
use bevy_rapier3d::parry::math::Real;
use bevy_rapier3d::prelude::{GenericJoint, JointAxis, MultibodyJoint};
use bevy_rapier3d::rapier::prelude::JointMotor;
use crate::arm::{AttachedArmChain, ArmInfo, BodySegment, EntityChain};
use crate::ik::{JacobianIKArm, JacobianIKArmBundle};

pub fn prepare_ik_nodes(
    mut ik_arm_q: Query<(&EntityChain, &mut JacobianIKArm<Real>)>,//get mutable access to prevent waiting for mutex
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
        let wrist_loc = transform_q.get(arm_chain.wrist())
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

        chain.find("fixed").unwrap().set_origin(shoulder_iso);

        chain.find("shoulder_x").unwrap().set_origin(shoulder_iso);
        chain.find("shoulder_y").unwrap().set_origin(shoulder_iso);
        chain.find("shoulder_z").unwrap().set_origin(shoulder_iso);

        chain.find("elbow_x").unwrap().set_origin(elbow_iso);
        chain.find("elbow_y").unwrap().set_origin(elbow_iso);

        if let Some(targ) = ik_arm.target_pos {
            chain.find("wrist_x").unwrap().set_origin(k::Isometry3::from(targ));
            chain.find("wrist_y").unwrap().set_origin(k::Isometry3::from(targ));
            chain.find("wrist_z").unwrap().set_origin(k::Isometry3::from(targ));
        }
    }
}

pub fn solve_ik(
    mut ik_arm_q: Query<(&EntityChain, &mut JacobianIKArm<Real>)>,//get mutable access to prevent waiting for mutex
    mut joint_q: Query<&mut MultibodyJoint>,
) {
    for (entity_chain, mut ik_arm) in ik_arm_q.iter_mut() {
        if ik_arm.target_pos.is_none() { continue; }
        let targ = k::Isometry3::from(ik_arm.target_pos.unwrap());
        let result = ik_arm.solve(&targ);
        if result.is_err() {
            println!("{}", result.err().unwrap());
            continue;
        }

        ik_arm.chain.update_transforms();
        let shoulder_pos = ik_arm.chain.find("shld_x").unwrap()
            .world_transform().unwrap().translation;
        let elbow_pos = ik_arm.chain.find("elb_x").unwrap()
            .world_transform().unwrap().translation;

        let dir = elbow_pos.vector - shoulder_pos.vector;
        let quat = k::UnitQuaternion::look_at_rh(&dir, &k::Vector3::y());
        let (shld_x, shld_y, shld_z) = quat.euler_angles();

        let arm_chain = entity_chain.as_attached_arm().unwrap();
        let shoulder_joints = arm_chain.shoulder_joints();

        joint_q.get_mut(shoulder_joints[0]).unwrap()
            .data.as_mut().raw.motors[JointAxis::AngX as usize].target_pos = shld_x;
        joint_q.get_mut(shoulder_joints[1]).unwrap()
            .data.as_mut().raw.motors[JointAxis::AngY as usize].target_pos = shld_y;
        joint_q.get_mut(shoulder_joints[2]).unwrap()
            .data.as_mut().raw.motors[JointAxis::AngZ as usize].target_pos = shld_z;
    }
}
