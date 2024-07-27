use bevy::prelude::{App, Plugin};
use bevy_rapier3d::{math::Real, na::Isometry3};

use crate::{chain::SerialKChain, math_utils::{angle_between, project_onto_plane}, node::{KError, KJointType}};


pub struct IKPlugin;

impl Plugin for IKPlugin {
    fn build(&self, _app: &mut App) {

    }
}

pub struct CyclicIKSolver {
    pub allowable_target_distance: Real,
    pub allowable_target_angle: Real,
    pub max_iterations: usize
}

impl CyclicIKSolver {
    pub fn solve(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>) -> Result<(), KError>{
        let mut dist_to_target = 0.;
        let mut angle_to_target = 0.;

        for _ in 0..self.max_iterations {
            
            //store the end effector's world space transform to avoid having to compute it twice
            let mut end_world_space = None;
            
            for (i, node) in chain.iter().enumerate().rev() {
                let mut curr_joint = node.joint_mut();
                let joint_axis = match curr_joint.joint_type() {
                    KJointType::Revolute { axis } => axis,
                    KJointType::Fixed => continue,
                    _ => return Err(KError::SolverIncompatibleWithJointType {
                        joint_name: curr_joint.name.clone(),
                        joint_type: format!("{:?}", curr_joint.joint_type()),
                        solver_type: "Cyclic".into()
                    })
                };

                let joint_space = {
                    let mut transform = Isometry3::identity();
                    for node in chain.iter().take(i) {
                        transform *= node.joint().local_transform();
                    }
                    transform *= curr_joint.local_transform();
                    transform
                };
                
                //if this is the end node, we store this joint's world space transform for later use.
                if i == chain.len()-1 {
                    end_world_space = Some(joint_space);
                }

                let local_target = joint_space.inverse() * target_pose;
                let local_end = {
                    let mut transform = Isometry3::identity();
                    for node in chain.iter().skip(i+1) {
                        transform *= node.joint().local_transform()
                    }
                    transform
                };

                let target_projected = project_onto_plane(&local_target.translation.vector, joint_axis);
                let end_projected = project_onto_plane(&local_end.translation.vector, joint_axis);
                //the angle between the projected vectors is the joint's position (limited by joint limits)
                let angle = angle_between(&end_projected, &target_projected, joint_axis);
                curr_joint.set_position_clamped(
                    angle
                );
            }
            
            //skip all the other iterations if we already reached the target pose.  
            let end_world_space = end_world_space.unwrap();
            dist_to_target = end_world_space.translation.vector.metric_distance(&target_pose.translation.vector);
            angle_to_target = end_world_space.rotation.angle_to(&target_pose.rotation);
            if dist_to_target <= self.allowable_target_distance &&
                angle_to_target <= self.allowable_target_angle {
                return Ok(())
            }
        }

        Err(KError::SolverNotConverged {
            solver_type: "Cyclic".into(),
            num_tries: self.max_iterations,
            position_diff: dist_to_target,
            angle_diff: angle_to_target
        })
    }
}
