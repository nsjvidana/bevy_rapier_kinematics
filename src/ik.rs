use std::{cell::RefCell, sync::Arc};

use bevy::{color::Color, math::Vec3, prelude::{App, Gizmos, Plugin}, utils::default};
use bevy_rapier3d::{math::Real, na::{Isometry3, Translation3, UnitQuaternion}};

use crate::{chain::SerialKChain, math_utils::{angle_between, project_onto_plane, rotation_between_vectors}, node::{KError, KJointType, KNodeData}};


pub struct IKPlugin;

impl Plugin for IKPlugin {
    fn build(&self, _app: &mut App) {

    }
}

pub struct CyclicIKSolver {
    pub allowable_target_distance: Real,
    pub allowable_target_angle: Real,
    pub max_iterations: usize,
    /// 0.0 for no damping, 1.0 for damping
    pub per_joint_dampening: Real
}

impl CyclicIKSolver {
    pub fn solve(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>) -> Result<(), KError>{
        let mut dist_to_target = 0.;
        let mut angle_to_target = 0.;

        for _ in 0..self.max_iterations {
            
            //store the end effector's world space transform to avoid having to compute it twice
            let mut end_world_space = None;
            
            for (i, node) in chain.iter().enumerate().rev() {
                let mut curr_joint = node.joint();
                
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

                let joint_axis = match curr_joint.joint_type() {
                    KJointType::Revolute { axis } => axis,
                    KJointType::Fixed => continue,
                    _ => return Err(KError::SolverIncompatibleWithJointType {
                        joint_name: curr_joint.name.clone(),
                        joint_type: format!("{:?}", curr_joint.joint_type()),
                        solver_type: "Cyclic".into()
                    })
                };

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
                    angle * (1.-self.per_joint_dampening)
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

    pub fn backwards_solve(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>, gizmos: &mut Gizmos) -> Result<(), KError>{
        let mut dist_to_target = 0.;
        let mut angle_to_target = 0.;

        let mut inv_chain = Vec::with_capacity(chain.len());

        for _ in 0..self.max_iterations {
            
            inv_chain.clear();
            {// create root for inv chain at the target pose.
                let end_joint = chain.get_node(chain.len()-1).unwrap().joint();
                let end_joint_space = {
                    let mut transform = Isometry3::identity();
                    for joint in chain.iter_joints().take(chain.len()-1) {
                        transform *= joint.local_transform();
                    }
                    transform * end_joint.local_transform()
                };

                let local_inv_root_child = end_joint.local_transform();
                let local_end_joint = target_pose.inv_mul(&end_joint_space);

                let mut inv_root_joint = end_joint.clone();
                let inv_root_rotation = match inv_root_joint.joint_type() {
                    KJointType::Fixed => {
                        rotation_between_vectors(&local_inv_root_child.translation.vector, &local_end_joint.translation.vector)
                    },
                    KJointType::Revolute { axis } => {
                        UnitQuaternion::from_axis_angle(
                            axis,
                            angle_between(
                                &project_onto_plane(&local_inv_root_child.translation.vector, axis),
                                &project_onto_plane(&local_end_joint.translation.vector, axis),
                                axis
                            )
                        )
                    },
                    KJointType::Linear { axis } => todo!()
                };
                inv_root_joint.set_origin(Isometry3 {
                    rotation: inv_root_rotation,
                    ..target_pose
                });

                let inv_root = KNodeData {
                    joint: inv_root_joint,
                    ..Default::default()
                };
                inv_chain.push(RefCell::new(inv_root));
            }
        
            for (i, node) in chain.iter().enumerate().rev() {
                if i == 0 { break; } //if we are at the root, end this iteration.
                
                let mut curr_joint = node.joint();
                let inv_node_idx = chain.len()-i-1;
                
                //attempt to attach a segment to the inverse chain
                //doing i-1 is safe because we already made sure that the iteration ends when i==0.
                let mut inv_child_joint = chain.get_node(i-1).unwrap().joint().clone();
                inv_child_joint.set_origin(curr_joint.local_transform());
                inv_chain.push(RefCell::new(KNodeData {
                    joint: inv_child_joint,
                    ..Default::default()
                }));

                if let KJointType::Fixed = curr_joint.joint_type() { continue; }

                let mut inv_node = inv_chain.get(inv_node_idx).unwrap().borrow_mut();
                
                let inv_node_space = {
                    let mut transform = Isometry3::identity();
                    for node in inv_chain.iter().take(inv_node_idx) {
                        transform *= node.borrow().joint.local_transform();
                    }
                    transform * inv_node.joint.local_transform()
                };

                //TODO: prevent aquiring mutex of the parent node twice.
                let parent_space = {
                    let mut transform = Isometry3::identity();
                    for node in chain.iter().take(i) {
                        transform *= node.joint().local_transform();
                    }
                    transform
                };
                let curr_joint_space = parent_space * curr_joint.local_transform();
                

                let par_local_inv = inv_node_space.inv_mul(&parent_space);
                let curr_joint_local_inv = inv_node_space.inv_mul(&curr_joint_space);
                
                match curr_joint.joint_type() {
                    KJointType::Revolute { axis } => {
                        let adjustment = angle_between(
                            &project_onto_plane(&curr_joint_local_inv.translation.vector, axis),
                            &project_onto_plane(&par_local_inv.translation.vector, axis),
                            axis
                        );
                        inv_node.joint.increment_position(adjustment);
                        curr_joint.increment_position(-adjustment);
                    },
                    //TODO: implement linear joints
                    //  (probably by projecting locals directly on the axis and comparing their norms)
                    KJointType::Linear { axis } => todo!(),
                    KJointType::Fixed => continue,
                }
            }

            {//drawing gizmos
                for (i, node) in inv_chain.iter().enumerate() {
                    let curr_joint = &mut node.borrow_mut().joint;
        
                    let mut transform = Isometry3::identity();
                    for joint in inv_chain.iter().take(i) {
                        transform *= joint.borrow().joint.local_transform();
                    }
                    transform *= curr_joint.local_transform();
        
                    curr_joint.world_transform_cache = Some(transform);
                }

                let mut prev = Vec3::ZERO;
                let color = Color::linear_rgb(0., 1., 0.);
                for node in inv_chain.iter() {
                    let joint = &node.borrow().joint;
                    let joint_pos: Vec3 = joint.world_transform().unwrap().translation.into();
                    
                    gizmos.sphere(
                        joint_pos,
                        default(),
                        0.05,
                        color
                    );
                    gizmos.line(prev, joint_pos, color);
                    prev = joint_pos;
                }
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
