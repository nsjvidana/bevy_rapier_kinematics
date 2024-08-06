use std::{cell::RefCell, sync::Arc};

use bevy::{color::Color, math::Vec3, prelude::{App, Gizmos, Plugin}, utils::default};
use bevy_rapier3d::{math::Real, na::{Isometry3, Translation3, UnitQuaternion, Vector3}};

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

    pub fn backwards_solve(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>, gizmos: &mut Gizmos) {
        let mut inv_chain = Vec::with_capacity(chain.len());

        {
            let end = chain.end().unwrap().joint();

            let mut inv_root_child_joint = chain.get_node(chain.len()-2).unwrap().joint().clone();
                inv_root_child_joint.set_origin(*end.origin());

            let end_space = {
                let mut transform = Isometry3::identity();
                for node in chain.iter().take(chain.len()-1) {
                    transform *= node.joint().local_transform()
                }
                transform * end.local_transform()
            };

            let root_space = Isometry3 {
                translation: target_pose.translation,
                ..default()
            };

            let end_local = root_space.inv_mul(&end_space);
            let inv_root_child_local = inv_root_child_joint.local_transform();
            let inv_root_rot = rotation_between_vectors(&inv_root_child_local.translation.vector, &end_local.translation.vector);
            
            
            let mut inv_root_joint = end.clone();
                inv_root_joint.set_origin(Isometry3 {
                    translation: target_pose.translation,
                    rotation: inv_root_rot
                });
            inv_chain.push(RefCell::new(KNodeData {
                joint: inv_root_joint,
                ..Default::default()
            }));
            
            inv_chain.push(RefCell::new(KNodeData {
                joint: inv_root_child_joint,
                ..Default::default()
            }));
        }

        for (i, curr_node) in chain.iter().take(chain.len()-1).enumerate().rev() {
            if i == 0 { break; }
            
            let mut curr_joint = curr_node.joint();
            match curr_joint.joint_type() {
                KJointType::Fixed => { continue; }
                KJointType::Revolute { axis } => {
                    let parent = chain.get_node(i-1).unwrap(); //parent of curr_node

                    let mut inv_seg_child_joint = parent.joint().clone();
                        inv_seg_child_joint.set_origin(*curr_joint.origin());

                    let parent_space = {
                        let mut transform = Isometry3::identity();
                        for node in chain.iter().take(i) {
                            transform *= node.joint().local_transform();
                        }
                        transform
                    };
                    
                    //root of the last segment of the inverse chain
                    let inv_seg_root_space = {
                        let mut transform = Isometry3::identity();
                        //at this point, the new segment isn't added yet, so iter() works.
                        for inv_node in inv_chain.iter() {
                            transform *= inv_node.borrow().joint.local_transform();
                        }
                        transform
                    };
                    
                    let inv_seg_child_local = inv_seg_child_joint.local_transform();
                    let parent_local = inv_seg_root_space.inv_mul(&parent_space);
                    let adjustment = angle_between(
                        &project_onto_plane(&inv_seg_child_local.translation.vector, axis),
                        &project_onto_plane(&parent_local.translation.vector, axis),
                        axis
                    );
                    inv_seg_child_joint.increment_position(adjustment);
                    curr_joint.increment_position(-adjustment);

                    inv_chain.push(RefCell::new(KNodeData {
                        joint: inv_seg_child_joint.clone(),
                        ..Default::default()
                    }));
                    
                },
                #[allow(unused)]
                KJointType::Linear { axis } => todo!()
            }
        }
        
        {//drawing gizmos
            // for (i, node) in inv_chain.iter().enumerate() {
            //     let curr_joint = &mut node.borrow_mut().joint;
    
            //     let mut transform = Isometry3::identity();
            //     for joint in inv_chain.iter().take(i) {
            //         transform *= joint.borrow().joint.local_transform();
            //     }
            //     transform *= curr_joint.local_transform();
    
            //     curr_joint.world_transform_cache = Some(transform);
            // }

            // let mut prev = inv_chain[0].borrow().joint.local_transform().translation.into();
            // let color = Color::linear_rgb(0., 1., 0.);
            // for node in inv_chain.iter().skip(1) {
            //     let joint = &node.borrow().joint;
            //     let joint_pos: Vec3 = joint.world_transform().unwrap().translation.into();
                
            //     gizmos.sphere(
            //         joint_pos,
            //         default(),
            //         0.05,
            //         color
            //     );
            //     gizmos.line(prev, joint_pos, color);
            //     prev = joint_pos;
            // }
        }

        
    }
}
