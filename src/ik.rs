use std::{cell::RefCell, sync::Arc};

use bevy::{color::Color, math::Vec3, prelude::{App, Gizmos, Plugin, TransformBundle}, utils::default};
use bevy_egui::node;
use bevy_rapier3d::{math::Real, na::{Isometry3, Translation3, UnitQuaternion, Vector3}};

use crate::{chain::SerialKChain, math_utils::{angle_to, project_onto_plane, rotation_between_vectors}, node::{KError, KJointRef, KJointType, KNodeData}};

pub struct IKPlugin;

impl Plugin for IKPlugin {
    fn build(&self, _app: &mut App) {

    }
}

macro_rules! cyclic_impl {
    ($struct_name:ident) => {

        pub struct $struct_name {
            pub allowable_target_distance: Real,
            pub allowable_target_angle: Real,
            pub max_iterations: usize,
            /// 0.0 for no damping, 1.0 for damping
            pub per_joint_dampening: Real
        }

        impl Default for $struct_name {
            fn default() -> Self {
                Self {
                    allowable_target_distance: 0.1,
                    allowable_target_angle: 0.1,
                    max_iterations: 1,
                    per_joint_dampening: 0.
                }
            }
        }
    };
}

cyclic_impl!(CyclicIKSolver);
cyclic_impl!(ForwardDescentCyclic);
cyclic_impl!(ForwardAscentCyclic);
cyclic_impl!(BackwardsCyclic);

impl CyclicIKSolver {
    
    pub fn forward_ascent(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>, _: Option<&mut Gizmos>) -> Result<(), KError> {
        let mut dist_to_target = 0.;
        let mut angle_to_target = 0.;
        
        let mut target_reached = false;
        for _ in 0..self.max_iterations {
            for (i, node) in chain.iter().enumerate() {
                let mut curr_joint = node.joint();
                
                let joint_space = {
                    let mut transform = Isometry3::identity();
                    for node in chain.iter().take(i) {
                        transform *= node.joint().local_transform();
                    }
                    transform *= curr_joint.local_transform();
                    transform
                };

                let joint_axis = match curr_joint.joint_type() {
                    KJointType::Revolute { axis } => axis,
                    KJointType::Fixed => continue,
                    #[allow(unused)]
                    KJointType::Linear { axis } => todo!()
                };

                let local_end = {
                    let mut transform = Isometry3::identity();
                    for node in chain.iter().skip(i+1) {
                        transform *= node.joint().local_transform()
                    }
                    transform
                };
                let local_target = joint_space.inverse() * target_pose;

                let end_projected = project_onto_plane(&local_end.translation.vector, joint_axis);
                let target_projected = project_onto_plane(&local_target.translation.vector, joint_axis);
                let adjustment = angle_to(&end_projected, &target_projected, joint_axis);
                curr_joint.increment_position(adjustment * (1. - self.per_joint_dampening));
            }
            
            let end_world_space = {
                let mut transform = chain.get_node(0).unwrap().joint().local_transform();
                for node in chain.iter().skip(1) {
                    transform *= node.joint().local_transform();
                }
                transform
            };
            dist_to_target = end_world_space.translation.vector.metric_distance(&target_pose.translation.vector);
            angle_to_target = end_world_space.rotation.angle_to(&target_pose.rotation);
            if dist_to_target <= self.allowable_target_distance &&
                angle_to_target <= self.allowable_target_angle
            {
                target_reached = true;
                break;
            }
        }

        if !target_reached {
            Err(KError::SolverNotConverged {
                solver_type: "Cyclic".into(),
                num_tries: self.max_iterations,
                position_diff: dist_to_target,
                angle_diff: angle_to_target
            })
        }
        else {
            Ok(())
        }
    }

    pub fn forward_descent(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>, _: Option<&mut Gizmos>) -> Result<(), KError> {
        let mut dist_to_target = 0.;
        let mut angle_to_target = 0.;

        let mut target_reached = false;
        for _ in 0..self.max_iterations {
            
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

                let joint_axis = match curr_joint.joint_type() {
                    KJointType::Revolute { axis } => axis,
                    KJointType::Fixed => continue,
                    #[allow(unused)]
                    KJointType::Linear { axis } => todo!()
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
                let adjustment = angle_to(&end_projected, &target_projected, joint_axis);
                curr_joint.increment_position(adjustment * (1. - self.per_joint_dampening));
            }
            
            
            let end_world_space = {
                let mut transform = chain.get_node(0).unwrap().joint().local_transform();
                for node in chain.iter().skip(1) {
                    transform *= node.joint().local_transform();
                }
                transform
            };
            dist_to_target = end_world_space.translation.vector.metric_distance(&target_pose.translation.vector);
            angle_to_target = end_world_space.rotation.angle_to(&target_pose.rotation);
            if dist_to_target <= self.allowable_target_distance &&
                angle_to_target <= self.allowable_target_angle
            {
                target_reached = true;
                break;
            }
        }

        if !target_reached {
            Err(KError::SolverNotConverged {
                solver_type: "Cyclic".into(),
                num_tries: self.max_iterations,
                position_diff: dist_to_target,
                angle_diff: angle_to_target
            })
        }
        else {
            Ok(())
        }
    }

    pub fn backwards_solve(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>, debug_gizmos: Option<&mut Gizmos>) -> Result<(), KError> {
        //only compute distance to target because this solver doesn't solve for target rotation
        let mut dist_to_target = 0.;
        
        let mut inv_chain = Vec::with_capacity(chain.len());

        let mut target_reached = false;
        for _ in 0..self.max_iterations {
            inv_chain.clear();
            {//preparing inv chain with a root that has the best rotation
                let mut end_joint = chain.end().unwrap().joint();

                let mut inv_root_child_joint = chain.get_node(chain.len()-2).unwrap().joint().clone();
                    inv_root_child_joint.set_origin(*end_joint.origin());

                let end_space = {
                    let mut transform = Isometry3::identity();
                    for node in chain.iter().take(chain.len()-1) {
                        transform *= node.joint().local_transform()
                    }
                    transform * end_joint.local_transform()
                };

                let mut inv_root_space = Isometry3 {
                    translation: target_pose.translation,
                    ..default()
                };

                let end_local = inv_root_space.inv_mul(&end_space);
                let inv_root_child_local = inv_root_child_joint.local_transform();

                let mut inv_root_joint = end_joint.clone();
                match inv_root_joint.joint_type() {
                    KJointType::Fixed => {
                        inv_root_space.rotation = rotation_between_vectors(&inv_root_child_local.translation.vector, &end_local.translation.vector);
                    },
                    KJointType::Revolute { axis } => {
                        let adjustment = angle_to(
                            &project_onto_plane(&inv_root_child_local.translation.vector, axis),
                            &project_onto_plane(&end_local.translation.vector, axis),
                            axis
                        );
                        inv_root_joint.increment_position(adjustment);
                        end_joint.increment_position(-adjustment);
                    },
                    //i might just make linear the same as fixed.
                    #[allow(unused)]
                    KJointType::Linear { axis } => todo!()
                }
                inv_root_joint.set_origin(inv_root_space);
                
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
                        let adjustment = angle_to(
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
            
            let end_space;

            {//do forwards-solve for the root so that it contributes to the motion of the chain
                let mut root_joint = chain.root().unwrap().joint();

                let root_space = root_joint.local_transform();
                
                end_space = {
                    let mut transform = root_space;
                    for node in chain.iter().skip(1) {
                        transform *= node.joint().local_transform();
                    }
                    transform
                };
                let end_local = root_space.inv_mul(&end_space);
                let target_local = root_space.inv_mul(&target_pose);

                match root_joint.joint_type() {
                    KJointType::Fixed => {}
                    KJointType::Revolute { axis } => {
                        let adjustment = angle_to(
                            &project_onto_plane(&end_local.translation.vector, axis),
                            &project_onto_plane(&target_local.translation.vector, axis),
                            axis
                        );
                        root_joint.increment_position(adjustment);
                    },
                    #[allow(unused)]
                    KJointType::Linear { axis } => todo!()
                }
            }


            dist_to_target = end_space.translation.vector.metric_distance(&target_pose.translation.vector);
            if dist_to_target <= self.allowable_target_distance {
                target_reached = true;
                break;
            }
        }

        if let Some(gizmos) = debug_gizmos {//drawing inverse chain
            for (i, node) in inv_chain.iter().enumerate() {
                let curr_joint = &mut node.borrow_mut().joint;
    
                let mut transform = Isometry3::identity();
                for joint in inv_chain.iter().take(i) {
                    transform *= joint.borrow().joint.local_transform();
                }
                transform *= curr_joint.local_transform();
    
                curr_joint.world_transform_cache = Some(transform);
            }

            let mut prev = inv_chain[0].borrow().joint.local_transform().translation.into();
            let color = Color::linear_rgb(0., 1., 0.);
            for node in inv_chain.iter().skip(1) {
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
        
        if !target_reached {
            Err(KError::SolverNotConverged {
                solver_type: "Cyclic".into(),
                num_tries: self.max_iterations,
                position_diff: dist_to_target,
                angle_diff: 0.
            })
        }
        else {
            Ok(())
        }
    }

}

pub trait IKSolver {
    fn solve(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>) -> Result<(), KError>;
    #[allow(unused_variables)]
    fn solve_debug(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>, debug_gizmos: Option<&mut Gizmos>) -> Result<(), KError> {
        unimplemented!()
    }
    fn solver_name(&self) -> &str;
}

impl IKSolver for ForwardAscentCyclic {
    fn solve(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>) -> Result<(), KError> {
        let mut dist_to_target = 0.;
        let mut angle_to_target = 0.;
        
        for _ in 0..self.max_iterations {
            for (i, node) in chain.iter().enumerate() {
                let mut curr_joint = node.joint();
                
                let joint_space = {
                    let mut transform = Isometry3::identity();
                    for node in chain.iter().take(i) {
                        transform *= node.joint().local_transform();
                    }
                    transform *= curr_joint.local_transform();
                    transform
                };

                let joint_axis = match curr_joint.joint_type() {
                    KJointType::Revolute { axis } => axis,
                    KJointType::Fixed => continue,
                    #[allow(unused)]
                    KJointType::Linear { axis } => todo!()
                };

                let local_end = {
                    let mut transform = Isometry3::identity();
                    for node in chain.iter().skip(i+1) {
                        transform *= node.joint().local_transform()
                    }
                    transform
                };
                let local_target = joint_space.inverse() * target_pose;

                let end_projected = project_onto_plane(&local_end.translation.vector, joint_axis);
                let target_projected = project_onto_plane(&local_target.translation.vector, joint_axis);
                let adjustment = angle_to(&end_projected, &target_projected, joint_axis);
                curr_joint.increment_position(adjustment * (1. - self.per_joint_dampening));
            }
            
            let end_world_space = {
                let mut transform = chain.get_node(0).unwrap().joint().local_transform();
                for node in chain.iter().skip(1) {
                    transform *= node.joint().local_transform();
                }
                transform
            };
            dist_to_target = end_world_space.translation.vector.metric_distance(&target_pose.translation.vector);
            angle_to_target = end_world_space.rotation.angle_to(&target_pose.rotation);
            if dist_to_target <= self.allowable_target_distance &&
                angle_to_target <= self.allowable_target_angle
            {
                return Ok(());
            }
        }

        Err(KError::SolverNotConverged {
            solver_type: "Cyclic".into(),
            num_tries: self.max_iterations,
            position_diff: dist_to_target,
            angle_diff: angle_to_target
        })
    }

    fn solver_name(&self) -> &str {
        "Forward Ascent Cyclic"
    }
}

impl IKSolver for ForwardDescentCyclic {
    fn solve(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>) -> Result<(), KError> {
        let mut dist_to_target = 0.;
        let mut angle_to_target = 0.;

        for _ in 0..self.max_iterations {
            
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

                let joint_axis = match curr_joint.joint_type() {
                    KJointType::Revolute { axis } => axis,
                    KJointType::Fixed => continue,
                    #[allow(unused)]
                    KJointType::Linear { axis } => todo!()
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
                let adjustment = angle_to(&end_projected, &target_projected, joint_axis);
                curr_joint.increment_position(adjustment * (1. - self.per_joint_dampening));
            }
            
            
            let end_world_space = {
                let mut transform = chain.get_node(0).unwrap().joint().local_transform();
                for node in chain.iter().skip(1) {
                    transform *= node.joint().local_transform();
                }
                transform
            };
            dist_to_target = end_world_space.translation.vector.metric_distance(&target_pose.translation.vector);
            angle_to_target = end_world_space.rotation.angle_to(&target_pose.rotation);
            if dist_to_target <= self.allowable_target_distance &&
                angle_to_target <= self.allowable_target_angle
            {
                return Ok(());
            }
        }

        Err(KError::SolverNotConverged {
            solver_type: "Cyclic".into(),
            num_tries: self.max_iterations,
            position_diff: dist_to_target,
            angle_diff: angle_to_target
        })
    }

    fn solver_name(&self) -> &str {
        "Forward Descent Cyclic"
    }
}

impl IKSolver for BackwardsCyclic {
    fn solve(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>) -> Result<(), KError> {
        //only compute distance to target because this solver doesn't solve for target rotation
        let mut dist_to_target = 0.;
        
        let mut inv_chain = Vec::with_capacity(chain.len());

        for _ in 0..self.max_iterations {
            inv_chain.clear();
            {//preparing inv chain with a root that has the best rotation
                let mut end_joint = chain.end().unwrap().joint();

                let mut inv_root_child_joint = chain.get_node(chain.len()-2).unwrap().joint().clone();
                    inv_root_child_joint.set_origin(*end_joint.origin());

                let end_space = {
                    let mut transform = Isometry3::identity();
                    for node in chain.iter().take(chain.len()-1) {
                        transform *= node.joint().local_transform()
                    }
                    transform * end_joint.local_transform()
                };

                let mut inv_root_space = Isometry3 {
                    translation: target_pose.translation,
                    ..default()
                };

                let end_local = inv_root_space.inv_mul(&end_space);
                let inv_root_child_local = inv_root_child_joint.local_transform();

                let mut inv_root_joint = end_joint.clone();
                match inv_root_joint.joint_type() {
                    KJointType::Fixed => {
                        inv_root_space.rotation = rotation_between_vectors(&inv_root_child_local.translation.vector, &end_local.translation.vector);
                    },
                    KJointType::Revolute { axis } => {
                        let adjustment = angle_to(
                            &project_onto_plane(&inv_root_child_local.translation.vector, axis),
                            &project_onto_plane(&end_local.translation.vector, axis),
                            axis
                        );
                        inv_root_joint.increment_position(adjustment);
                        end_joint.increment_position(-adjustment);
                    },
                    //i might just make linear the same as fixed.
                    #[allow(unused)]
                    KJointType::Linear { axis } => todo!()
                }
                inv_root_joint.set_origin(inv_root_space);
                
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
                        let adjustment = angle_to(
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
            
            let end_space;

            {//do forwards-solve for the root so that it contributes to the motion of the chain
                let mut root_joint = chain.root().unwrap().joint();

                let root_space = root_joint.local_transform();
                
                end_space = {
                    let mut transform = root_space;
                    for node in chain.iter().skip(1) {
                        transform *= node.joint().local_transform();
                    }
                    transform
                };
                let end_local = root_space.inv_mul(&end_space);
                let target_local = root_space.inv_mul(&target_pose);

                match root_joint.joint_type() {
                    KJointType::Fixed => {}
                    KJointType::Revolute { axis } => {
                        let adjustment = angle_to(
                            &project_onto_plane(&end_local.translation.vector, axis),
                            &project_onto_plane(&target_local.translation.vector, axis),
                            axis
                        );
                        root_joint.increment_position(adjustment);
                    },
                    #[allow(unused)]
                    KJointType::Linear { axis } => todo!()
                }
            }


            dist_to_target = end_space.translation.vector.metric_distance(&target_pose.translation.vector);
            if dist_to_target <= self.allowable_target_distance {
                return Ok(());
            }
        }

        Err(KError::SolverNotConverged {
            solver_type: "Cyclic".into(),
            num_tries: self.max_iterations,
            position_diff: dist_to_target,
            angle_diff: 0.
        })
    }
    fn solve_debug(&self, chain: &mut SerialKChain, target_pose: Isometry3<Real>, debug_gizmos: Option<&mut Gizmos>) -> Result<(), KError> {
        //only compute distance to target because this solver doesn't solve for target rotation
        let mut dist_to_target = 0.;
        
        let mut inv_chain = Vec::with_capacity(chain.len());

        for _ in 0..self.max_iterations {
            inv_chain.clear();
            {//preparing inv chain with a root that has the best rotation
                let mut end_joint = chain.end().unwrap().joint();

                let mut inv_root_child_joint = chain.get_node(chain.len()-2).unwrap().joint().clone();
                    inv_root_child_joint.set_origin(*end_joint.origin());

                let end_space = {
                    let mut transform = Isometry3::identity();
                    for node in chain.iter().take(chain.len()-1) {
                        transform *= node.joint().local_transform()
                    }
                    transform * end_joint.local_transform()
                };

                let mut inv_root_space = Isometry3 {
                    translation: target_pose.translation,
                    ..default()
                };

                let end_local = inv_root_space.inv_mul(&end_space);
                let inv_root_child_local = inv_root_child_joint.local_transform();

                let mut inv_root_joint = end_joint.clone();
                match inv_root_joint.joint_type() {
                    KJointType::Fixed => {
                        inv_root_space.rotation = rotation_between_vectors(&inv_root_child_local.translation.vector, &end_local.translation.vector);
                    },
                    KJointType::Revolute { axis } => {
                        let adjustment = angle_to(
                            &project_onto_plane(&inv_root_child_local.translation.vector, axis),
                            &project_onto_plane(&end_local.translation.vector, axis),
                            axis
                        );
                        inv_root_joint.increment_position(adjustment);
                        end_joint.increment_position(-adjustment);
                    },
                    //i might just make linear the same as fixed.
                    #[allow(unused)]
                    KJointType::Linear { axis } => todo!()
                }
                inv_root_joint.set_origin(inv_root_space);
                
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
                        let adjustment = angle_to(
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
            
            let end_space;

            {//do forwards-solve for the root so that it contributes to the motion of the chain
                let mut root_joint = chain.root().unwrap().joint();

                let root_space = root_joint.local_transform();
                
                end_space = {
                    let mut transform = root_space;
                    for node in chain.iter().skip(1) {
                        transform *= node.joint().local_transform();
                    }
                    transform
                };
                let end_local = root_space.inv_mul(&end_space);
                let target_local = root_space.inv_mul(&target_pose);

                match root_joint.joint_type() {
                    KJointType::Fixed => {}
                    KJointType::Revolute { axis } => {
                        let adjustment = angle_to(
                            &project_onto_plane(&end_local.translation.vector, axis),
                            &project_onto_plane(&target_local.translation.vector, axis),
                            axis
                        );
                        root_joint.increment_position(adjustment);
                    },
                    #[allow(unused)]
                    KJointType::Linear { axis } => todo!()
                }
            }


            dist_to_target = end_space.translation.vector.metric_distance(&target_pose.translation.vector);
            if dist_to_target <= self.allowable_target_distance {
                return Ok(());
            }
        }

        if let Some(gizmos) = debug_gizmos {//drawing inverse chain
            for (i, node) in inv_chain.iter().enumerate() {
                let curr_joint = &mut node.borrow_mut().joint;
    
                let mut transform = Isometry3::identity();
                for joint in inv_chain.iter().take(i) {
                    transform *= joint.borrow().joint.local_transform();
                }
                transform *= curr_joint.local_transform();
    
                curr_joint.world_transform_cache = Some(transform);
            }

            let mut prev = inv_chain[0].borrow().joint.local_transform().translation.into();
            let color = Color::linear_rgb(0., 1., 0.);
            for node in inv_chain.iter().skip(1) {
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
        
        Err(KError::SolverNotConverged {
            solver_type: "Cyclic".into(),
            num_tries: self.max_iterations,
            position_diff: dist_to_target,
            angle_diff: 0.
        })
    }

    fn solver_name(&self) -> &str {
        "Backwards Cyclic"
    }
}
