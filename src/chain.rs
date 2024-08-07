use std::slice::Iter;

use bevy_rapier3d::{math::Real, na::Isometry3};

use crate::node::{KError, KJointRef, KNode};

#[derive(Default)]
pub struct SerialKChain {
    nodes: Vec<KNode>
}

impl SerialKChain {
    pub fn from_root(root: &KNode) -> Self {
        let mut chain = SerialKChain::default();
        chain.nodes.push(root.clone());

        for child in root.iter_children() {
            chain.nodes.push(child);
        }

        chain
    }

    pub fn iter_joints(&self) -> impl Iterator<Item = KJointRef<'_>> {
        self.nodes.iter().map(|n| n.joint())
    }

    pub fn iter(&self) -> Iter<KNode> {
        self.nodes.iter()
    }

    pub fn end(&self) -> Option<&KNode> {
        self.nodes.last()
    }

    pub fn root(&self) -> Option<&KNode> {
        self.nodes.first()
    }

    pub fn get_node(&self, idx: usize) -> Option<&KNode> {
        self.nodes.get(idx)
    }

    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    pub fn set_joint_positions(&self, positions: &[Real]) -> Result<(), KError> {
        if positions.len() != self.len() {
            return Err(KError::SizeMismatchError { input_size: positions.len(), required_size: self.len() })
        }

        for (node, pos) in self.nodes.iter().zip(positions.iter()) {
            node.joint().set_position(*pos)?;
        }
        Ok(())
    }

    pub fn set_joint_positions_deg(&self, positions: &[Real]) -> Result<(), KError> {
        if positions.len() != self.len() {
            return Err(KError::SizeMismatchError { input_size: positions.len(), required_size: self.len() })
        }

        for (node, pos) in self.nodes.iter().zip(positions.iter()) {
            node.joint().set_position(pos.to_radians())?;
        }
        Ok(())
    }

    pub fn update_world_transforms(&mut self) {
        for (i, node) in self.iter().enumerate() {
            let mut curr_joint = node.joint();

            let mut transform = Isometry3::identity();
            for joint in self.iter_joints().take(i) {
                transform *= joint.local_transform();
            }
            transform *= curr_joint.local_transform();

            curr_joint.world_transform_cache = Some(transform);
        }
    }
}
