use bevy_rapier3d::na::Isometry3;

use crate::node::{KJointRef, KJointRefMut, KNode};

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

    pub fn iter_joints_mut(&mut self) -> impl Iterator<Item = KJointRefMut<'_>> {
        self.nodes.iter_mut().map(|n| n.joint_mut())
    }

    pub fn iter(&self) -> impl Iterator<Item = &KNode> {
        self.nodes.iter()
    }

    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    pub fn update_world_transforms(&mut self) {
        for (i, node) in self.iter().enumerate() {
            let mut curr_joint = node.joint_mut();

            let mut transform = Isometry3::identity();
            for joint in self.iter_joints().take(i) {
                transform *= joint.local_transform();
            }
            transform *= curr_joint.local_transform();

            curr_joint.world_transform_cache = Some(transform);
        }
    }
}
