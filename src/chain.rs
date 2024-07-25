use crate::node::{KJointRef, KJointRefMut, KNode};

pub struct SerialKChain {
    nodes: Vec<KNode>
}

impl SerialKChain {
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
}
