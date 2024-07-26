use crate::node::KNode;

pub struct KNodeChildren {
    curr_node: KNode
}

impl KNodeChildren {
    pub fn new(node: KNode) -> Self {
        Self {
            curr_node: node
        }
    }
}

impl Iterator for KNodeChildren {
    type Item = KNode;
    
    fn next(&mut self) -> Option<Self::Item> {
        self.curr_node.0.borrow().child.clone()
    }
}
