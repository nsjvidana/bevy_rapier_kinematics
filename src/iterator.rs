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
        let child_opt = self.curr_node.0.lock().child.clone();
        match child_opt {
            Some(child) => {
                self.curr_node = child.clone();
                Some(self.curr_node.clone())
            },
            None => None
        }
    }
}
