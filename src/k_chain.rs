use bevy_rapier3d::na::{Isometry3, RealField, Translation3};
use k::InverseKinematicsSolver;

pub struct SerialKChain<T: RealField> {
    pub nodes: Vec<KNode<T>>
}

pub struct KNode<T: RealField> {
    pub origin: Isometry3<T>,
    pub world_transform: Isometry3<T>,
}

pub struct FabrikSolver<T: RealField> {
    pub allowable_target_distance: T,
    pub num_max_iterations: usize
}

impl<T> FabrikSolver<T>
where
    T: RealField
{
    pub fn solve(serial_chain: &mut SerialKChain<T>, target: Isometry3<T>) {
        let nodes = &mut serial_chain.nodes;
        let mut segment_lens = Vec::<T>::with_capacity(nodes.len()-1);
        for i in 0..(nodes.len()-1) {
            segment_lens.push(nodes[i].origin.translation.vector.metric_distance(&nodes[i+1].origin.translation.vector));
        }
        for node in nodes.iter_mut() {
            node.world_transform = node.origin.clone();
        }

        let node_count = nodes.len();
        nodes[node_count].world_transform.translation = target.translation;
        
        // |   | ||
        for i in (1..node_count-1).rev() {
            let child = &nodes[i+1].world_transform.translation.vector;
            let curr = &nodes[i].world_transform.translation.vector;
            let mut child_to_curr = curr-child.clone();
                child_to_curr.set_magnitude(segment_lens[i+1].clone());
            nodes[i].world_transform.translation = (child + child_to_curr).into();
        }

        for i in 1..node_count-1 {
            let parent = &nodes[i-1].world_transform.translation.vector;
            let curr = &nodes[i].world_transform.translation.vector;
            let mut parent_to_curr = curr-parent.clone();
                parent_to_curr.set_magnitude(segment_lens[i-1].clone());
            nodes[i].world_transform.translation = (parent + parent_to_curr).into();
        }

        let second_to_last = &nodes[node_count-2].world_transform.translation.vector;
        let end = &nodes[node_count-1].world_transform.translation.vector;
        let mut end_segment_axis = end - second_to_last;
            end_segment_axis.set_magnitude(segment_lens.last().unwrap().clone());
        nodes[node_count-1].world_transform.translation = (second_to_last + end_segment_axis).into();
        
    }
}
