use bevy::prelude::{App, Bundle, Component, Entity, Last, Plugin};
use bevy::utils::default;
use bevy_rapier3d::na::Vector3;
use k::{Chain, connect, Error, InverseKinematicsSolver, Isometry3, JacobianIkSolver, JointType, NodeBuilder, RealField, SerialChain, SubsetOf};
use crate::arm::{ArmInfo, BodySegment, CapsuleSegment};
use crate::ik_systems::set_ik_arm_positions;


pub struct IKPlugin;

impl Plugin for IKPlugin {
    fn build(&self, app: &mut App) {
        use bevy_trait_query::RegisterExt;

        app
            .register_component_as::<dyn BodySegment, CapsuleSegment>()
            .add_systems(Last, set_ik_arm_positions);
    }
}


#[derive(Component)]
pub struct IKArm<T: RealField> {
    pub chain: SerialChain<T>,
    pub ik_solver: JacobianIkSolver<T>,
    pub target_pos: Option<Vector3<T>>,
    pub elbow_ik_pole: Option<Vector3<T>>,
}

impl<T> IKArm<T>
where
    T: RealField + SubsetOf<f64>
{
    pub fn solve(&mut self, target: Isometry3<T>) -> Result<(), Error> {
        self.ik_solver.solve(&self.chain, &target)
    }
}

impl<T> Default for IKArm<T> 
where 
    T: RealField + SubsetOf<f64>
{
    fn default() -> Self {
        let fixed: k::Node<T> = NodeBuilder::new()
            .name("fixed")
            .joint_type(JointType::Fixed)
            .finalize()
            .into();
        let l0: k::Node<T> = NodeBuilder::new()
            .name("shoulder_x_rot")
            .joint_type(JointType::Rotational {
                axis: k::Vector3::x_axis()
            })
            .finalize()
            .into();
        let l1: k::Node<T> = NodeBuilder::new()
            .name("shoulder_y_rot")
            .joint_type(JointType::Rotational {
                axis: k::Vector3::y_axis()
            })
            .finalize()
            .into();
        let l2: k::Node<T> = NodeBuilder::new()
            .name("shoulder_z_rot")
            .joint_type(JointType::Rotational {
                axis: k::Vector3::z_axis()
            })
            .finalize()
            .into();
        let l3: k::Node<T> = NodeBuilder::new()
            .name("elbow_y_rot")
            .joint_type(JointType::Rotational {
                axis: k::Vector3::y_axis()
            })
            .finalize()
            .into();
        let l4: k::Node<T> = NodeBuilder::new()
            .name("wrist")
            .finalize()
            .into();
        connect![fixed => l0 => l1 => l2 => l3 => l4];

        Self {
            chain: SerialChain::new_unchecked(k::Chain::from_root(fixed)),
            ik_solver: JacobianIkSolver::default(),
            elbow_ik_pole: None,
            target_pos: None,
        }
    }
}

#[derive(Bundle)]
pub struct IKArmBundle<T: RealField> {
    pub arm_info: ArmInfo,
    pub ik_arm: IKArm<T>,
}

impl<T> IKArmBundle<T>
where 
    T: RealField + SubsetOf<f64>
{
    pub fn new(upper_arm: Entity, lower_arm: Entity) -> Self {
        Self {
            arm_info: ArmInfo {
                upper_arm,
                lower_arm
            },
            ik_arm: default(),
        }
    }
}

impl<T> Default for IKArmBundle<T>
where
    T: RealField + SubsetOf<f64>
{
    fn default() -> Self {
        Self {
            arm_info: default(),
            ik_arm: default(),
        }
    }
}
