use bevy::prelude::{App, Bundle, Component, Entity, Last, Plugin};
use bevy::utils::default;
use k::{Chain, Error, InverseKinematicsSolver, Isometry3, JacobianIkSolver, NodeBuilder, RealField, SerialChain, SubsetOf};
use crate::arm::{ArmInfo, BodySegment, CapsuleSegment};
use crate::ik_systems::initialize_ik_arms;


pub struct IKPlugin;

impl Plugin for IKPlugin {
    fn build(&self, app: &mut App) {
        use bevy_trait_query::RegisterExt;

        app
            .register_component_as::<dyn BodySegment, CapsuleSegment>()
            .add_systems(Last, initialize_ik_arms);
    }
}


#[derive(Component)]
pub struct UninitializedIKArm;

#[derive(Component)]
pub struct IKArm<T: RealField> {
    pub arm_chain: SerialChain<T>,
    pub ik_solver: JacobianIkSolver<T>,
}

impl<T> IKArm<T>
where
    T: RealField + SubsetOf<f64>
{
    pub fn solve(&mut self, target: Isometry3<T>) -> Result<(), Error> {
        self.ik_solver.solve(&self.arm_chain, &target)
    }
}

impl<T> Default for IKArm<T> 
where 
    T: RealField + SubsetOf<f64>
{
    fn default() -> Self {
        Self {
            arm_chain: SerialChain::new_unchecked(Chain::from_root(
                NodeBuilder::new().finalize().into()
            )),
            ik_solver: JacobianIkSolver::default()
        }
    }
}

#[derive(Bundle)]
pub struct UninitIKArmBundle<T: RealField> {
    pub arm_info: ArmInfo,
    pub ik_arm: IKArm<T>,
    pub _uninit: UninitializedIKArm
}

impl<T> UninitIKArmBundle<T> 
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
            _uninit: UninitializedIKArm,
        }
    }
}

impl<T> Default for UninitIKArmBundle<T>
where
    T: RealField + SubsetOf<f64>
{
    fn default() -> Self {
        Self {
            arm_info: ArmInfo::default(),
            ik_arm: IKArm::default(),
            _uninit: UninitializedIKArm
        }
    }
}
