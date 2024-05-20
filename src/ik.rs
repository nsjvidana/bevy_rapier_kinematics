use bevy::prelude::{Bundle, Component};
use k::{Error, InverseKinematicsSolver, Isometry3, JacobianIkSolver, RealField, SerialChain, SubsetOf};
use crate::arm::ArmInfo;

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

#[derive(Bundle)]
pub struct IKArmBundle<T: RealField> {
    pub arm_info: ArmInfo,
    pub ik_arm: IKArm<T>
}
