use std::{cell::RefCell, ops::{Deref, DerefMut}, rc::{Rc, Weak}};

use derivative::Derivative;
use thiserror::Error;
use bevy_rapier3d::{math::Real, na::{Isometry3, Translation3, UnitQuaternion, UnitVector3}};

use crate::iterator::KNodeChildren;

#[derive(Default)]
pub struct KNodeData {
    pub parent: Option<Weak<RefCell<KNodeData>>>,
    pub child: Option<KNode>,
    pub(crate) joint: KJoint,
}

#[derive(Clone)]
pub struct KNode(pub(crate) Rc<RefCell<KNodeData>>);

impl KNode {
    pub fn set_parent(&self, parent: &KNode) {
        (*self.0).borrow_mut().parent = Some(Rc::downgrade(&parent.0));
        (*parent.0).borrow_mut().child = Some(self.clone());
    }

    pub fn iter_children(&self) -> KNodeChildren {
        KNodeChildren::new(self.clone())
    }

    pub fn joint(&self) -> KJointRef {
        KJointRef {
            node_ref: (*self.0).borrow()
        }
    }

    pub fn joint_mut(&self) -> KJointRefMut {
        KJointRefMut { node_ref: (*self.0).borrow_mut() }
    }

    pub fn joint_position(&self) -> Real {
        self.joint().position
    }
}

pub struct KJointRef<'a> {
    node_ref: std::cell::Ref<'a, KNodeData>
}

impl<'a> Deref for KJointRef<'a> {
    type Target = KJoint;
    fn deref(&self) -> &Self::Target {
        &self.node_ref.joint
    }
}

pub struct KJointRefMut<'a> {
    node_ref: std::cell::RefMut<'a, KNodeData>
}

impl<'a> Deref for KJointRefMut<'a> {
    type Target = KJoint;
    fn deref(&self) -> &Self::Target {
        &self.node_ref.joint
    }
}

impl<'a> DerefMut for KJointRefMut<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.node_ref.joint
    }
}

#[derive(Default)]
#[allow(unused)]
pub struct KNodeBuilder(pub(crate) KNodeData);

impl KNodeBuilder {
    pub fn new() -> Self {
        KNodeBuilder::default()
    }

    pub fn joint_type(mut self, joint_type: KJointType) -> Self {
        self.0.joint.joint_type = joint_type;
        self
    }

    pub fn origin(mut self, origin: Isometry3<Real>) -> Self {
        self.0.joint.origin = origin;
        self
    }

    pub fn translation(mut self, translation: Translation3<Real>) -> Self {
        self.0.joint.origin.translation = translation;
        self
    }

    pub fn rotation(mut self, rotation: UnitQuaternion<Real>) -> Self {
        self.0.joint.origin.rotation = rotation;
        self
    }

    pub fn limits(mut self, limits: [Real; 2]) -> Self {
        self.0.joint.limits = limits;
        self
    }

    pub fn name(mut self, name: String) -> Self {
        self.0.joint.name = name;
        self
    }

    pub fn build(self) -> KNode {
        KNode(Rc::new(RefCell::new(self.0)))
    }
}

#[derive(Derivative)]
#[derivative(Default)]
pub struct KJoint {
    pub name: String,
    joint_type: KJointType,
    /// The origin local transform of the joint.
    origin: Isometry3<Real>,
    position: Real,
    #[derivative(Default(value="[f32::NEG_INFINITY, f32::INFINITY]"))]
    pub limits: [Real; 2],
    pub(crate) world_transform_cache: Option<Isometry3<Real>>,
}

impl KJoint {
    pub fn position(&self) -> Real {
        self.position
    }

    pub fn set_position(&mut self, pos: Real) -> Result<&mut Self, KError> {
        match self.joint_type {
            KJointType::Fixed => {
                return Err(KError::SettingFixedJointPos { joint_name: self.name.clone() });
            },
            _ => {
                if pos < self.limits[0] || pos > self.limits[1] {
                    return Err(KError::OutOfLimits { joint_name: self.name.clone(), position: pos, min_limit: self.limits[0], max_limit: self.limits[1] })
                }
                self.set_position_unchecked(pos);
            }
        }

        Ok(self)
    }

    pub fn set_position_clamped(&mut self, pos: Real) {
        if pos < self.limits[0] {
            self.position = self.limits[0];
        }
        else if pos > self.limits[1] {
            self.position = self.limits[1];
        }
        else {
            self.position = pos;
        }
        self.clear_cache();
    }

    pub fn set_position_unchecked(&mut self, pos: Real) -> &mut Self {
        self.position = pos;
        self.clear_cache();
        self
    }

    #[inline]
    pub fn set_origin(&mut self, origin: Isometry3<Real>) -> &mut Self {
        self.origin = origin;
        self.clear_cache();
        self
    }

    pub fn new(joint_type: KJointType) -> Self {
        Self {
            joint_type,
            ..Default::default()
        }
    }

    #[inline]
    pub fn clear_cache(&mut self) {
        self.world_transform_cache = None;
    }

    pub fn joint_type(&self) -> &KJointType {
        &self.joint_type
    }

    pub fn local_transform(&self) -> Isometry3<Real> {
        match self.joint_type {
            KJointType::Fixed => 
                Isometry3::identity(),
            KJointType::Linear { axis } => 
                self.origin * Translation3::from(axis.into_inner() * self.position),
            KJointType::Revolute { axis } => 
                self.origin * UnitQuaternion::from_axis_angle(&axis, self.position)
        }
    }

    pub fn world_transform(&self) -> Option<Isometry3<Real>> {
        self.world_transform_cache
    }
}

#[derive(Debug, Default)]
pub enum KJointType {
    #[default]
    Fixed,
    Revolute {
        axis: UnitVector3<Real>
    },
    Linear {
        axis: UnitVector3<Real>
    },
}


#[derive(Debug, Error)]
pub enum KError {
    #[error("Tried setting joint position for joint \"{}\", but the joint was a fixed joint.", joint_name)]
    SettingFixedJointPos {
        joint_name: String
    },
    #[error("Joint {0} is set out of limits [min: {1}, max:{2}] with posiiton {3}",
        joint_name,
        min_limit,
        max_limit,
        position
    )]
    OutOfLimits {
        joint_name: String,
        position: Real,
        min_limit: Real,
        max_limit: Real,
    },
    #[error("Solver incompatible with joint \"{0}\" of type {1}. Solver type: {2}", joint_name, joint_type, solver_type)]
    SolverIncompatibleWithJointType {
        joint_name: String,
        joint_type: String,
        solver_type: String
    },
    #[error(
        "IK Solver of type {0} tried {1} times but did not converge. position_diff = {2}, angle_diff = {3}",
        solver_type,
        num_tries,
        position_diff,
        angle_diff
    )]
    SolverNotConverged {
        solver_type: String,
        num_tries: usize,
        position_diff: Real,
        angle_diff: Real,
    }
}

/// Easily set parents of nodes in the order given.
///
/// ```
/// let n0 = KNodeBuilder::new().build();
/// let n1 = KNodeBuilder::new().build();
/// let n2 = KNodeBuilder::new().build();
///
/// chain_nodes![n0 => n1 => n2];
/// 
/// // equivalent code:
/// // n1.set_parent(&n0);
/// // n2.set_parent(&n1);
/// ```
#[macro_export]
macro_rules! chain_nodes {
    ($n0:expr => $n1:expr) => {
        $n1.set_parent(&$n0);
    };
    ($n0:expr => $n1:expr => $($rest:tt)+) => {
        $n1.set_parent(&$n0);
        $crate::chain_nodes!($n1 => $($rest)*);
    };
}