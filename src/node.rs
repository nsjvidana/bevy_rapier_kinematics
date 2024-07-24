use std::{cell::RefCell, ops::Deref, rc::{Rc, Weak}};

use thiserror::Error;
use bevy_rapier3d::{math::Real, na::{Isometry3, Translation3, UnitQuaternion, UnitVector3}};

#[derive(Default)]
pub struct KNodeData {
    pub parent: Option<Weak<RefCell<KNodeData>>>,
    pub children: Vec<KNode>,
    joint: KJoint,
}

#[derive(Clone)]
pub struct KNode(pub(crate) Rc<RefCell<KNodeData>>);

impl KNode {
    pub fn set_parent(&self, parent: &KNode) {
        (*self.0).borrow_mut().parent = Some(Rc::downgrade(&parent.0));
        (*parent.0).borrow_mut().children.push(self.clone());
    }

    pub fn joint(&self) -> KJointRef {
        KJointRef {
            node_ref: (*self.0).borrow()
        }
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

#[derive(Default)]
#[allow(unused)]
pub struct KNodeBuilder(pub(crate) KNodeData);

impl KNodeBuilder {
    #[inline]
    pub fn new() {
        KNodeBuilder::default();
    }

    #[inline]
    pub fn joint_type(mut self, joint_type: KJointType) -> Self {
        self.0.joint.joint_type = joint_type;
        self
    }

    #[inline]
    pub fn origin(mut self, origin: Isometry3<Real>) -> Self {
        self.0.joint.origin = origin;
        self
    }

    #[inline]
    pub fn translation(mut self, translation: Translation3<Real>) -> Self {
        self.0.joint.origin.translation = translation;
        self
    }

    #[inline]
    pub fn rotation(mut self, rotation: UnitQuaternion<Real>) -> Self {
        self.0.joint.origin.rotation = rotation;
        self
    }

    #[inline]
    pub fn limits(mut self, limits: [Real; 2]) -> Self {
        self.0.joint.limits = limits;
        self
    }

    #[inline]
    pub fn name(mut self, name: String) -> Self {
        self.0.joint.name = name;
        self
    }

    #[inline]
    pub fn into_knode(self) -> KNode {
        KNode(Rc::new(RefCell::new(self.0)))
    }
}

#[derive(Default)]
pub struct KJoint {
    pub name: String,
    joint_type: KJointType,
    position: Real,
    pub limits: [Real; 2],
    origin: Isometry3<Real>,
    world_transform_cache: Option<Isometry3<Real>>,
}

impl KJoint {
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
}

#[derive(Default)]
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
    }
}

#[macro_export]
macro_rules! chain_nodes {
    ($n1:expr => $n2:expr) => {
        $n2.set_parent(&$n1);
    };
    ($n1:expr => $n2:expr => $($nn:tt)+) => {
        $n2.set_parent(&$n1);
        $crate::connect!($n2 => $($nn)*);
    };
}
