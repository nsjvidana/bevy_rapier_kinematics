use std::f32::consts::PI;
use bevy::math::*;
use bevy_rapier3d::na::{Matrix3, UnitQuaternion, UnitVector3, Vector, Vector3, ArrayStorage, Rotation3};
use bevy_rapier3d::parry::math::Real;
use bevy_rapier3d::rapier::utils::SimdBasis;

pub const FRAC_PI_12: f32 = PI/12.;

/// Returns a rotation's right, up, and forward vectors from a given forward vector.
/// The returned tuple is in the form: (right, up, fwd). Vectors are normalized.
pub fn get_rot_axes_from_forward(fwd: &UnitVector3<Real>) -> (UnitVector3<Real>, UnitVector3<Real>, UnitVector3<Real>) {
    let mut temp_up = Vector3::new(0., 1., 0.);
    if fwd.dot(&temp_up) > 0.9 {
        temp_up = Vector3::new(1., 0., 0.);
    }

    let right = UnitVector3::new_normalize(temp_up.cross(&fwd));
    return (
        right,
        UnitVector3::new_normalize(fwd.cross(&right).normalize()),
        fwd.clone(),
    );
}

/// Creates a rotation from the given right-up-forward vectors without ensuring that they
/// are orthonormalized.
#[inline]
pub fn rot_mat_from_right_up_fwd(
    right: &UnitVector3<Real>,
    up: &UnitVector3<Real>,
    fwd: &UnitVector3<Real>
) -> Matrix3<Real> {
    return Matrix3::new(
        right.x, right.y, right.z,
        up.x, up.y, up.z,
        fwd.x, fwd.y, fwd.z,
    );
}

/// Creates a rotation that points an object in the given forward direction.
pub fn rotation_from_fwd(fwd: &UnitVector3<Real>) -> UnitQuaternion<Real> {
    let basis = fwd.orthonormal_basis();

    let mat = Matrix3::from_columns(&[
        -basis[1],
        basis[0],
        fwd.into_inner(),
    ]);
    let rot_mat = Rotation3::from_matrix_unchecked(mat);
    return UnitQuaternion::from_rotation_matrix(&rot_mat);
}

pub fn get_rot_axes(rot: &UnitQuaternion<Real>) -> (UnitVector3<Real>, UnitVector3<Real>, UnitVector3<Real>) {
    return (
        UnitVector3::new_unchecked(rot * Vector::x()),
        UnitVector3::new_unchecked(rot * Vector::y()),
        UnitVector3::new_unchecked(rot * Vector::z()),
    );
}

#[inline(always)]
pub fn vec3_y(y: f32) -> Vec3 {
    Vec3::new(0., y, 0.)
}
