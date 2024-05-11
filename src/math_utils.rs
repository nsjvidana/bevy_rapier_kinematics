use bevy::math::*;

/// Returns a rotation's right, up, and forward vectors from a given forward vector.
/// The returned tuple is in the form: (right, up, fwd). Vectors are normalized.
pub fn get_rot_axes_from_forward(fwd: Vec3) -> (Vec3, Vec3, Vec3) {
    let mut temp_up = Vec3::Y;
    if fwd.dot(temp_up) > 0.9 {
        temp_up = Vec3::X;
    }

    let right = temp_up.cross(fwd).normalize();
    return (
        right,
        fwd.cross(right).normalize(), //up
        fwd
    )
}

/// Creates a rotation from the given right-up-forward vectors without ensuring that they
/// are orthonormalized.
pub fn rotation_from_right_up_fwd_unchecked(right: Vec3, up: Vec3, fwd: Vec3) -> Quat {
    let rot_mat = Mat3 {
        x_axis: right,
        y_axis: up,
        z_axis: fwd
    };
    return Quat::from_mat3(&rot_mat);
}

/// Creates a rotation that points an object in the given forward direction.
pub fn rotation_from_fwd(fwd: Vec3) -> Quat {
    let tuple = get_rot_axes_from_forward(fwd);
    return rotation_from_right_up_fwd_unchecked(tuple.0, tuple.1, tuple.2);
}
