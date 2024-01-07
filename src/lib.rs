pub mod ops;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ForceVec6 {
    data: [f64; 6],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionVec6 {
    data: [f64; 6],
}

/// Motion form of a transformation matrix
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransformationMatrix {
    data: [f64; 36],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RotationMatrix {
    data: [f64; 9],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TranslationVector {
    data: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Basis {
    X,
    Y,
    Z,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inertia {
    pub mass: f64,
    pub i_xx: f64,
    pub i_yy: f64,
    pub i_zz: f64,
    pub i_xy: f64,
    pub i_xz: f64,
    pub i_yz: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InverseInertia {
    pub mass: f64,
    pub i_xx: f64,
    pub i_yy: f64,
    pub i_zz: f64,
    pub i_xy: f64,
    pub i_xz: f64,
    pub i_yz: f64,
}

//Body("pedal", Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), 1.0, 1.0, 1.0, 1.0, 1.0);
//let inertia = Inertia(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
//
//Body("pedal", model.body(1), Vec3(0.0), inertia.clone().i_xx(5.0));
//
//Inertia::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
//Inertia::new([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]);
//Inertia::new(my_array);

//new_motion = my_motion.rotate(RotMat3);
//
//new_motion = my_motion >> RotMat3;
//
//my_motion >>= RotMat3;
//translation = my_motion.translation();
//new_position = my_position >> translation as Transform;

// Find rotation matrix between two frames

// Apply rotation matrix to vector

// Find transl

//impl Mul for MotionVec6 {
//    type Output = ForceVec6;
//
//    fn mul(self, rhs: Self) -> Self::Output {
//        ForceVec6::cross_motion(self, rhs)
//    }
//}
//
//ForceVec6::cross_motion(my_force, my_motion);
//ForceVec6::cross_force(my_force, my_force);
//MotionVec6::cross_force(my_motion, my_force);
//MotionVec6::cross_motion(my_motion, my_motion);
//MotionVec6::dot(my_motion, my_force);
//
//ForceVec6::cross(my_force, MotionVec6::dot(my_motion, my_force));
//
//my_force.cross(my_motion.dot(my_force));
//
//new_force_vec = my_motion % my_force;
//new_motion_vec = my_force % my_motion;
//
//force_vec *= motion_vec;
//counter += 1;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dot() {
        let force = ForceVec6::from_array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let motion = MotionVec6::from_array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);

        assert_eq!(force.dot(motion), 91.0);
    }
}
