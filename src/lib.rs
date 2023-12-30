#![no_std]

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
//new_force_vec = my_force * my_motion;
//new_motion_vec = my_motion * (my_force | my_motion);
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
