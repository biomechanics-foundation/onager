use crate::{
    Basis, ForceVec6, Inertia, MotionVec6, RotationMatrix, TransformationMatrix, TranslationVector, InverseInertia,
};
use core::ops::{
    Add, AddAssign, BitXor, BitXorAssign, Div, DivAssign, Mul, MulAssign, Neg, Not, Shr, ShrAssign,
    Sub, SubAssign,
};

impl ForceVec6 {
    pub fn new() -> Self {
        Self { data: [0.0; 6] }
    }

    pub fn from_array(data: [f64; 6]) -> Self {
        Self { data }
    }

    pub fn rotational_force(&self) -> [f64; 3] {
        [self.data[0], self.data[1], self.data[2]]
    }

    pub fn translational_force(&self) -> [f64; 3] {
        [self.data[3], self.data[4], self.data[5]]
    }

    pub fn dot(&self, rhs: MotionVec6) -> f64 {
        self.data[0] * rhs.data[0]
            + self.data[1] * rhs.data[1]
            + self.data[2] * rhs.data[2]
            + self.data[3] * rhs.data[3]
            + self.data[4] * rhs.data[4]
            + self.data[5] * rhs.data[5]
    }

    pub fn scale(&self, rhs: f64) -> Self {
        ForceVec6::from_array([
            self.data[0] * rhs,
            self.data[1] * rhs,
            self.data[2] * rhs,
            self.data[3] * rhs,
            self.data[4] * rhs,
            self.data[5] * rhs,
        ])
    }

    pub fn scale_mut(&mut self, rhs: f64) {
        self.data[0] *= rhs;
        self.data[1] *= rhs;
        self.data[2] *= rhs;
        self.data[3] *= rhs;
        self.data[4] *= rhs;
        self.data[5] *= rhs;
    }

    pub fn add_mut(&mut self, rhs: ForceVec6) {
        self.data[0] += rhs.data[0];
        self.data[1] += rhs.data[1];
        self.data[2] += rhs.data[2];
        self.data[3] += rhs.data[3];
        self.data[4] += rhs.data[4];
        self.data[5] += rhs.data[5];
    }

    pub fn sub_mut(&mut self, rhs: ForceVec6) {
        self.data[0] -= rhs.data[0];
        self.data[1] -= rhs.data[1];
        self.data[2] -= rhs.data[2];
        self.data[3] -= rhs.data[3];
        self.data[4] -= rhs.data[4];
        self.data[5] -= rhs.data[5];
    }

    // transform the force vector by a transformation matrix
    pub fn transform(&self, rhs: TransformationMatrix) -> Self {
        ForceVec6::from_array([
            self.data[0] * rhs.data[0]
                + self.data[1] * rhs.data[1]
                + self.data[2] * rhs.data[2]
                + self.data[3] * rhs.data[18]
                + self.data[4] * rhs.data[19]
                + self.data[5] * rhs.data[20],
            self.data[0] * rhs.data[6]
                + self.data[1] * rhs.data[7]
                + self.data[2] * rhs.data[8]
                + self.data[3] * rhs.data[24]
                + self.data[4] * rhs.data[25]
                + self.data[5] * rhs.data[26],
            self.data[0] * rhs.data[12]
                + self.data[1] * rhs.data[13]
                + self.data[2] * rhs.data[14]
                + self.data[3] * rhs.data[30]
                + self.data[4] * rhs.data[31]
                + self.data[5] * rhs.data[32],
            self.data[3] * rhs.data[21] + self.data[4] * rhs.data[22] + self.data[5] * rhs.data[23],
            self.data[3] * rhs.data[27] + self.data[4] * rhs.data[28] + self.data[5] * rhs.data[29],
            self.data[3] * rhs.data[33] + self.data[4] * rhs.data[34] + self.data[5] * rhs.data[35],
        ])
    }

    pub fn cross_force(&self, rhs: ForceVec6) -> ForceVec6 {
        ForceVec6::from_array([
            self.data[1] * rhs.data[2] - self.data[2] * rhs.data[1] + self.data[4] * rhs.data[5]
                - self.data[5] * rhs.data[4],
            self.data[2] * rhs.data[0] - self.data[0] * rhs.data[2] + self.data[5] * rhs.data[3]
                - self.data[3] * rhs.data[5],
            self.data[0] * rhs.data[1] - self.data[1] * rhs.data[0] + self.data[3] * rhs.data[4]
                - self.data[4] * rhs.data[3],
            self.data[1] * rhs.data[5] - self.data[2] * rhs.data[4],
            self.data[2] * rhs.data[3] - self.data[0] * rhs.data[5],
            self.data[0] * rhs.data[4] - self.data[1] * rhs.data[3],
        ])
    }

    pub fn cross_motion(&self, rhs: MotionVec6) -> MotionVec6 {
        MotionVec6::from_array([
            self.data[1] * rhs.data[2] - self.data[2] * rhs.data[1],
            self.data[2] * rhs.data[0] - self.data[0] * rhs.data[2],
            self.data[0] * rhs.data[1] - self.data[1] * rhs.data[0],
            self.data[1] * rhs.data[5] - self.data[2] * rhs.data[4] + self.data[4] * rhs.data[2]
                - self.data[5] * rhs.data[1],
            self.data[2] * rhs.data[3] - self.data[0] * rhs.data[5] + self.data[5] * rhs.data[0]
                - self.data[3] * rhs.data[2],
            self.data[0] * rhs.data[4] - self.data[1] * rhs.data[3] + self.data[3] * rhs.data[1]
                - self.data[4] * rhs.data[0],
        ])
    }
}

impl Default for ForceVec6 {
    fn default() -> Self {
        Self::new()
    }
}

impl Mul<f64> for ForceVec6 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        self.scale(rhs)
    }
}

impl Mul<MotionVec6> for ForceVec6 {
    type Output = f64;

    fn mul(self, rhs: MotionVec6) -> Self::Output {
        self.dot(rhs)
    }
}

impl MulAssign<f64> for ForceVec6 {
    fn mul_assign(&mut self, rhs: f64) {
        self.scale_mut(rhs);
    }
}

impl Add<ForceVec6> for ForceVec6 {
    type Output = Self;

    fn add(self, rhs: ForceVec6) -> Self::Output {
        ForceVec6::from_array([
            self.data[0] + rhs.data[0],
            self.data[1] + rhs.data[1],
            self.data[2] + rhs.data[2],
            self.data[3] + rhs.data[3],
            self.data[4] + rhs.data[4],
            self.data[5] + rhs.data[5],
        ])
    }
}

impl AddAssign<ForceVec6> for ForceVec6 {
    fn add_assign(&mut self, rhs: ForceVec6) {
        self.add_mut(rhs);
    }
}

impl Sub<ForceVec6> for ForceVec6 {
    type Output = Self;

    fn sub(self, rhs: ForceVec6) -> Self::Output {
        ForceVec6::from_array([
            self.data[0] - rhs.data[0],
            self.data[1] - rhs.data[1],
            self.data[2] - rhs.data[2],
            self.data[3] - rhs.data[3],
            self.data[4] - rhs.data[4],
            self.data[5] - rhs.data[5],
        ])
    }
}

impl SubAssign<ForceVec6> for ForceVec6 {
    fn sub_assign(&mut self, rhs: ForceVec6) {
        self.sub_mut(rhs);
    }
}

impl Div<f64> for ForceVec6 {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        ForceVec6::from_array([
            self.data[0] / rhs,
            self.data[1] / rhs,
            self.data[2] / rhs,
            self.data[3] / rhs,
            self.data[4] / rhs,
            self.data[5] / rhs,
        ])
    }
}

impl DivAssign<f64> for ForceVec6 {
    fn div_assign(&mut self, rhs: f64) {
        self.data[0] /= rhs;
        self.data[1] /= rhs;
        self.data[2] /= rhs;
        self.data[3] /= rhs;
        self.data[4] /= rhs;
        self.data[5] /= rhs;
    }
}

impl Neg for ForceVec6 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        ForceVec6::from_array([
            -self.data[0],
            -self.data[1],
            -self.data[2],
            -self.data[3],
            -self.data[4],
            -self.data[5],
        ])
    }
}

impl Shr<RotationMatrix> for ForceVec6 {
    type Output = Self;

    fn shr(self, rhs: RotationMatrix) -> Self::Output {
        self >> rhs.as_transform()
    }
}

impl ShrAssign<RotationMatrix> for ForceVec6 {
    fn shr_assign(&mut self, rhs: RotationMatrix) {
        *self = *self >> rhs.as_transform();
    }
}

impl Shr<TranslationVector> for ForceVec6 {
    type Output = Self;

    fn shr(self, rhs: TranslationVector) -> Self::Output {
        self >> rhs.as_transform()
    }
}

impl ShrAssign<TranslationVector> for ForceVec6 {
    fn shr_assign(&mut self, rhs: TranslationVector) {
        *self = *self >> rhs.as_transform();
    }
}

impl Shr<TransformationMatrix> for ForceVec6 {
    type Output = Self;

    fn shr(self, rhs: TransformationMatrix) -> Self::Output {
        self.transform(rhs)
    }
}

impl ShrAssign<TransformationMatrix> for ForceVec6 {
    fn shr_assign(&mut self, rhs: TransformationMatrix) {
        *self = *self >> rhs;
    }
}

impl BitXor<ForceVec6> for ForceVec6 {
    type Output = ForceVec6;

    fn bitxor(self, rhs: ForceVec6) -> Self::Output {
        self.cross_force(rhs)
    }
}

impl BitXor<MotionVec6> for ForceVec6 {
    type Output = MotionVec6;

    fn bitxor(self, rhs: MotionVec6) -> Self::Output {
        self.cross_motion(rhs)
    }
}

impl BitXorAssign<ForceVec6> for ForceVec6 {
    fn bitxor_assign(&mut self, rhs: ForceVec6) {
        *self = *self ^ rhs;
    }
}

impl MotionVec6 {
    pub fn new() -> Self {
        Self { data: [0.0; 6] }
    }

    pub fn from_array(data: [f64; 6]) -> Self {
        Self { data }
    }

    pub fn rotational_motion(&self) -> [f64; 3] {
        [self.data[0], self.data[1], self.data[2]]
    }

    pub fn translational_motion(&self) -> [f64; 3] {
        [self.data[3], self.data[4], self.data[5]]
    }

    pub fn dot(&self, rhs: ForceVec6) -> f64 {
        self.data[0] * rhs.data[0]
            + self.data[1] * rhs.data[1]
            + self.data[2] * rhs.data[2]
            + self.data[3] * rhs.data[3]
            + self.data[4] * rhs.data[4]
            + self.data[5] * rhs.data[5]
    }

    pub fn scale(&self, rhs: f64) -> Self {
        MotionVec6::from_array([
            self.data[0] * rhs,
            self.data[1] * rhs,
            self.data[2] * rhs,
            self.data[3] * rhs,
            self.data[4] * rhs,
            self.data[5] * rhs,
        ])
    }

    pub fn scale_mut(&mut self, rhs: f64) {
        self.data[0] *= rhs;
        self.data[1] *= rhs;
        self.data[2] *= rhs;
        self.data[3] *= rhs;
        self.data[4] *= rhs;
        self.data[5] *= rhs;
    }

    pub fn transform(&self, rhs: TransformationMatrix) -> Self {
        MotionVec6::from_array([
            self.data[0] * rhs.data[0] + self.data[1] * rhs.data[1] + self.data[2] * rhs.data[2],
            self.data[0] * rhs.data[6] + self.data[1] * rhs.data[7] + self.data[2] * rhs.data[8],
            self.data[0] * rhs.data[12] + self.data[1] * rhs.data[13] + self.data[2] * rhs.data[14],
            self.data[0] * rhs.data[18]
                + self.data[1] * rhs.data[19]
                + self.data[2] * rhs.data[20]
                + self.data[3] * rhs.data[21]
                + self.data[4] * rhs.data[22]
                + self.data[5] * rhs.data[23],
            self.data[0] * rhs.data[24]
                + self.data[1] * rhs.data[25]
                + self.data[2] * rhs.data[26]
                + self.data[3] * rhs.data[27]
                + self.data[4] * rhs.data[28]
                + self.data[5] * rhs.data[29],
            self.data[0] * rhs.data[30]
                + self.data[1] * rhs.data[31]
                + self.data[2] * rhs.data[32]
                + self.data[3] * rhs.data[33]
                + self.data[4] * rhs.data[34]
                + self.data[5] * rhs.data[35],
        ])
    }

    pub fn cross_force(&self, rhs: ForceVec6) -> ForceVec6 {
        ForceVec6::from_array([
            self.data[1] * rhs.data[2] - self.data[2] * rhs.data[1] + self.data[4] * rhs.data[5]
                - self.data[5] * rhs.data[4],
            self.data[2] * rhs.data[0] - self.data[0] * rhs.data[2] + self.data[5] * rhs.data[3]
                - self.data[3] * rhs.data[5],
            self.data[0] * rhs.data[1] - self.data[1] * rhs.data[0] + self.data[3] * rhs.data[4]
                - self.data[4] * rhs.data[3],
            self.data[1] * rhs.data[5] - self.data[2] * rhs.data[4],
            self.data[2] * rhs.data[3] - self.data[0] * rhs.data[5],
            self.data[0] * rhs.data[4] - self.data[1] * rhs.data[3],
        ])
    }

    pub fn cross_motion(&self, rhs: MotionVec6) -> MotionVec6 {
        MotionVec6::from_array([
            self.data[1] * rhs.data[2] - self.data[2] * rhs.data[1],
            self.data[2] * rhs.data[0] - self.data[0] * rhs.data[2],
            self.data[0] * rhs.data[1] - self.data[1] * rhs.data[0],
            self.data[1] * rhs.data[5] - self.data[2] * rhs.data[4] + self.data[4] * rhs.data[2]
                - self.data[5] * rhs.data[1],
            self.data[2] * rhs.data[3] - self.data[0] * rhs.data[5] + self.data[5] * rhs.data[0]
                - self.data[3] * rhs.data[2],
            self.data[0] * rhs.data[4] - self.data[1] * rhs.data[3] + self.data[3] * rhs.data[1]
                - self.data[4] * rhs.data[0],
        ])
    }
}

impl Default for MotionVec6 {
    fn default() -> Self {
        Self::new()
    }
}

impl Mul<f64> for MotionVec6 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        self.scale(rhs)
    }
}

impl Mul<ForceVec6> for MotionVec6 {
    type Output = f64;

    fn mul(self, rhs: ForceVec6) -> Self::Output {
        self.dot(rhs)
    }
}

impl MulAssign<f64> for MotionVec6 {
    fn mul_assign(&mut self, rhs: f64) {
        self.scale_mut(rhs);
    }
}

impl Add<MotionVec6> for MotionVec6 {
    type Output = Self;

    fn add(self, rhs: MotionVec6) -> Self::Output {
        MotionVec6::from_array([
            self.data[0] + rhs.data[0],
            self.data[1] + rhs.data[1],
            self.data[2] + rhs.data[2],
            self.data[3] + rhs.data[3],
            self.data[4] + rhs.data[4],
            self.data[5] + rhs.data[5],
        ])
    }
}

impl AddAssign<MotionVec6> for MotionVec6 {
    fn add_assign(&mut self, rhs: MotionVec6) {
        self.data[0] += rhs.data[0];
        self.data[1] += rhs.data[1];
        self.data[2] += rhs.data[2];
        self.data[3] += rhs.data[3];
        self.data[4] += rhs.data[4];
        self.data[5] += rhs.data[5];
    }
}

impl Sub<MotionVec6> for MotionVec6 {
    type Output = Self;

    fn sub(self, rhs: MotionVec6) -> Self::Output {
        MotionVec6::from_array([
            self.data[0] - rhs.data[0],
            self.data[1] - rhs.data[1],
            self.data[2] - rhs.data[2],
            self.data[3] - rhs.data[3],
            self.data[4] - rhs.data[4],
            self.data[5] - rhs.data[5],
        ])
    }
}

impl SubAssign<MotionVec6> for MotionVec6 {
    fn sub_assign(&mut self, rhs: MotionVec6) {
        self.data[0] -= rhs.data[0];
        self.data[1] -= rhs.data[1];
        self.data[2] -= rhs.data[2];
        self.data[3] -= rhs.data[3];
        self.data[4] -= rhs.data[4];
        self.data[5] -= rhs.data[5];
    }
}

impl Div<f64> for MotionVec6 {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        MotionVec6::from_array([
            self.data[0] / rhs,
            self.data[1] / rhs,
            self.data[2] / rhs,
            self.data[3] / rhs,
            self.data[4] / rhs,
            self.data[5] / rhs,
        ])
    }
}

impl DivAssign<f64> for MotionVec6 {
    fn div_assign(&mut self, rhs: f64) {
        self.data[0] /= rhs;
        self.data[1] /= rhs;
        self.data[2] /= rhs;
        self.data[3] /= rhs;
        self.data[4] /= rhs;
        self.data[5] /= rhs;
    }
}

impl Neg for MotionVec6 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        MotionVec6::from_array([
            -self.data[0],
            -self.data[1],
            -self.data[2],
            -self.data[3],
            -self.data[4],
            -self.data[5],
        ])
    }
}

impl Shr<RotationMatrix> for MotionVec6 {
    type Output = Self;

    fn shr(self, rhs: RotationMatrix) -> Self::Output {
        self >> rhs.as_transform()
    }
}

impl ShrAssign<RotationMatrix> for MotionVec6 {
    fn shr_assign(&mut self, rhs: RotationMatrix) {
        *self = *self >> rhs.as_transform();
    }
}

impl Shr<TranslationVector> for MotionVec6 {
    type Output = Self;

    fn shr(self, rhs: TranslationVector) -> Self::Output {
        self >> rhs.as_transform()
    }
}

impl ShrAssign<TranslationVector> for MotionVec6 {
    fn shr_assign(&mut self, rhs: TranslationVector) {
        *self = *self >> rhs.as_transform();
    }
}

impl Shr<TransformationMatrix> for MotionVec6 {
    type Output = Self;

    fn shr(self, rhs: TransformationMatrix) -> Self::Output {
        self.transform(rhs)
    }
}

impl ShrAssign<TransformationMatrix> for MotionVec6 {
    fn shr_assign(&mut self, rhs: TransformationMatrix) {
        *self = *self >> rhs;
    }
}

impl BitXor<ForceVec6> for MotionVec6 {
    type Output = ForceVec6;

    fn bitxor(self, rhs: ForceVec6) -> Self::Output {
        self.cross_force(rhs)
    }
}

impl BitXor<MotionVec6> for MotionVec6 {
    type Output = MotionVec6;

    fn bitxor(self, rhs: MotionVec6) -> Self::Output {
        self.cross_motion(rhs)
    }
}

impl BitXorAssign<MotionVec6> for MotionVec6 {
    fn bitxor_assign(&mut self, rhs: MotionVec6) {
        *self = *self ^ rhs;
    }
}

impl TransformationMatrix {
    pub fn new() -> Self {
        Self { data: [0.0; 36] }
    }

    pub fn from_array(data: [f64; 36]) -> Self {
        Self { data }
    }

    pub fn to_rotation(&self) -> RotationMatrix {
        RotationMatrix {
            data: [
                self.data[0],
                self.data[1],
                self.data[2],
                self.data[6],
                self.data[7],
                self.data[8],
                self.data[12],
                self.data[13],
                self.data[14],
            ],
        }
    }

    pub fn to_translation(&self) -> TranslationVector {
        TranslationVector {
            data: [self.data[26], self.data[30], self.data[19]],
        }
    }

    pub fn multiply(&self, rhs: TransformationMatrix) -> Self {
        let mut data = [0.0; 36];
        for i in 0..6 {
            for j in 0..6 {
                for k in 0..6 {
                    data[i * 6 + j] += self.data[i * 6 + k] * rhs.data[k * 6 + j];
                }
            }
        }
        Self { data }
    }

    pub fn inverse_transform(&self) -> Self {
        !self.to_rotation() + !self.to_translation()
    }
}

impl Default for TransformationMatrix {
    fn default() -> Self {
        Self::new()
    }
}

impl Mul<TransformationMatrix> for TransformationMatrix {
    type Output = Self;

    fn mul(self, rhs: TransformationMatrix) -> Self::Output {
        self.multiply(rhs)
    }
}

// the inverse transformation
impl Not for TransformationMatrix {
    type Output = Self;

    fn not(self) -> Self::Output {
        self.inverse_transform()
    }
}

impl RotationMatrix {
    pub fn new() -> Self {
        Self { data: [0.0; 9] }
    }

    pub fn from_array(data: [f64; 9]) -> Self {
        Self { data }
    }

    pub fn from_angle(axis: Basis, angle: f64) -> Self {
        match axis {
            Basis::X => Self::from_array([
                1.0,
                0.0,
                0.0,
                0.0,
                angle.cos(),
                angle.sin(),
                0.0,
                -angle.sin(),
                angle.cos(),
            ]),
            Basis::Y => Self::from_array([
                angle.cos(),
                0.0,
                -angle.sin(),
                0.0,
                1.0,
                0.0,
                angle.sin(),
                0.0,
                angle.cos(),
            ]),
            Basis::Z => Self::from_array([
                angle.cos(),
                angle.sin(),
                0.0,
                -angle.sin(),
                angle.cos(),
                0.0,
                0.0,
                0.0,
                1.0,
            ]),
        }
    }

    pub fn from_x_rotation(angle: f64) -> Self {
        RotationMatrix::from_angle(Basis::X, angle)
    }

    pub fn from_y_rotation(angle: f64) -> Self {
        RotationMatrix::from_angle(Basis::Y, angle)
    }

    pub fn from_z_rotation(angle: f64) -> Self {
        RotationMatrix::from_angle(Basis::Z, angle)
    }

    pub fn as_transform(&self) -> TransformationMatrix {
        TransformationMatrix {
            data: [
                self.data[0],
                self.data[1],
                self.data[2],
                0.0,
                0.0,
                0.0,
                self.data[3],
                self.data[4],
                self.data[5],
                0.0,
                0.0,
                0.0,
                self.data[6],
                self.data[7],
                self.data[8],
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                self.data[0],
                self.data[1],
                self.data[2],
                0.0,
                0.0,
                0.0,
                self.data[3],
                self.data[4],
                self.data[5],
                0.0,
                0.0,
                0.0,
                self.data[6],
                self.data[7],
                self.data[8],
            ],
        }
    }

    pub fn multiply(&self, rhs: RotationMatrix) -> Self {
        RotationMatrix::from_array([
            self.data[0] * rhs.data[0] + self.data[1] * rhs.data[3] + self.data[2] * rhs.data[6],
            self.data[0] * rhs.data[1] + self.data[1] * rhs.data[4] + self.data[2] * rhs.data[7],
            self.data[0] * rhs.data[2] + self.data[1] * rhs.data[5] + self.data[2] * rhs.data[8],
            self.data[3] * rhs.data[0] + self.data[4] * rhs.data[3] + self.data[5] * rhs.data[6],
            self.data[3] * rhs.data[1] + self.data[4] * rhs.data[4] + self.data[5] * rhs.data[7],
            self.data[3] * rhs.data[2] + self.data[4] * rhs.data[5] + self.data[5] * rhs.data[8],
            self.data[6] * rhs.data[0] + self.data[7] * rhs.data[3] + self.data[8] * rhs.data[6],
            self.data[6] * rhs.data[1] + self.data[7] * rhs.data[4] + self.data[8] * rhs.data[7],
            self.data[6] * rhs.data[2] + self.data[7] * rhs.data[5] + self.data[8] * rhs.data[8],
        ])
    }

    pub fn transpose(&self) -> Self {
        RotationMatrix::from_array([
            self.data[0],
            self.data[3],
            self.data[6],
            self.data[1],
            self.data[4],
            self.data[7],
            self.data[2],
            self.data[5],
            self.data[8],
        ])
    }
}

impl Default for RotationMatrix {
    fn default() -> Self {
        Self::new()
    }
}

impl Mul<RotationMatrix> for RotationMatrix {
    type Output = Self;

    fn mul(self, rhs: RotationMatrix) -> Self::Output {
        self.multiply(rhs)
    }
}

impl Not for RotationMatrix {
    type Output = Self;

    fn not(self) -> Self::Output {
        self.transpose()
    }
}

impl Add<TranslationVector> for RotationMatrix {
    type Output = TransformationMatrix;

    fn add(self, rhs: TranslationVector) -> Self::Output {
        self.as_transform() * rhs.as_transform()
    }
}

impl TranslationVector {
    pub fn new() -> Self {
        Self { data: [0.0; 3] }
    }

    pub fn from_array(data: [f64; 3]) -> Self {
        Self { data }
    }

    pub fn as_transform(&self) -> TransformationMatrix {
        TransformationMatrix {
            data: [
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                self.data[2],
                -self.data[1],
                1.0,
                0.0,
                0.0,
                -self.data[2],
                0.0,
                self.data[0],
                0.0,
                1.0,
                0.0,
                self.data[1],
                -self.data[0],
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        }
    }
}

impl Default for TranslationVector {
    fn default() -> Self {
        Self::new()
    }
}

impl Mul<f64> for TranslationVector {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        TranslationVector::from_array([self.data[0] * rhs, self.data[1] * rhs, self.data[2] * rhs])
    }
}

impl Add<TranslationVector> for TranslationVector {
    type Output = Self;

    fn add(self, rhs: TranslationVector) -> Self::Output {
        TranslationVector::from_array([
            self.data[0] + rhs.data[0],
            self.data[1] + rhs.data[1],
            self.data[2] + rhs.data[2],
        ])
    }
}

impl Sub<TranslationVector> for TranslationVector {
    type Output = Self;

    fn sub(self, rhs: TranslationVector) -> Self::Output {
        TranslationVector::from_array([
            self.data[0] - rhs.data[0],
            self.data[1] - rhs.data[1],
            self.data[2] - rhs.data[2],
        ])
    }
}

impl Neg for TranslationVector {
    type Output = Self;

    fn neg(self) -> Self::Output {
        TranslationVector::from_array([-self.data[0], -self.data[1], -self.data[2]])
    }
}

impl Not for TranslationVector {
    type Output = Self;

    fn not(self) -> Self::Output {
        -self
    }
}

impl Add<RotationMatrix> for TranslationVector {
    type Output = TransformationMatrix;

    fn add(self, rhs: RotationMatrix) -> Self::Output {
        self.as_transform() * rhs.as_transform()
    }
}

impl Inertia {
    pub fn new(
        mass: f64,
        i_xx: f64,
        i_yy: f64,
        i_zz: f64,
        i_xy: f64,
        i_xz: f64,
        i_yz: f64,
    ) -> Self {
        Self {
            mass,
            i_xx,
            i_yy,
            i_zz,
            i_xy,
            i_xz,
            i_yz,
        }
    }

    pub fn motion_multiply(
        &self,
        motion: MotionVec6,
        center_of_mass: TranslationVector,
    ) -> ForceVec6 {
        ForceVec6::from_array([
            self.i_xx * motion.data[0] + self.i_xy * motion.data[1] + self.i_xz * motion.data[2],
            self.i_xy * motion.data[0] + self.i_yy * motion.data[1] + self.i_yz * motion.data[2],
            self.i_xz * motion.data[0] + self.i_yz * motion.data[1] + self.i_zz * motion.data[2],
            self.mass
                * (motion.data[4] * -center_of_mass.data[2]
                    + motion.data[5] * center_of_mass.data[1]),
            self.mass
                * (motion.data[3] * center_of_mass.data[2]
                    + motion.data[5] * -center_of_mass.data[0]),
            self.mass
                * (motion.data[3] * -center_of_mass.data[1]
                    + motion.data[4] * center_of_mass.data[0]),
        ])
    }
}

impl InverseInertia {
    pub fn new(
        mass: f64,
        i_xx: f64,
        i_yy: f64,
        i_zz: f64,
        i_xy: f64,
        i_xz: f64,
        i_yz: f64,
    ) -> Self {
        Self {
            mass,
            i_xx,
            i_yy,
            i_zz,
            i_xy,
            i_xz,
            i_yz,
        }
    }
}
