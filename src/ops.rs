use crate::{ForceVec6, MotionVec6, RotationMatrix, TransformationMatrix, TranslationVector};
use core::ops::{
    Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Not, Shr, ShrAssign, Sub, SubAssign,
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
}

impl Default for ForceVec6 {
    fn default() -> Self {
        Self::new()
    }
}

impl Mul<f64> for ForceVec6 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        ForceVec6::from_array([
            self.data[0] * rhs,
            self.data[1] * rhs,
            self.data[2] * rhs,
            self.data[3] * rhs,
            self.data[4] * rhs,
            self.data[5] * rhs,
        ])
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
        self.data[0] *= rhs;
        self.data[1] *= rhs;
        self.data[2] *= rhs;
        self.data[3] *= rhs;
        self.data[4] *= rhs;
        self.data[5] *= rhs;
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
        self.data[0] += rhs.data[0];
        self.data[1] += rhs.data[1];
        self.data[2] += rhs.data[2];
        self.data[3] += rhs.data[3];
        self.data[4] += rhs.data[4];
        self.data[5] += rhs.data[5];
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
        self.data[0] -= rhs.data[0];
        self.data[1] -= rhs.data[1];
        self.data[2] -= rhs.data[2];
        self.data[3] -= rhs.data[3];
        self.data[4] -= rhs.data[4];
        self.data[5] -= rhs.data[5];
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

impl Shr<TranslationVector> for ForceVec6 {
    type Output = Self;

    fn shr(self, rhs: TranslationVector) -> Self::Output {
        self >> rhs.as_transform()
    }
}

impl Shr<TransformationMatrix> for ForceVec6 {
    type Output = Self;

    fn shr(self, rhs: TransformationMatrix) -> Self::Output {
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
}

impl Default for MotionVec6 {
    fn default() -> Self {
        Self::new()
    }
}

impl Mul<f64> for MotionVec6 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        MotionVec6::from_array([
            self.data[0] * rhs,
            self.data[1] * rhs,
            self.data[2] * rhs,
            self.data[3] * rhs,
            self.data[4] * rhs,
            self.data[5] * rhs,
        ])
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
        self.data[0] *= rhs;
        self.data[1] *= rhs;
        self.data[2] *= rhs;
        self.data[3] *= rhs;
        self.data[4] *= rhs;
        self.data[5] *= rhs;
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

impl Shr<TranslationVector> for MotionVec6 {
    type Output = Self;

    fn shr(self, rhs: TranslationVector) -> Self::Output {
        self >> rhs.as_transform()
    }
}

impl Shr<TransformationMatrix> for MotionVec6 {
    type Output = Self;

    fn shr(self, rhs: TransformationMatrix) -> Self::Output {
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

    pub fn as_transform(&self) -> Self {
        Self { data: self.data }
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
}

// the inverse transformation
impl Not for TransformationMatrix {
    type Output = Self;

    fn not(self) -> Self::Output {
        !self.to_rotation() + !self.to_translation()
    }
}

impl RotationMatrix {
    pub fn new() -> Self {
        Self { data: [0.0; 9] }
    }

    pub fn from_array(data: [f64; 9]) -> Self {
        Self { data }
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
}

impl Default for RotationMatrix {
    fn default() -> Self {
        Self::new()
    }
}

impl Mul<RotationMatrix> for RotationMatrix {
    type Output = Self;

    fn mul(self, rhs: RotationMatrix) -> Self::Output {
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
}

impl Not for RotationMatrix {
    type Output = Self;

    fn not(self) -> Self::Output {
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
