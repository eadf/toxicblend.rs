use ilattice::glam::{IVec3, Vec3A /*Vec3*/};

#[inline]
pub(crate) fn to_ivec3([x, y, z]: [u32; 3]) -> IVec3 {
    IVec3::new(x as i32, y as i32, z as i32)
}

pub(crate) trait ToInt {
    fn to_int(self) -> IVec3;
}
impl ToInt for Vec3A {
    #[inline]
    fn to_int(self) -> IVec3 {
        IVec3::new(self.x as i32, self.y as i32, self.z as i32)
    }
}

pub(crate) trait ToFloat {
    fn to_float(self) -> Vec3A;
}
impl ToFloat for IVec3 {
    #[inline]
    fn to_float(self) -> Vec3A {
        Vec3A::new(self.x as f32, self.y as f32, self.z as f32)
    }
}
impl ToFloat for cgmath::Point3<f32> {
    #[inline]
    fn to_float(self) -> Vec3A {
        Vec3A::new(self.x, self.y, self.z)
    }
}