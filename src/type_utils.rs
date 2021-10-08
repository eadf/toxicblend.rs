use ilattice::glam::{IVec3, Vec3A /*Vec3*/};

pub(crate) trait ToInt<T> {
    fn to_int(self) -> IVec3;
}
impl ToInt<Vec3A> for Vec3A {
    #[inline]
    fn to_int(self) -> IVec3 {
        IVec3::new(self.x as i32, self.y as i32, self.z as i32)
    }
}

pub(crate) trait ToFloat<T> {
    fn to_float(self) -> Vec3A;
}
impl ToFloat<IVec3> for IVec3 {
    #[inline]
    fn to_float(self) -> Vec3A {
        Vec3A::new(self.x as f32, self.y as f32, self.z as f32)
    }
}
impl ToFloat<cgmath::Point3<f32>> for cgmath::Point3<f32> {
    #[inline]
    fn to_float(self) -> Vec3A {
        Vec3A::new(self.x, self.y, self.z)
    }
}

pub(crate) trait Corners<T> {
    fn corners(&self) -> [T; 8];
}

impl Corners<IVec3> for ilattice::extent::Extent<IVec3> {
    #[inline]
    fn corners(&self) -> [IVec3; 8] {
        let min = self.minimum;
        let lub = self.least_upper_bound();

        [
            IVec3::new(min.x, min.y, min.z),
            IVec3::new(lub.x, min.y, min.z),
            IVec3::new(min.x, lub.y, min.z),
            IVec3::new(lub.x, lub.y, min.z),
            IVec3::new(min.x, min.y, lub.z),
            IVec3::new(lub.x, min.y, lub.z),
            IVec3::new(min.x, lub.y, lub.z),
            IVec3::new(lub.x, lub.y, lub.z),
        ]
    }
}

impl Corners<Vec3A> for ilattice::extent::Extent<Vec3A> {
    #[inline]
    fn corners(&self) -> [Vec3A; 8] {
        let min = self.minimum;
        let lub = self.least_upper_bound();

        [
            Vec3A::new(min.x, min.y, min.z),
            Vec3A::new(lub.x, min.y, min.z),
            Vec3A::new(min.x, lub.y, min.z),
            Vec3A::new(lub.x, lub.y, min.z),
            Vec3A::new(min.x, min.y, lub.z),
            Vec3A::new(lub.x, min.y, lub.z),
            Vec3A::new(min.x, lub.y, lub.z),
            Vec3A::new(lub.x, lub.y, lub.z),
        ]
    }
}

/* Does not work until Vec3 "is a" Vector
impl Corners<Vec3> for ilattice::extent::Extent<Vec3>
{
    #[inline]
    fn corners(&self) -> [Vec3; 8] {
        let min = self.minimum;
        let lub = self.least_upper_bound();

        [
            Vec3::new(min.x, min.y, min.z),
            Vec3::new(lub.x, min.y, min.z),
            Vec3::new(min.x, lub.y, min.z),
            Vec3::new(lub.x, lub.y, min.z),
            Vec3::new(min.x, min.y, lub.z),
            Vec3::new(lub.x, min.y, lub.z),
            Vec3::new(min.x, lub.y, lub.z),
            Vec3::new(lub.x, lub.y, lub.z),
        ]
    }
}*/
