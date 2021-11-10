use crate::PB_Vertex;
use cgmath::Zero;
use ilattice::glam::{IVec3, Vec3A};

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

impl From<cgmath::Point3<f64>> for PB_Vertex {
    fn from(other: cgmath::Point3<f64>) -> PB_Vertex {
        PB_Vertex {
            x: other.x,
            y: other.y,
            z: other.z,
        }
    }
}

impl PB_Vertex {
    #[inline(always)]
    pub fn distance_squared(&self, other: &PB_Vertex) -> f64 {
        let x = self.x - other.x;
        let y = self.y - other.y;
        let z = self.z - other.z;
        x * x + y * y + z * z
    }
}

/// converts a Point2 to Point3 using the XY coordinates, sets Z to zero.
#[inline(always)]
pub(crate) fn xy_to_3d(point: &cgmath::Point2<f64>) -> cgmath::Point3<f64> {
    cgmath::Point3 {
        x: point.x,
        y: point.y,
        z: f64::zero(),
    }
}

/// converts a Point3 to Point2 using the XY coordinates
pub(crate) fn xy_to_2d(point: &cgmath::Point3<f64>) -> cgmath::Point2<f64> {
    cgmath::Point2 {
        x: point.x,
        y: point.y,
    }
}
