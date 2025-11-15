#pragma once
#include <pcl/point_types.h>
namespace sloam {
using PointType = pcl::PointXYZI;
/*
 * A point cloud type that has "ring" channel
 */
struct PointXYZIR {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace sloam

// 注册必须要在命名空间外面进行
POINT_CLOUD_REGISTER_POINT_STRUCT (sloam::PointXYZIR,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (std::uint16_t, ring, ring)
)