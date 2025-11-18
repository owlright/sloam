#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 定义调试宏
#define DEBUG_CHECK_SEQ(seq, min_seq, max_seq)                          \
    do {                                                                \
        if ((seq) < (min_seq) || (seq) > (max_seq)) {                   \
            ROS_WARN_STREAM("Skipping seq " << (seq)                   \
                             << " (out of range: [" << (min_seq) << ", " << (max_seq) << "])"); \
            return;                                                     \
        }                                                               \
    } while (0)

namespace sloam {
using PointType = pcl::PointXYZI;
/*
 * A point cloud type that has "ring" channel
 */
struct PointXYZIR {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace sloam

// 注册必须要在命名空间外面进行
POINT_CLOUD_REGISTER_POINT_STRUCT (sloam::PointXYZIR,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (std::uint16_t, ring, ring)
)