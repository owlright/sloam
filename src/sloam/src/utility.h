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

template <typename PointT>
void removeClosePointCloud(const pcl::PointCloud<PointT>& cloud_in, pcl::PointCloud<PointT>& cloud_out, float thres) {
    if (&cloud_in != &cloud_out) {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i) {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y
                + cloud_in.points[i].z * cloud_in.points[i].z
            < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size()) {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

} // namespace sloam

// 注册必须要在命名空间外面进行
POINT_CLOUD_REGISTER_POINT_STRUCT (sloam::PointXYZIR,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (std::uint16_t, ring, ring)
)