#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"
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
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = pcl::PointCloud<PointType>::Ptr;
// pose represented as sophus structs
using SE2 = Sophus::SE2d;
using SE2f = Sophus::SE2f;
using SO2 = Sophus::SO2d;
using SE3 = Sophus::SE3d;
using SE3f = Sophus::SE3f;
using SO3 = Sophus::SO3d;

using Quatd = Eigen::Quaterniond;
using Quatf = Eigen::Quaternionf;
/*
 * A point cloud type that has "ring" channel
 */
#if PCL_VERSION_COMPARE(<=, 1, 10, 0)
struct PointXYZIR {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
#else
struct EIGEN_ALIGN16 PointXYZIR {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif

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

template <typename S>
inline SE3 Mat4ToSE3(const Eigen::Matrix<S, 4, 4>& m) {
    /// 对R做归一化，防止sophus里的检查不过
    Quatd q(m.template block<3, 3>(0, 0).template cast<double>());
    q.normalize();
    return SE3(q, m.template block<3, 1>(0, 3).template cast<double>());
}
} // namespace sloam

// 注册必须要在命名空间外面进行
POINT_CLOUD_REGISTER_POINT_STRUCT (sloam::PointXYZIR,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (std::uint16_t, ring, ring))