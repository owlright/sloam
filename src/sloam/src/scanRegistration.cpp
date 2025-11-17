#include "utility.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl/filters/filter.h>  // removeNaNFromPointCloud

namespace sloam {

const bool useCloudRing = true;
// VLP-16
const int N_SCAN = 16;
const int Horizon_SCAN = 1800;
const float ang_res_x = 0.2;
const float ang_res_y = 2.0;
const float ang_bottom = 15.0 + 0.1;
const int groundScanInd = 7;

const float sensorMinimumRange = 1.0; // 在这个范围内的点将被舍弃

class ScanRegistration {
public:
    ScanRegistration(): nh_("~") {
        subLaserCloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100,
            &ScanRegistration::laserCloudHandler, this);
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<sloam::PointXYZIR>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullCloud->points.resize(N_SCAN * Horizon_SCAN);
    }
    ~ScanRegistration() { }

private:
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        if (useCloudRing == true) {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false) {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }
        }
        double orientationStart = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        double orientationEnd   = -atan2(
            laserCloudIn->points[laserCloudIn->points.size() - 1].y,
            laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        ROS_INFO("orientation start: %.3f, end: %.3f", orientationStart, orientationEnd);
        if (orientationEnd - orientationStart > 3 * M_PI) {
            orientationEnd -= 2 * M_PI;
        } else if (orientationEnd - orientationStart < M_PI) {
            orientationEnd += 2 * M_PI;
        }
        double orientationDiff = orientationEnd - orientationStart;
        projectPointCloud();
    }

    void projectPointCloud() {
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize;
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i) {
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            // find the row and column index in the iamge for this point
            if (useCloudRing) {
                rowIdn = laserCloudInRing->points[i].ring;
            } else {
                // z/√(x^2+y^2)
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                if (verticalAngle + ang_bottom < 0)
                    continue;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
                if (rowIdn >= N_SCAN)
                    continue;
            }

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;

            // rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            // fullInfoCloud->points[index] = thisPoint;
            // fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber subLaserCloud_;
    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<sloam::PointXYZIR>::Ptr laserCloudInRing;
    pcl::PointCloud<PointType>::Ptr fullCloud;
};

} // namespace sloam

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistrationNode");
    sloam::ScanRegistration scanRegistration;
    ros::spin();
    return 0;
}
