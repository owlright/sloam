#include "utility.h"
#include "sloam/directNDTLO.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> // fromROSMsg
#include <pcl/filters/filter.h>              // removeNaNFromPointCloud

std::ofstream logFile;
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
    ScanRegistration()
        : nh_("~") {
        subLaserCloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100,
                                                                 &ScanRegistration::laserCloudHandler, this);
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<sloam::PointXYZIR>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullCloud->points.resize(N_SCAN * Horizon_SCAN);
        pubFullCloud = nh_.advertise<sensor_msgs::PointCloud2>("/fullCloud", 1);
        logFile.open("scanRegistrationLog.txt");
        if (!logFile.is_open()) {
            ROS_ERROR("Failed to open log file!");
            ros::shutdown();
        }
    }
    ~ScanRegistration() {
        if (logFile.is_open()) {
            logFile.close();
        }
    }

private:
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
        DEBUG_CHECK_SEQ(laserCloudMsg->header.seq, 1, 10);
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        SE3 pose;
        ndt_lo_.AddCloud(laserCloudIn, pose);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        removeClosePointCloud<PointType>(*laserCloudIn, *laserCloudIn, 0.01);
        auto curSeq = laserCloudMsg->header.seq;
        std::string frame_pcd_name = "frame_" + std::to_string(curSeq) + ".pcd";
        if (pcl::io::savePCDFileASCII(frame_pcd_name, *laserCloudIn) == -1) {
            std::cerr << "Failed to save point cloud to " << frame_pcd_name << std::endl;
        }

        if (useCloudRing == true) {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false) {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }
        }
        int cloudSize = laserCloudIn->points.size();
        // clang-format off
        double orientationStart = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        double orientationEnd   = -atan2(laserCloudIn->points[cloudSize - 1].y, laserCloudIn->points[cloudSize - 1].x) + 2 * M_PI;
        // clang-format on
        ROS_INFO("start_x: %.3f, start_y: %.3f", laserCloudIn->points[0].x, laserCloudIn->points[0].y);
        ROS_INFO("end_x: %.3f, end_y: %.3f", laserCloudIn->points[cloudSize - 1].x, laserCloudIn->points[cloudSize - 1].y);
        ROS_INFO("Seq %u orientation start: %.3f, end: %.3f", curSeq, orientationStart, orientationEnd);
        if (orientationEnd - orientationStart > 3 * M_PI) {
            orientationEnd -= 2 * M_PI;
        } else if (orientationEnd - orientationStart < M_PI) {
            orientationEnd += 2 * M_PI;
        }
        double orientationDiff = orientationEnd - orientationStart;

        bool halfPassed = false;
        int count = cloudSize;
        PointType point;
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCAN);
        for (int i = 0; i < cloudSize; i++) {
            point.x = laserCloudIn->points[i].x;
            point.y = laserCloudIn->points[i].y;
            point.z = laserCloudIn->points[i].z;
            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (N_SCAN == 16) {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (N_SCAN - 1) || scanID < 0) {
                    count--;
                    continue;
                }
            } else if (N_SCAN == 32) {
                scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
                if (scanID > (N_SCAN - 1) || scanID < 0) {
                    count--;
                    continue;
                }
            } else if (N_SCAN == 64) {
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = N_SCAN / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                // use [0 50]  > 50 remove outlies
                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0) {
                    count--;
                    continue;
                }
            } else {
                printf("wrong scan number\n");
                ROS_BREAK();
            }
            // printf("angle %f scanID %d \n", angle, scanID);

            float ori = -atan2(point.y, point.x);
            // logFile << "Point " << i << ": ori = " << ori << std::endl;
            if (!halfPassed) {
                if (ori < orientationStart - M_PI / 2) {
                    ori += 2 * M_PI;
                } else if (ori > orientationStart + M_PI * 3 / 2) {
                    ori -= 2 * M_PI;
                }

                if (ori - orientationStart > M_PI) {
                    halfPassed = true;
                }
            } else {
                ori += 2 * M_PI;
                if (ori < orientationEnd - M_PI * 3 / 2) {
                    ori += 2 * M_PI;
                } else if (ori > orientationEnd + M_PI / 2) {
                    ori -= 2 * M_PI;
                }
            }

            float relTime = (ori - orientationStart) / (orientationEnd - orientationStart);
            point.intensity = scanID + 0.1 * relTime;
            laserCloudScans[scanID].push_back(point);
        }
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
                verticalAngle
                    = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                if (verticalAngle + ang_bottom < 0) continue;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
                if (rowIdn >= N_SCAN) continue;
            }

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
            if (columnIdn >= Horizon_SCAN) columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN) continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange) continue;

            // rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            // fullInfoCloud->points[index] = thisPoint;
            // fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as
            // "intensity"
        }
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*fullCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time().fromSec(0);
        laserCloudTemp.header.frame_id = "base_link";
        pubFullCloud.publish(laserCloudTemp);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber subLaserCloud_;

    ros::Publisher pubFullCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<sloam::PointXYZIR>::Ptr laserCloudInRing;
    pcl::PointCloud<PointType>::Ptr fullCloud;

    sloam::DirectNDTLO ndt_lo_;
};

} // namespace sloam

int main(int argc, char** argv) {
    ros::init(argc, argv, "scanRegistrationNode");
    sloam::ScanRegistration scanRegistration;
    ros::spin();
    return 0;
}
