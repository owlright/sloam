#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl/filters/filter.h>  // removeNaNFromPointCloud

namespace sloam {

class ScanRegistration {
public:
    ScanRegistration(): nh_("~") {
        subLaserCloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100,
            &ScanRegistration::laserCloudHandler, this);
        laserCloudIn_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    ~ScanRegistration() { }

private:
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn_);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn_, *laserCloudIn_, indices);

        double orientationStart = -atan2(laserCloudIn_->points[0].y, laserCloudIn_->points[0].x);
        double orientationEnd   = -atan2(
            laserCloudIn_->points[laserCloudIn_->points.size() - 1].y,
            laserCloudIn_->points[laserCloudIn_->points.size() - 1].x) + 2 * M_PI;
        ROS_INFO("orientation start: %.3f, end: %.3f", orientationStart, orientationEnd);
        if (orientationEnd - orientationStart > 3 * M_PI) {
            orientationEnd -= 2 * M_PI;
        } else if (orientationEnd - orientationStart < M_PI) {
            orientationEnd += 2 * M_PI;
        }
        double orientationDiff = orientationEnd - orientationStart;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber subLaserCloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn_;
};

} // namespace sloam

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistrationNode");
    sloam::ScanRegistration scanRegistration;
    ros::spin();
    return 0;
}
