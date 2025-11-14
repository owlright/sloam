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
        sub_laser_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100,
            &ScanRegistration::laserCloudHandler, this);
        laser_cloud_in_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    ~ScanRegistration() { }

private:
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laser_cloud_msg) {
        pcl::fromROSMsg(*laser_cloud_msg, *laser_cloud_in_);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laser_cloud_in_, *laser_cloud_in_, indices);

        double orientation_start = -atan2(laser_cloud_in_->points[0].y, laser_cloud_in_->points[0].x);
        double orientation_end   = -atan2(
            laser_cloud_in_->points[laser_cloud_in_->points.size() - 1].y,
            laser_cloud_in_->points[laser_cloud_in_->points.size() - 1].x) + 2 * M_PI;
        ROS_INFO("orientation start: %.3f, end: %.3f", orientation_start, orientation_end);
        if (orientation_end - orientation_start > 3 * M_PI) {
            orientation_end -= 2 * M_PI;
        } else if (orientation_end - orientation_start < M_PI) {
            orientation_end += 2 * M_PI;
        }
        double orientation_diff = orientation_end - orientation_start;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_laser_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_in_;
};

} // namespace sloam

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_registration");
    sloam::ScanRegistration scan_registration;
    ros::spin();
    return 0;
}
