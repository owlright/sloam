#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
class ScanRegistration {
public:
    ScanRegistration(): nh_("~") {
        sub_laser_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 100,
            &ScanRegistration::laserCloudHandler, this);
        laser_cloud_in_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    ~ScanRegistration() { }
private:
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laser_cloud_msg) {
        pcl::fromROSMsg(*laser_cloud_msg, *laser_cloud_in_);
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_laser_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_in_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_registration");
    ScanRegistration scan_registration;
    ros::spin();
    return 0;
}
