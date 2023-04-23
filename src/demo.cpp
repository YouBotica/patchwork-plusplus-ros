#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "patchworkpp/patchworkpp.hpp"

using PointType = pcl::PointXYZI;
using namespace std;

class PatchworkppGroundSeg : public rclcpp::Node {
  public:
    PatchworkppGroundSeg() : Node("patchworkpp_demo") {
        std::string cloud_topic = declare_parameter<string>("cloud_topic", "pointcloud");
        patchworkpp_instance = std::make_shared<PatchWorkpp<PointType>>(this);
        pub_cloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            "cloud", rclcpp::SystemDefaultsQoS());
        pub_ground = create_publisher<sensor_msgs::msg::PointCloud2>(
            "ground", rclcpp::SystemDefaultsQoS());
        pub_non_ground = create_publisher<sensor_msgs::msg::PointCloud2>(
            "nonground", rclcpp::SystemDefaultsQoS());
        sub_cloud = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic,
            rclcpp::SensorDataQoS(), std::bind(&PatchworkppGroundSeg::callbackCloud, this, std::placeholders::_1));
    };

  private:
    std::shared_ptr<PatchWorkpp<PointType>> patchworkpp_instance;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud, pub_ground, pub_non_ground;
    rclcpp::SubscriptionBase::SharedPtr sub_cloud;

    template<typename T>
    sensor_msgs::msg::PointCloud2 cloud2msg(const pcl::PointCloud<T>& cloud, const std::string& frame_id = "map") {
        sensor_msgs::msg::PointCloud2 cloud_ROS;
        pcl::toROSMsg(cloud, cloud_ROS);
        cloud_ROS.header.frame_id = frame_id;
        return cloud_ROS;
    }

    void callbackCloud(std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_msg)
    {
        double time_taken;

        pcl::PointCloud<PointType> pc_curr;
        pcl::PointCloud<PointType> pc_ground;
        pcl::PointCloud<PointType> pc_non_ground;
        
        pcl::fromROSMsg(*cloud_msg, pc_curr);

        patchworkpp_instance->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

        cout << "\033[1;32m" << "Result: Input PointCloud: " << pc_curr.size() << " -> Ground: " << pc_ground.size() <<  "/ NonGround: " << pc_non_ground.size()
            << " (running_time: " << time_taken << " sec)" << "\033[0m" << endl;

        pub_cloud->publish(cloud2msg(pc_curr, cloud_msg->header.frame_id));
        pub_ground->publish(cloud2msg(pc_ground, cloud_msg->header.frame_id));
        pub_non_ground->publish(cloud2msg(pc_non_ground, cloud_msg->header.frame_id));
    }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatchworkppGroundSeg>());
  rclcpp::shutdown();
  return 0;
}
