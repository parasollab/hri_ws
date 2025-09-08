#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <filesystem> // C++17 feature

class PointCloudSubscriber : public rclcpp::Node
{
public:
    PointCloudSubscriber(const std::string &topic_name) : Node("pointcloud_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, 10, std::bind(&PointCloudSubscriber::topic_callback, this, std::placeholders::_1));
        octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_binary", 10);

        // Create directory if it doesn't exist
        std::filesystem::create_directories("maps");
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
        pcl::fromROSMsg(*msg, pcl_pointcloud);

        // Save to a PCD file
        std::string pcd_filename = "maps/map.pcd";
        pcl::io::savePCDFileASCII(pcd_filename, pcl_pointcloud);
        RCLCPP_INFO(this->get_logger(), "Saved point cloud to %s", pcd_filename.c_str());

        // Convert PCL point cloud to OctoMap
        octomap::OcTree tree(0.1); // Resolution of 0.1 meters
        for (const auto &point : pcl_pointcloud.points)
        {
            tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
        }

        // Save the OctoMap to a file
        std::string bt_filename = "maps/map.bt";
        tree.writeBinary(bt_filename);
        RCLCPP_INFO(this->get_logger(), "Saved OctoMap to %s", bt_filename.c_str());

        // Publish the OctoMap
        auto octomap_msg = std::make_shared<octomap_msgs::msg::Octomap>();
        octomap_msgs::fullMapToMsg(tree, *octomap_msg);
        octomap_msg->header.frame_id = "map";
        octomap_msg->header.stamp = this->now();
        octomap_publisher_->publish(*octomap_msg);
        RCLCPP_INFO(this->get_logger(), "Published OctoMap to octomap_binary topic");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: pointcloud_subscriber <point_cloud_topic>");
        return 1;
    }

    std::string topic_name = argv[1];
    rclcpp::spin(std::make_shared<PointCloudSubscriber>(topic_name));
    rclcpp::shutdown();
    return 0;
}
