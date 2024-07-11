#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <random>

class PointCloudPublisher : public rclcpp::Node
{
public:
    PointCloudPublisher() : Node("pointcloud_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000), std::bind(&PointCloudPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto msg = sensor_msgs::msg::PointCloud2();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        msg.height = 1;
        msg.width = 100;
        msg.is_bigendian = false;
        msg.point_step = 3 * sizeof(float);
        msg.row_step = msg.point_step * msg.width;
        msg.is_dense = true;

        sensor_msgs::PointCloud2Modifier pcd_modifier(msg);
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);

        for (size_t i = 0; i < msg.width; ++i, ++iter_x, ++iter_y, ++iter_z)
        {
            *iter_x = dis(gen);
            *iter_y = dis(gen);
            *iter_z = dis(gen);
        }

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing point cloud data");
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}
