#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class MockTerrainPublisher : public rclcpp::Node {
public:
    MockTerrainPublisher() : Node("mock_terrain_publisher") {
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_raw", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MockTerrainPublisher::publishTerrain, this));
        RCLCPP_INFO(this->get_logger(), "Mock terrain publisher elindult (5 Hz).");
    }

private:
    void publishTerrain() {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        // Talaj pontok generalasa: x: 1-12m, y: -4 - 4m
        for (float x = 1.0f; x <= 12.0f; x += 0.2f) {
            for (float y = -4.0f; y <= 4.0f; y += 0.2f) {
                float z = -1.5f; // Alap talajszint a szenzorhoz kepest
                if (x >= 5.5f && x <= 7.0f) {
                    z = -2.2f; // 70 cm mely arok
                }
                cloud.points.emplace_back(x, y, z);
            }
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "base_link";
        msg.header.stamp = this->now();
        pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockTerrainPublisher>());
    rclcpp::shutdown();
    return 0;
}
