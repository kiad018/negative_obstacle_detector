#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <vector>

class MockTerrainPublisher : public rclcpp::Node {
public:
    MockTerrainPublisher() : Node("mock_terrain_node") {
        this->declare_parameter<double>("vehicle_speed", 2.0);

        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_raw", 10);
        pub_vehicle_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vehicle_marker", 10);

        world_x_ = 0.0;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(70),
            std::bind(&MockTerrainPublisher::publishLoop, this));

        RCLCPP_INFO(this->get_logger(), "Aszimmetrikus terep es Pickup generator elindult.");
    }

private:
    float getTerrainHeight(float wx, float wy) {
        float main_hills = 1.35f * std::sin(0.18f * wx) + 0.55f * std::sin(0.42f * wx + 0.6f);
        float side_bank = (0.35f * std::sin(0.24f * wx + 1.2f) + 0.18f) * wy + 0.20f * std::sin(0.75f * wy);
        return main_hills + side_bank;
    }

    void publishLoop() {
        double speed = this->get_parameter("vehicle_speed").as_double();
        world_x_ += speed * 0.07;

        float ground_at_car = getTerrainHeight(world_x_, 0.0f);
        float ground_in_front = getTerrainHeight(world_x_ + 2.5f, 0.0f);
        float pitch_rad = std::atan2(ground_in_front - ground_at_car, 2.5f);

        pcl::PointCloud<pcl::PointXYZI> cloud;

        for (float r = 1.0f; r <= 15.5f; r += 0.22f) {
            for (float angle = -0.82f; angle <= 0.82f; angle += 0.026f) {
                float lx = r * std::cos(angle);
                float ly = r * std::sin(angle);

                float wx = world_x_ + lx;
                float raw_z = getTerrainHeight(wx, ly);

                float obs_cycle = std::fmod(wx, 60.0f);
                float depth_drop = 0.0f;

                if (obs_cycle >= 14.0f && obs_cycle <= 17.0f) {
                    depth_drop = 1.30f;
                } else if (obs_cycle >= 32.0f && obs_cycle <= 35.5f) {
                    if (ly > -3.2f && ly < 0.5f) {
                        depth_drop = 0.90f;
                    }
                } else if (obs_cycle >= 48.0f && obs_cycle <= 51.5f) {
                    if (ly > -0.5f && ly < 3.2f) {
                        depth_drop = 1.45f;
                    }
                }

                float rel_z = (raw_z - depth_drop) - ground_at_car - (lx * std::tan(pitch_rad)) - 1.50f;

                pcl::PointXYZI pt;
                pt.x = lx;
                pt.y = ly;
                pt.z = rel_z;
                pt.intensity = (depth_drop > 0.1f) ? 15.0f : (100.0f + rel_z * 35.0f);
                cloud.points.push_back(pt);
            }
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "base_link";
        msg.header.stamp = this->now();
        pub_cloud_->publish(msg);

        publishPickup();
    }

    void publishPickup() {
        visualization_msgs::msg::MarkerArray truck;
        auto stamp = this->now();

        auto add_part = [&](int id, double x, double y, double z, double sx, double sy, double sz,
                            float r, float g, float b, float a, int type = visualization_msgs::msg::Marker::CUBE) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "base_link";
            m.header.stamp = stamp;
            m.ns = "pickup";
            m.id = id;
            m.type = type;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = x;
            m.pose.position.y = y;
            m.pose.position.z = z;
            m.pose.orientation.w = 1.0;
            m.scale.x = sx; m.scale.y = sy; m.scale.z = sz;
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
            truck.markers.push_back(m);
        };

        add_part(0, -0.6, 0.0, -0.72, 2.2, 1.85, 0.65, 0.22f, 0.26f, 0.30f, 1.0f);
        add_part(1, -0.85, 0.0, -0.18, 1.35, 1.60, 0.55, 0.10f, 0.12f, 0.15f, 0.95f);
        add_part(2, -2.4, 0.0, -0.82, 1.8, 1.85, 0.45, 0.18f, 0.22f, 0.25f, 1.0f);
        add_part(3, -1.6, 0.0, -0.22, 0.15, 1.65, 0.65, 0.05f, 0.05f, 0.05f, 1.0f);
        add_part(4, 0.55, 0.0, -0.85, 0.25, 1.80, 0.35, 0.08f, 0.08f, 0.08f, 1.0f);

        add_part(5,  0.0,  0.96, -1.15, 0.88, 0.38, 0.88, 0.05f, 0.05f, 0.05f, 1.0f);
        add_part(6,  0.0, -0.96, -1.15, 0.88, 0.38, 0.88, 0.05f, 0.05f, 0.05f, 1.0f);
        add_part(7, -2.4,  0.96, -1.15, 0.88, 0.38, 0.88, 0.05f, 0.05f, 0.05f, 1.0f);
        add_part(8, -2.4, -0.96, -1.15, 0.88, 0.38, 0.88, 0.05f, 0.05f, 0.05f, 1.0f);

        add_part(9,  0.48,  0.68, -0.65, 0.12, 0.35, 0.18, 1.0f, 0.95f, 0.65f, 1.0f);
        add_part(10, 0.48, -0.68, -0.65, 0.12, 0.35, 0.18, 1.0f, 0.95f, 0.65f, 1.0f);

        add_part(11, 0.0, 0.0, 0.12, 0.30, 0.30, 0.18, 0.95f, 0.60f, 0.0f, 1.0f, visualization_msgs::msg::Marker::CYLINDER);

        pub_vehicle_->publish(truck);
    }

    double world_x_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vehicle_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockTerrainPublisher>());
    rclcpp::shutdown();
    return 0;
}
