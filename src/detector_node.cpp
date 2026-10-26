#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <map>

struct RayPoint {
    float x;
    float y;
    float z;
    float range_2d;
};

class NegativeObstacleDetector : public rclcpp::Node {
public:
    NegativeObstacleDetector() : Node("negative_obstacle_detector_node") {
        this->declare_parameter<double>("sensor_height", 1.5);
        this->declare_parameter<double>("drop_threshold", 0.35);
        this->declare_parameter<double>("radial_gap_threshold", 1.0);
        this->declare_parameter<double>("min_range", 1.0);
        this->declare_parameter<double>("max_range", 15.0);
        this->declare_parameter<int>("num_azimuth_bins", 72);

        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_raw", 10,
            std::bind(&NegativeObstacleDetector::cloudCallback, this, std::placeholders::_1));

        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/negative_obstacles_marker", 10);
        pub_warning_ = this->create_publisher<std_msgs::msg::Bool>(
            "/hazard_warning", 10);

        RCLCPP_INFO(this->get_logger(), "Negativ akadaly detektalo node elindult.");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        double sensor_height = this->get_parameter("sensor_height").as_double();
        double drop_threshold = this->get_parameter("drop_threshold").as_double();
        double gap_threshold = this->get_parameter("radial_gap_threshold").as_double();
        double min_r = this->get_parameter("min_range").as_double();
        double max_r = this->get_parameter("max_range").as_double();
        int num_bins = this->get_parameter("num_azimuth_bins").as_int();

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        std::map<int, std::vector<RayPoint>> azimuth_bins;
        for (const auto& pt : pcl_cloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            if (pt.x <= 0.0) continue;

            float range_2d = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            if (range_2d < min_r || range_2d > max_r) continue;

            float angle = std::atan2(pt.y, pt.x);
            int bin_idx = static_cast<int>((angle + M_PI_2) / M_PI * num_bins);
            bin_idx = std::clamp(bin_idx, 0, num_bins - 1);

            azimuth_bins[bin_idx].push_back({pt.x, pt.y, pt.z, range_2d});
        }

        visualization_msgs::msg::MarkerArray marker_array;
        std::vector<geometry_msgs::msg::Point> ditch_edges;
        bool hazard_found = false;

        for (auto& [bin, points] : azimuth_bins) {
            std::sort(points.begin(), points.end(), [](const RayPoint& a, const RayPoint& b) {
                return a.range_2d < b.range_2d;
            });

            for (size_t i = 1; i < points.size(); ++i) {
                float delta_z = points[i].z - points[i - 1].z;
                float delta_r = points[i].range_2d - points[i - 1].range_2d;

                if (delta_z < -drop_threshold || (delta_r > gap_threshold && points[i - 1].z < -sensor_height * 0.7)) {
                    geometry_msgs::msg::Point p;
                    p.x = points[i - 1].x;
                    p.y = points[i - 1].y;
                    p.z = points[i - 1].z;
                    ditch_edges.push_back(p);
                    hazard_found = true;
                    break;
                }
            }
        }

        visualization_msgs::msg::Marker ditch_marker;
        ditch_marker.header = msg->header;
        ditch_marker.ns = "ditch_edges";
        ditch_marker.id = 0;
        ditch_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        ditch_marker.action = visualization_msgs::msg::Marker::ADD;
        ditch_marker.scale.x = 0.35;
        ditch_marker.scale.y = 0.35;
        ditch_marker.scale.z = 0.35;
        ditch_marker.color.r = 1.0f;
        ditch_marker.color.g = 0.1f;
        ditch_marker.color.b = 0.1f;
        ditch_marker.color.a = 0.95f;
        ditch_marker.points = ditch_edges;

        marker_array.markers.push_back(ditch_marker);
        pub_markers_->publish(marker_array);

        std_msgs::msg::Bool warn_msg;
        warn_msg.data = hazard_found;
        pub_warning_->publish(warn_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_warning_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NegativeObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}
