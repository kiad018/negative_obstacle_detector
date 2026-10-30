#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <limits>
#include <sstream>
#include <iomanip>

struct RayPoint { float x, y, z, range_2d; };

class NegativeObstacleDetector : public rclcpp::Node {
public:
    NegativeObstacleDetector() : Node("detector_node") {
        this->declare_parameter<double>("drop_threshold", 0.42);
        this->declare_parameter<double>("min_range", 1.0);
        this->declare_parameter<double>("max_range", 15.0);
        this->declare_parameter<int>("num_azimuth_bins", 64);

        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_raw", 10, std::bind(&NegativeObstacleDetector::cloudCallback, this, std::placeholders::_1));

        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/negative_obstacles_marker", 10);
        pub_warning_ = this->create_publisher<std_msgs::msg::Bool>("/hazard_warning", 10);
        pub_distance_ = this->create_publisher<std_msgs::msg::Float32>("/hazard_distance", 10);
        pub_slope_ = this->create_publisher<std_msgs::msg::Float32>("/terrain_slope_pct", 10);

        RCLCPP_INFO(this->get_logger(), "Detektor es lejtesmero elindult.");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        double drop_threshold = this->get_parameter("drop_threshold").as_double();
        double min_r = this->get_parameter("min_range").as_double();
        double max_r = this->get_parameter("max_range").as_double();
        int num_bins = this->get_parameter("num_azimuth_bins").as_int();

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *raw_cloud);

        float sum_z_near = 0.0f, sum_z_far = 0.0f;
        int cnt_near = 0, cnt_far = 0;
        float sum_z_left = 0.0f, sum_z_right = 0.0f;
        int cnt_left = 0, cnt_right = 0;

        std::map<int, std::vector<RayPoint>> azimuth_bins;
        for (const auto& pt : raw_cloud->points) {
            if (pt.x <= 0.6f) continue;
            float r2d = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            if (r2d < min_r || r2d > max_r) continue;

            if (std::abs(pt.y) < 0.8f) {
                if (pt.x >= 1.5f && pt.x <= 2.5f) { sum_z_near += pt.z; cnt_near++; }
                else if (pt.x >= 5.0f && pt.x <= 6.5f) { sum_z_far += pt.z; cnt_far++; }
            }
            if (pt.x >= 2.0f && pt.x <= 4.5f) {
                if (pt.y > 1.0f && pt.y < 2.5f) { sum_z_left += pt.z; cnt_left++; }
                else if (pt.y < -1.0f && pt.y > -2.5f) { sum_z_right += pt.z; cnt_right++; }
            }

            float angle = std::atan2(pt.y, pt.x);
            int bin_idx = std::clamp(static_cast<int>((angle + 0.85f) / 1.70f * num_bins), 0, num_bins - 1);
            azimuth_bins[bin_idx].push_back({pt.x, pt.y, pt.z, r2d});
        }

        float slope_pct = (cnt_near > 0 && cnt_far > 0) ? ((sum_z_far / cnt_far - sum_z_near / cnt_near) / 4.0f) * 100.0f : 0.0f;
        float bank_pct = (cnt_left > 0 && cnt_right > 0) ? ((sum_z_left / cnt_left - sum_z_right / cnt_right) / 3.5f) * 100.0f : 0.0f;

        visualization_msgs::msg::MarkerArray marker_array;
        std::vector<geometry_msgs::msg::Point> ditch_pts;
        float min_dist = std::numeric_limits<float>::max();
        float max_depth = 0.0f;
        int left_cnt = 0, right_cnt = 0;
        bool hazard = false;

        for (auto& [bin, points] : azimuth_bins) {
            std::sort(points.begin(), points.end(), [](const RayPoint& a, const RayPoint& b) { return a.range_2d < b.range_2d; });
            for (size_t i = 1; i < points.size(); ++i) {
                float dz = points[i].z - points[i - 1].z;
                float dr = points[i].range_2d - points[i - 1].range_2d;
                float local_slope = (dr > 0.02f) ? (dz / dr) : 0.0f;

                if (dz < -drop_threshold && local_slope < -1.45f) {
                    geometry_msgs::msg::Point p;
                    p.x = points[i - 1].x; p.y = points[i - 1].y; p.z = points[i - 1].z;
                    ditch_pts.push_back(p);
                    hazard = true;
                    if (p.y >= 0.0) left_cnt++; else right_cnt++;
                    if (std::abs(dz) > max_depth) max_depth = std::abs(dz);
                    if (points[i - 1].range_2d < min_dist) min_dist = points[i - 1].range_2d;
                    break;
                }
            }
        }

        // Vonalmarker
        visualization_msgs::msg::Marker line_m;
        line_m.header = msg->header; line_m.ns = "ditch_line"; line_m.id = 0;
        line_m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_m.action = visualization_msgs::msg::Marker::ADD;
        line_m.scale.x = 0.16; line_m.color.r = 1.0f; line_m.color.a = 1.0f;
        line_m.points = ditch_pts;
        marker_array.markers.push_back(line_m);

        // Pontmarker
        visualization_msgs::msg::Marker sph_m;
        sph_m.header = msg->header; sph_m.ns = "ditch_spheres"; sph_m.id = 1;
        sph_m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        sph_m.action = visualization_msgs::msg::Marker::ADD;
        sph_m.scale.x = 0.32; sph_m.scale.y = 0.32; sph_m.scale.z = 0.32;
        sph_m.color.r = 1.0f; sph_m.color.a = 0.95f;
        sph_m.points = ditch_pts;
        marker_array.markers.push_back(sph_m);

        // 3D HUD
        visualization_msgs::msg::Marker hud_m;
        hud_m.header = msg->header; hud_m.ns = "hud"; hud_m.id = 2;
        hud_m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        hud_m.action = visualization_msgs::msg::Marker::ADD;
        hud_m.pose.position.x = 1.2; hud_m.pose.position.y = 0.0; hud_m.pose.position.z = 2.2;
        hud_m.scale.z = 0.48; hud_m.color.a = 1.0f;

        std::stringstream ss;
        ss << std::fixed << std::setprecision(1);
        std::string s_type = (slope_pct >= 0.0f) ? "CLIMB: +" : "DESCENT: ";
        ss << "[ TERRAIN SLOPE: " << s_type << static_cast<int>(slope_pct) << "% | BANK: " << static_cast<int>(bank_pct) << "% ]\n";

        if (hazard) {
            std::string o_type = "ROAD TRENCH";
            if (max_depth > 1.35f) o_type = "EXTREME RAVINE";
            else if (left_cnt == 0 && right_cnt > 0) o_type = "RIGHT LANE COLLAPSE";
            else if (right_cnt == 0 && left_cnt > 0) o_type = "LEFT LANE COLLAPSE";

            std::string action = (min_dist < 4.5f) ? "EMERGENCY BRAKE" : "DECELERATE";
            if (o_type == "RIGHT LANE COLLAPSE") action = "STEER LEFT (ESCAPE)";
            else if (o_type == "LEFT LANE COLLAPSE") action = "STEER RIGHT (ESCAPE)";

            hud_m.color.r = 1.0f; hud_m.color.g = (min_dist < 4.5f) ? 0.0f : 0.85f; hud_m.color.b = 0.0f;
            ss << std::setprecision(2) << "TYPE: " << o_type << "\nDIST: " << min_dist << " m | DEPTH: " << max_depth << " m\nACTION: " << action;
        } else {
            hud_m.color.r = 0.1f; hud_m.color.g = 1.0f; hud_m.color.b = 0.2f;
            ss << "STATUS: ROAD CLEAR / PASSABLE\nNO HAZARDS IN PATH";
        }

        hud_m.text = ss.str();
        marker_array.markers.push_back(hud_m);
        pub_markers_->publish(marker_array);

        std_msgs::msg::Bool w_msg; w_msg.data = hazard; pub_warning_->publish(w_msg);
        std_msgs::msg::Float32 d_msg; d_msg.data = hazard ? min_dist : -1.0f; pub_distance_->publish(d_msg);
        std_msgs::msg::Float32 sl_msg; sl_msg.data = slope_pct; pub_slope_->publish(sl_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_warning_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_distance_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slope_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NegativeObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}
