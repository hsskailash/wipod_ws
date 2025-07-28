#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "TransverseMercatorProjector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class WaypointProcessor : public rclcpp::Node {
public:
    WaypointProcessor() : Node("waypoint_processor") {
        // Parameters
        declare_parameter("distance_threshold", 7.0);
        declare_parameter("map_origin_lat", 12.923903488321232);
        declare_parameter("map_origin_lon", 77.50052742264235);
        declare_parameter("map_origin_alt", 0.0);

        // Subscribers
        osm_waypoints_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/osm_waypoints", 10,
            std::bind(&WaypointProcessor::osm_waypoints_cb, this, std::placeholders::_1));
            
        current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current_pose", 10,
            std::bind(&WaypointProcessor::current_pose_cb, this, std::placeholders::_1));

        // Publishers
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_markers", 10);

        // Initialize projector
        auto origin = lanelet::Origin({
            get_parameter("map_origin_lat").as_double(),
            get_parameter("map_origin_lon").as_double(),
            get_parameter("map_origin_alt").as_double()
        });
        projector_ = std::make_shared<lanelet::projection::TransverseMercatorProjector>(origin);
    }

private:
    void osm_waypoints_cb(const nav_msgs::msg::Path::SharedPtr msg) {
        // Convert all waypoints to local coordinates
        local_waypoints_.clear();
        current_goal_index_ = 0;
        
        for (const auto& pose : msg->poses) {
            auto gps_point = lanelet::GPSPoint{
                pose.pose.position.x,  // Latitude
                pose.pose.position.y,  // Longitude
                0.0
            };
            
            auto projected = projector_->forward(gps_point);
            
            geometry_msgs::msg::PoseStamped local_pose;
            local_pose.pose.position.x = projected.x();
            local_pose.pose.position.y = projected.y();
            local_pose.pose.orientation = pose.pose.orientation; // Keep orientation
            local_pose.header = pose.header;
            local_pose.header.frame_id = "map"; // Update frame
            
            local_waypoints_.push_back(local_pose);
        }
        
        if (!local_waypoints_.empty()) {
            // Send first waypoint immediately
            auto first_goal = local_waypoints_[0];
            first_goal.header.stamp = now();
            goal_pub_->publish(first_goal);
            publish_markers();
        }
    }

    void current_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        
        if (local_waypoints_.empty() || current_goal_index_ >= local_waypoints_.size())
            return;

        // Calculate distance to current goal
        auto& current_goal = local_waypoints_[current_goal_index_];
        double dx = current_pose_.pose.position.x - current_goal.pose.position.x;
        double dy = current_pose_.pose.position.y - current_goal.pose.position.y;
        double distance = std::hypot(dx, dy);

        double threshold = get_parameter("distance_threshold").as_double();
        if (distance < threshold) {
            current_goal_index_++;
            if (current_goal_index_ < local_waypoints_.size()) {
                auto next_goal = local_waypoints_[current_goal_index_];
                next_goal.header.stamp = now();
                goal_pub_->publish(next_goal);
                publish_markers();
            }
        }
    }

    void publish_markers() {
        visualization_msgs::msg::MarkerArray markers;
        
        // Point markers (spheres)
        visualization_msgs::msg::Marker point_marker;
        point_marker.header.frame_id = "map";
        point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker.action = visualization_msgs::msg::Marker::ADD;
        point_marker.scale.x = point_marker.scale.y = point_marker.scale.z = 0.5;
        point_marker.color.a = 1.0;

        // Direction markers (arrows)
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = "map";
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.scale.x = 1.0; // Length
        arrow_marker.scale.y = 0.1; // Width
        arrow_marker.scale.z = 0.1; // Height
        arrow_marker.color.a = 0.8;

        for (size_t i = 0; i < local_waypoints_.size(); ++i) {
            // Point marker
            point_marker.id = i * 2;
            point_marker.pose = local_waypoints_[i].pose;
            
            // Color coding: current goal is green, others are red
            if (i == current_goal_index_) {
                point_marker.color.r = 0.0;
                point_marker.color.g = 1.0;
                point_marker.color.b = 0.0;
            } else {
                point_marker.color.r = 1.0;
                point_marker.color.g = 0.0;
                point_marker.color.b = 0.0;
            }
            markers.markers.push_back(point_marker);

            // Direction arrow
            if (i < local_waypoints_.size() - 1) {
                arrow_marker.id = i * 2 + 1;
                arrow_marker.pose = local_waypoints_[i].pose;
                arrow_marker.color.r = 0.0;
                arrow_marker.color.g = 0.0;
                arrow_marker.color.b = 1.0;
                markers.markers.push_back(arrow_marker);
            }
        }
        
        markers_pub_->publish(markers);
    }

    // Member variables
    std::vector<geometry_msgs::msg::PoseStamped> local_waypoints_;
    size_t current_goal_index_ = 0;
    geometry_msgs::msg::PoseStamped current_pose_;
    std::shared_ptr<lanelet::projection::TransverseMercatorProjector> projector_;
    
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr osm_waypoints_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}