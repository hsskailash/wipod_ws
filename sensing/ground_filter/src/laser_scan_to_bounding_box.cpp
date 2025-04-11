#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
// #include <pcl/recognition/raft/raft.h>

class LaserScanToBoundingBoxNode : public rclcpp::Node
{
public:
    LaserScanToBoundingBoxNode() : Node("laser_scan_to_bounding_box")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "ouster/scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanToBoundingBoxNode::laserScanCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/bounding_boxes", rclcpp::SensorDataQoS());
    }

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Convert LaserScan to PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

        // Convert LaserScan to PointCloud by projecting onto a flat plane (add a z-coordinate of 0)
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (std::isinf(msg->ranges[i]) || msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;

            float angle = msg->angle_min + i * msg->angle_increment;
            pcl::PointXYZ point;
            point.x = msg->ranges[i] * cos(angle);
            point.y = msg->ranges[i] * sin(angle);
            point.z = 0.0;
            cloud->push_back(point);
        }

        // Apply Statistical Outlier Removal filter
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud);

        // Perform Euclidean Cluster Extraction to detect distinct objects
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setInputCloud(cloud);
        ec.setClusterTolerance(0.5);  // Adjust based on your environment
        ec.setMinClusterSize(10);     // Min number of points per cluster
        ec.setMaxClusterSize(25000);  // Max number of points per cluster
        ec.extract(cluster_indices);

        // Visualize the bounding boxes in RViz
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < cluster_indices.size(); ++i)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
            for (size_t j = 0; j < cluster_indices[i].indices.size(); ++j)
            {
                cluster->push_back(cloud->points[cluster_indices[i].indices[j]]);
            }

            // Compute bounding box for the cluster
            pcl::PointXYZ min_point, max_point;
            pcl::getMinMax3D(*cluster, min_point, max_point);

            // Create a bounding box marker
            visualization_msgs::msg::Marker box_marker;
            box_marker.header.frame_id = "base_link";
            box_marker.header.stamp = this->get_clock()->now();
            box_marker.ns = "bounding_boxes";
            box_marker.id = i;
            box_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            box_marker.action = visualization_msgs::msg::Marker::ADD;
            box_marker.scale.x = 0.1;
            box_marker.color.r = 1.0;
            box_marker.color.g = 0.0;
            box_marker.color.b = 0.0;
            box_marker.color.a = 1.0;

            // Define 8 points for the bounding box
            geometry_msgs::msg::Point p1, p2, p3, p4, p5, p6, p7, p8;
            p1.x = min_point.x;
            p1.y = min_point.y;
            p1.z = min_point.z;
            p2.x = max_point.x;
            p2.y = min_point.y;
            p2.z = min_point.z;
            p3.x = max_point.x;
            p3.y = max_point.y;
            p3.z = min_point.z;
            p4.x = min_point.x;
            p4.y = max_point.y;
            p4.z = min_point.z;
            p5.x = min_point.x;
            p5.y = min_point.y;
            p5.z = max_point.z;
            p6.x = max_point.x;
            p6.y = min_point.y;
            p6.z = max_point.z;
            p7.x = max_point.x;
            p7.y = max_point.y;
            p7.z = max_point.z;
            p8.x = min_point.x;
            p8.y = max_point.y;
            p8.z = max_point.z;

            // Create the bounding box lines
            box_marker.points.push_back(p1);
            box_marker.points.push_back(p2);
            box_marker.points.push_back(p3);
            box_marker.points.push_back(p4);
            box_marker.points.push_back(p1);  // Closing the bottom rectangle
            box_marker.points.push_back(p5);
            box_marker.points.push_back(p6);
            box_marker.points.push_back(p7);
            box_marker.points.push_back(p8);
            box_marker.points.push_back(p5);  // Closing the top rectangle
            box_marker.points.push_back(p6);
            box_marker.points.push_back(p7);
            box_marker.points.push_back(p8);
            box_marker.points.push_back(p5);  // Closing the top box

            marker_array.markers.push_back(box_marker);
        }

        publisher_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanToBoundingBoxNode>());
    rclcpp::shutdown();
    return 0;
}
