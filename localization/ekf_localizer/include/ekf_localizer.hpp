#ifndef EKF_LOCALIZER_HPP_
#define EKF_LOCALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <queue>

class EKFLocalizer : public rclcpp::Node
{
public:
  explicit EKFLocalizer(const rclcpp::NodeOptions & options);

private:
  // Parameters
  double predict_frequency_;
  double pose_gate_dist_;
  double twist_gate_dist_;
  bool enable_yaw_bias_estimation_;
  double pose_additional_delay_;
  double twist_additional_delay_;
  size_t pose_smoothing_steps_;
  size_t twist_smoothing_steps_;
  double proc_stddev_vx_c_;
  double proc_stddev_wz_c_;
  double proc_stddev_yaw_c_;
  double proc_stddev_yaw_bias_;
  double z_filter_proc_dev_;
  double roll_filter_proc_dev_;
  double pitch_filter_proc_dev_;
  double pose_delay_threshold_;
  double twist_delay_threshold_;
  rclcpp::TimerBase::SharedPtr predict_timer_;
  std::mutex state_mutex_;
  // Topic names
  std::string pose_topic_;
  std::string twist_topic_;
  std::string pose_pub_topic_;
  std::string pose_cov_pub_topic_;
  std::string odom_pub_topic_;
  std::string twist_pub_topic_;
  std::string twist_cov_pub_topic_;
  std::string diag_pub_topic_;

  // State vector: [x, y, yaw, yaw_bias, vx, wz]
  Eigen::VectorXd state_;
  Eigen::MatrixXd covariance_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_cov_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Queues for smooth updates
  std::queue<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> pose_queue_;
  std::queue<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> twist_queue_;

  // Buffers for delayed states and covariances
  std::vector<std::pair<Eigen::VectorXd, rclcpp::Time>> delayed_states_;
  std::vector<Eigen::MatrixXd> covariance_buffer_;

  // Diagnostics variables
  double pose_delay_;
  double pose_mahalanobis_distance_;

  // Callbacks
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  // EKF Functions
  void predict(double dt);
  bool updatePose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose, double delay_time);
  bool updateTwist(const geometry_msgs::msg::TwistWithCovarianceStamped & twist, double delay_time);
  void publishEstimate();
  void publishDiagnostics();

  // Helper Functions
  double mahalanobisDistance(const Eigen::VectorXd & x, const Eigen::VectorXd & y, const Eigen::MatrixXd & C);
  void compensateRPHDelay(geometry_msgs::msg::PoseWithCovarianceStamped & pose, const tf2::Vector3 & angular_velocity, double delay_time);
  size_t find_closest_delay_time_index(double target_delay);
  bool is_measurement_valid(const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & P, const Eigen::MatrixXd & R, double threshold, const Eigen::VectorXd & delayed_state);

  // Parameter declaration
  void declare_parameters();
  void broadcastTransform();
  void timerCallback();
  void processPoseQueue();
  void processTwistQueue();

  // Diagnostics functions
  diagnostic_msgs::msg::DiagnosticStatus check_measurement_delay(const std::string& measurement_type, double delay_time, double threshold);
  diagnostic_msgs::msg::DiagnosticStatus check_mahalanobis_distance(const std::string& measurement_type, double distance, double threshold);
};

#endif  // EKF_LOCALIZER_HPP_