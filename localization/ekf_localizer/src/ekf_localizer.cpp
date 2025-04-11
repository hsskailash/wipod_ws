#include "ekf_localizer.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <queue>
#include <algorithm>

using namespace std::chrono_literals;

EKFLocalizer::EKFLocalizer(const rclcpp::NodeOptions & options)
: Node("ekf_localizer", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Declare parameters
  declare_parameters();

  // Initialize state and covariance
  state_ = Eigen::VectorXd::Zero(6);  // x, y, yaw, yaw_bias, vx, wz
  covariance_ = Eigen::MatrixXd::Identity(6, 6) * 1e6;

  // Subscribers
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_topic_, 10, std::bind(&EKFLocalizer::poseCallback, this, std::placeholders::_1));
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    twist_topic_, 10, std::bind(&EKFLocalizer::twistCallback, this, std::placeholders::_1));

  // Publishers
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_pub_topic_, 10);
  pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_cov_pub_topic_, 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_pub_topic_, 10);
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_pub_topic_, 10);
  twist_cov_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(twist_cov_pub_topic_, 10);
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diag_pub_topic_, 10);

  // TF Broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  const double predict_dt = 1.0 / predict_frequency_;
  predict_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(predict_dt),
    std::bind(&EKFLocalizer::timerCallback, this)
  );

  // Log initialization
  RCLCPP_INFO(this->get_logger(), "EKF Localizer initialized successfully.");
}

void EKFLocalizer::declare_parameters() {
  // Prediction parameters
  predict_frequency_ = this->declare_parameter<double>("predict_frequency", 50.0);
  proc_stddev_vx_c_ = this->declare_parameter<double>("proc_stddev_vx_c", 0.1);
  proc_stddev_wz_c_ = this->declare_parameter<double>("proc_stddev_wz_c", 0.1);
  proc_stddev_yaw_c_ = this->declare_parameter<double>("proc_stddev_yaw_c", 0.1);
  proc_stddev_yaw_bias_ = this->declare_parameter<double>("proc_stddev_yaw_bias", 0.1);

  // Measurement parameters
  pose_gate_dist_ = this->declare_parameter<double>("pose_gate_dist", 10.0);
  twist_gate_dist_ = this->declare_parameter<double>("twist_gate_dist", 10.0);
  pose_additional_delay_ = this->declare_parameter<double>("pose_additional_delay", 0.0);
  twist_additional_delay_ = this->declare_parameter<double>("twist_additional_delay", 0.0);

  // Diagnostics parameters
  pose_delay_threshold_ = this->declare_parameter<double>("pose_delay_threshold", 0.1);
  twist_delay_threshold_ = this->declare_parameter<double>("twist_delay_threshold", 0.1);

  // Topic names
  pose_topic_ = this->declare_parameter<std::string>("pose_topic", "measured_pose_with_covariance");
  twist_topic_ = this->declare_parameter<std::string>("twist_topic", "measured_twist_with_covariance");
  pose_pub_topic_ = this->declare_parameter<std::string>("pose_pub_topic", "ekf_pose");
  pose_cov_pub_topic_ = this->declare_parameter<std::string>("pose_cov_pub_topic", "ekf_pose_with_covariance");
  odom_pub_topic_ = this->declare_parameter<std::string>("odom_pub_topic", "ekf_odom");
  twist_pub_topic_ = this->declare_parameter<std::string>("twist_pub_topic", "ekf_twist");
  twist_cov_pub_topic_ = this->declare_parameter<std::string>("twist_cov_pub_topic", "ekf_twist_with_covariance");
  diag_pub_topic_ = this->declare_parameter<std::string>("diag_pub_topic", "diagnostics");
}

void EKFLocalizer::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  try {
    // Transform the pose to the map frame if necessary
    geometry_msgs::msg::PoseWithCovarianceStamped transformed_pose;
    if (msg->header.frame_id != "map") {
      RCLCPP_DEBUG(this->get_logger(), "Transforming pose from frame %s to map", msg->header.frame_id.c_str());
      tf_buffer_.transform(*msg, transformed_pose, "map");
    } else {
      transformed_pose = *msg;
    }

    // Add the transformed pose to the queue
    if (pose_queue_.size() < 100) {
      pose_queue_.push(std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(transformed_pose));
      // RCLCPP_INFO(this->get_logger(), "Pose added to queue. Queue size: %zu", pose_queue_.size()); 
    } else {
      RCLCPP_WARN(this->get_logger(), "Pose queue is full. Dropping the oldest measurement.");
      pose_queue_.pop(); // Remove the oldest measurement
      pose_queue_.push(std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(transformed_pose)); // Add the new measurement
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error in poseCallback: %s", e.what());
  }
}

void EKFLocalizer::twistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
  try {
    if (twist_queue_.size() < 100) { // Limit queue size
      twist_queue_.push(msg);
      // RCLCPP_INFO(this->get_logger(), "Twist added to queue. Queue size: %zu", twist_queue_.size());
      RCLCPP_DEBUG(this->get_logger(), "Twist measurement received and added to the queue.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Twist queue is full. Dropping the oldest measurement.");
      twist_queue_.pop(); // Remove the oldest measurement
      twist_queue_.push(msg); // Add the new measurement
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error in twistCallback: %s", e.what());
  }
}

void EKFLocalizer::predict(double dt) {
  try {
    // State transition matrix (A)
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
    A(0, 4) = cos(state_(2)) * dt;
    A(1, 4) = sin(state_(2)) * dt;
    A(2, 5) = dt;
    A(2, 3) = -dt;  // yaw += (wz - yaw_bias) * dt

    // Process noise covariance (Q)
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
    Q(4, 4) = proc_stddev_vx_c_ * proc_stddev_vx_c_ * dt * dt;
    Q(5, 5) = proc_stddev_wz_c_ * proc_stddev_wz_c_ * dt * dt;
    Q(2, 2) = proc_stddev_yaw_c_ * proc_stddev_yaw_c_ * dt * dt;
    Q(3, 3) = proc_stddev_yaw_bias_ * dt; 

    // Predict state and covariance
    state_ = A * state_;
    covariance_ = A * covariance_ * A.transpose() + Q;
    // RCLCPP_INFO(this->get_logger(), "Predicted state - x: %.3f, y: %.3f, yaw: %.3f", state_(0), state_(1), state_(2));

    // Add the predicted state and covariance to the buffers
    delayed_states_.push_back({state_, this->now()});
    covariance_buffer_.push_back(covariance_);

    // Limit the buffer size
    if (delayed_states_.size() > 100) {
      delayed_states_.erase(delayed_states_.begin());
      covariance_buffer_.erase(covariance_buffer_.begin());
    }

    RCLCPP_DEBUG(this->get_logger(), "State prediction completed.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error in predict: %s", e.what());
  }
}

bool EKFLocalizer::updatePose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose, double delay_time) {
  try {
    // Find the closest delayed state
    size_t delay_step = find_closest_delay_time_index(delay_time);
    Eigen::VectorXd delayed_state = delayed_states_[delay_step].first;
    Eigen::MatrixXd delayed_covariance = covariance_buffer_[delay_step];

    // Measurement matrix (H)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H(0, 0) = 1;  // x
    H(1, 1) = 1;  // y
    H(2, 2) = 1;  // yaw

    // Measurement noise covariance (R)
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3, 3);
    R(0, 0) = pose.pose.covariance[0];  // x noise
    R(1, 1) = pose.pose.covariance[7];  // y noise
    R(2, 2) = pose.pose.covariance[35]; // yaw noise

    // Measurement vector (z)
    Eigen::VectorXd z(3);
    z << pose.pose.pose.position.x, pose.pose.pose.position.y, tf2::getYaw(pose.pose.pose.orientation);

    double distance;
    // Check if the measurement is valid
    if (!is_measurement_valid(z, H, delayed_covariance, R, pose_gate_dist_, delayed_state)) {
      RCLCPP_WARN(this->get_logger(), "Pose measurement rejected due to large Mahalanobis distance");
      return false;
    }
    

    // Kalman gain (K)
    Eigen::MatrixXd K = delayed_covariance * H.transpose() * (H * delayed_covariance * H.transpose() + R).inverse();

    // Update state and covariance
    state_ = delayed_state + K * (z - H * delayed_state);
    covariance_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * delayed_covariance;

    RCLCPP_DEBUG(this->get_logger(), "Pose update completed.");
    // RCLCPP_INFO(this->get_logger(), "Updated pose state - x: %.3f, y: %.3f, yaw: %.3f", state_(0), state_(1), state_(2));
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error in updatePose: %s", e.what());
    return false;
  }
}

bool EKFLocalizer::updateTwist(const geometry_msgs::msg::TwistWithCovarianceStamped & twist, double delay_time) {
  try {
    // Find the closest delayed state
    size_t delay_step = find_closest_delay_time_index(delay_time);
    Eigen::VectorXd delayed_state = delayed_states_[delay_step].first;
    Eigen::MatrixXd delayed_covariance = covariance_buffer_[delay_step];

    // Measurement matrix (H)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
    H(0, 4) = 1;  // vx
    H(1, 5) = 1;  // wz

    // Measurement noise covariance (R)
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
    R(0, 0) = twist.twist.covariance[0];  // vx noise
    R(1, 1) = twist.twist.covariance[35]; // wz noise

    // Measurement vector (z)
    Eigen::VectorXd z(2);
    z << twist.twist.twist.linear.x, twist.twist.twist.angular.z;

    // Check if the measurement is valid
    double distance;
    // if (!is_measurement_valid(z, H, delayed_covariance, R, twist_gate_dist_, delayed_state)) {
    //   RCLCPP_WARN(this->get_logger(), "Twist measurement rejected due to large Mahalanobis distance");
      
    //   return false;
    // }
    

    // Kalman gain (K)
    Eigen::MatrixXd K = delayed_covariance * H.transpose() * (H * delayed_covariance * H.transpose() + R).inverse();

    // Update state and covariance
    state_ = delayed_state + K * (z - H * delayed_state);
    covariance_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * delayed_covariance;

    RCLCPP_DEBUG(this->get_logger(), "Twist update completed.");
    // RCLCPP_INFO(this->get_logger(), "Updated twist state - vx: %.3f, wz: %.3f", state_(4), state_(5));
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error in updateTwist: %s", e.what());
    return false;
  }
}

void EKFLocalizer::timerCallback() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // 1. Perform prediction
  predict(1.0 / predict_frequency_);

  // 2. Process measurement queues
  processPoseQueue();
  processTwistQueue();

  // 3. Publish outputs
  publishEstimate();
  broadcastTransform();
  publishDiagnostics();
}

void EKFLocalizer::processPoseQueue() {
  while (!pose_queue_.empty()) {
    auto pose_msg = pose_queue_.front();
    pose_queue_.pop();

    // Calculate measurement delay
    const double delay_time = (this->now() - pose_msg->header.stamp).seconds() + pose_additional_delay_;
    
    if (delay_time > 1.0) {  // Skip stale measurements
      RCLCPP_WARN(this->get_logger(), "Skipping old pose measurement");
      continue;
    }

    updatePose(*pose_msg, delay_time);
  }
}

void EKFLocalizer::processTwistQueue() {
  while (!twist_queue_.empty()) {
    auto twist_msg = twist_queue_.front();
    twist_queue_.pop();

    const double delay_time = (this->now() - twist_msg->header.stamp).seconds() + twist_additional_delay_;
    
    if (delay_time > 1.0) {
      RCLCPP_WARN(this->get_logger(), "Skipping old twist measurement");
      continue;
    }

    updateTwist(*twist_msg, delay_time);
  }
}

bool EKFLocalizer::is_measurement_valid(const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & P, const Eigen::MatrixXd & R, double threshold, const Eigen::VectorXd & delayed_state) {
  double mahalanobis_distance = std::sqrt((z - H * delayed_state).transpose() * (H * P * H.transpose() + R).inverse() * (z - H * delayed_state));
  RCLCPP_INFO(this->get_logger(), "Mahalanobis Distance: %lf", mahalanobis_distance);
  return mahalanobis_distance < threshold;
}

size_t EKFLocalizer::find_closest_delay_time_index(double target_delay) {
  auto lower = std::lower_bound(delayed_states_.begin(), delayed_states_.end(), target_delay,
      [this](const std::pair<Eigen::VectorXd, rclcpp::Time>& state, double delay) {
          return (state.second - this->delayed_states_.front().second).seconds() < delay;
      });
  if (lower == delayed_states_.begin()) return 0;
  if (lower == delayed_states_.end()) return delayed_states_.size() - 1;
  return (target_delay - (lower - 1)->second.seconds()) < (lower->second.seconds() - target_delay) ? 
         (lower - delayed_states_.begin() - 1) : (lower - delayed_states_.begin());
}

void EKFLocalizer::publishEstimate() {
  // Publish the estimated pose
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = "map";  // Set frame_id to "map"
  pose_msg.pose.position.x = state_(0);
  pose_msg.pose.position.y = state_(1);
  pose_msg.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, state_(2));
  pose_msg.pose.orientation = tf2::toMsg(q);
  pose_pub_->publish(pose_msg);
  // RCLCPP_INFO(this->get_logger(), "Publishing estimated pose - x: %.3f, y: %.3f, yaw: %.3f", state_(0), state_(1), state_(2));

  // Publish the estimated twist
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = this->now();
  twist_msg.header.frame_id = "map";  // Set frame_id to "map"
  twist_msg.twist.linear.x = state_(4);
  twist_msg.twist.angular.z = state_(5);
  twist_pub_->publish(twist_msg);
}

void EKFLocalizer::broadcastTransform() {
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "map";  // Parent frame
  transformStamped.child_frame_id = "odom";  // Child frame
  transformStamped.transform.translation.x = state_(0);
  transformStamped.transform.translation.y = state_(1);
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, state_(2));
  transformStamped.transform.rotation = tf2::toMsg(q);

  // Send the transformation
  // tf_broadcaster_->sendTransform(transformStamped);
}

void EKFLocalizer::publishDiagnostics() {
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();
  diag_array.status.push_back(check_measurement_delay("pose", pose_delay_, pose_delay_threshold_));
  diag_array.status.push_back(check_mahalanobis_distance("pose", pose_mahalanobis_distance_, pose_gate_dist_));
  diag_pub_->publish(diag_array);
}

diagnostic_msgs::msg::DiagnosticStatus EKFLocalizer::check_measurement_delay(const std::string& measurement_type, double delay_time, double threshold) {
  diagnostic_msgs::msg::DiagnosticStatus stat;
  stat.level = (delay_time > threshold) ? diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat.message = (delay_time > threshold) ? "[WARN] " + measurement_type + " topic is delayed" : "OK";
  return stat;
}

diagnostic_msgs::msg::DiagnosticStatus EKFLocalizer::check_mahalanobis_distance(const std::string& measurement_type, double distance, double threshold) {
  diagnostic_msgs::msg::DiagnosticStatus stat;
  stat.level = (distance > threshold) ? diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat.message = (distance > threshold) ? "[WARN] Mahalanobis distance of " + measurement_type + " is large" : "OK";
  return stat;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EKFLocalizer>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}