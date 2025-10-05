#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

class Robot_Chase : public rclcpp::Node {
public:
  Robot_Chase() : Node("robot_chase") {

    // For the TF Listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher
    rick_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 10);

    // Timer for control loop  (10Hz)
    control_timer_ = this->create_wall_timer(
        100ms, std::bind(&Robot_Chase::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Robot Chase Node initialized and ready!");
  }

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rick_vel_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  const std::string rick_link_ = "rick/base_link";
  const std::string morty_link_ = "morty/base_link";
  const double stop_distance_ =
      0.356 * 1.01; // robot dimater * 1.01 for the error
  const double kp_distance_ = 0.5;
  const double kp_yaw_ = 1.0;

  void control_loop() {
    geometry_msgs::msg::TransformStamped transform;
    try {
      // Look up for the transformation between rick and morty frames
      transform =
          tf_buffer_->lookupTransform(rick_link_, morty_link_, rclcpp::Time(0),
                                      rclcpp::Duration::from_seconds(0.1));
      //   RCLCPP_INFO(this->get_logger(), "Translation: [%.3f, %.3f, %.3f]",
      //               transform.transform.translation.x,
      //               transform.transform.translation.y,
      //               transform.transform.translation.z);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Couldn't transform %s to %s: %s",
                  rick_link_.c_str(), morty_link_.c_str(), ex.what());
      return;
    }

    const double dx = transform.transform.translation.x;
    const double dy = transform.transform.translation.y;

    const double error_distance = std::hypot(dx, dy);
    const double error_yaw = normalize_angle(std::atan2(dy, dx));
    RCLCPP_INFO(this->get_logger(), "Distance: %.3f  Yaw: %.3f", error_distance,
                error_yaw);

    geometry_msgs::msg::Twist msg;
    // Stop if close enough (avoid continuous bumping)
    if (error_distance < stop_distance_) {
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      rick_vel_publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Chase complete! Final distance: %.3f",
                  error_distance);
      return;
    }

    msg.angular.z = kp_yaw_ * error_yaw;
    msg.linear.x = kp_distance_ * error_distance;

    rick_vel_publisher_->publish(msg);
  }

  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto robot_chase_node = std::make_shared<Robot_Chase>();
  rclcpp::spin(robot_chase_node);
  rclcpp::shutdown();
  return 0;
}