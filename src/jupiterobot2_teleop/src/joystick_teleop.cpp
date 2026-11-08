#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

// Note: Using standard JointState since dynamixel_msgs/JointState doesn't exist in ROS 2
// You might need to change this if your hardware uses a specific custom interface
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define ARM_SLOW 0.1
#define ARM_FAST 0.3

class Jupiter2Teleop : public rclcpp::Node
{
public:
  Jupiter2Teleop() : Node("jupiter2_teleop_joy")
  {
    // Initialize parameters (ROS 2 style)
    this->declare_parameter("axis_linear", 1);
    this->declare_parameter("axis_angular", 0);
    this->declare_parameter("scale_linear", 0.2);
    this->declare_parameter("scale_angular", 1.0);

    linear_ = this->get_parameter("axis_linear").as_int();
    angular_ = this->get_parameter("axis_angular").as_int();
    l_scale_ = this->get_parameter("scale_linear").as_double();
    a_scale_ = this->get_parameter("scale_angular").as_double();

    // Fixed indices from your ROS 1 code
    half_linear_ = 3; half_angular_ = 2; deadman_axis_ = 5;
    accelerate_button_ = 4; dir_left_right_ = 4; dir_up_down_ = 5;
    x_left_ = 0; b_right_ = 2; y_up_ = 3; a_down_ = 1;
    back_ = 8; start_ = 9; left_trigger_ = 6; right_trigger_ = 7;

    deadman_pressed_ = false;
    accelerate_pressed_ = false;
    zero_twist_published_ = false;

    // Publishers
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    arm1_pub_ = this->create_publisher<std_msgs::msg::Float64>("arm1_joint/command", 10);
    arm2_pub_ = this->create_publisher<std_msgs::msg::Float64>("arm2_joint/command", 10);
    arm3_pub_ = this->create_publisher<std_msgs::msg::Float64>("arm3_joint/command", 10);
    arm4_pub_ = this->create_publisher<std_msgs::msg::Float64>("arm4_joint/command", 10);
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64>("gripper_joint/command", 10);
    head_pub_ = this->create_publisher<std_msgs::msg::Float64>("head_joint/command", 10);
    capture_pub_ = this->create_publisher<std_msgs::msg::String>("capture_command", 10);

    // Subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Jupiter2Teleop::joyCallback, this, _1));
    
    // Timer for publishing (replaces boost thread/timer)
    timer_ = this->create_wall_timer(100ms, std::bind(&Jupiter2Teleop::publish, this));
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    
    geometry_msgs::msg::Twist vel;
    deadman_pressed_ = joy->buttons[deadman_axis_];
    accelerate_pressed_ = joy->buttons[accelerate_button_];

    // Speed Logic
    if (joy->axes[half_linear_] || joy->axes[half_angular_]) {
      vel.linear.x = l_scale_ * 0.6 * joy->axes[half_linear_];
      vel.angular.z = a_scale_ * 0.6 * joy->axes[half_angular_];
    } else if (accelerate_pressed_) {
      vel.linear.x = l_scale_ * 1.5 * joy->axes[linear_];
      vel.angular.z = a_scale_ * 2.0 * joy->axes[angular_];
    } else {
      vel.linear.x = l_scale_ * joy->axes[linear_];
      vel.angular.z = a_scale_ * joy->axes[angular_];
    }

    // Arm Speed Logic
    double current_arm_speed = accelerate_pressed_ ? ARM_FAST : ARM_SLOW;
    
    if (joy->buttons[x_left_]) arm_xb_ = current_arm_speed;
    else if (joy->buttons[b_right_]) arm_xb_ = -current_arm_speed;
    else arm_xb_ = 0.0;

    if (joy->buttons[a_down_]) arm_ya_ = current_arm_speed;
    else if (joy->buttons[y_up_]) arm_ya_ = -current_arm_speed;
    else arm_ya_ = 0.0;

    // Servo Logic
    if (joy->axes[dir_up_down_] > 0) {
      gripper_.data = std::max(-0.4, std::min(0.6, gripper_.data + arm_xb_));
      head_.data = std::max(-0.8, std::min(0.6, head_.data + arm_ya_));
    } else if (joy->axes[dir_up_down_] < 0) {
      arm1_.data = std::max(-2.6, std::min(2.6, arm1_.data + arm_xb_));
      arm3_.data = std::max(-2.5, std::min(2.6, arm3_.data + arm_ya_));
    }

    if (joy->axes[dir_left_right_] > 0) {
      arm2_.data = std::max(-2.1, std::min(2.2, arm2_.data + arm_ya_));
    } else if (joy->axes[dir_left_right_] < 0) {
      arm4_.data = std::max(-1.8, std::min(1.8, arm4_.data + arm_ya_));
    }

    // Presets
    if (joy->buttons[back_]) head_.data = -0.5;
    if (joy->buttons[start_]) head_.data = 0.5;
    if (joy->buttons[left_trigger_]) { arm2_.data = -1.4; arm3_.data = 2.2; arm4_.data = 0.6; }

    last_published_vel_ = vel;
  }

  void publish()
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);

    if (deadman_pressed_) {
      vel_pub_->publish(last_published_vel_);
      zero_twist_published_ = false;
    } else if (!zero_twist_published_) {
      vel_pub_->publish(geometry_msgs::msg::Twist());
      zero_twist_published_ = true;
    }

    arm1_pub_->publish(arm1_);
    arm2_pub_->publish(arm2_);
    arm3_pub_->publish(arm3_);
    arm4_pub_->publish(arm4_);
    gripper_pub_->publish(gripper_);
    head_pub_->publish(head_);
  }

  // Members
  int linear_, angular_, half_linear_, half_angular_, deadman_axis_, accelerate_button_, 
      dir_left_right_, dir_up_down_, x_left_, b_right_, y_up_, a_down_, back_, start_, 
      left_trigger_, right_trigger_;
  double l_scale_, a_scale_;
  double arm_xb_, arm_ya_;

  bool deadman_pressed_, accelerate_pressed_, zero_twist_published_;

  std::mutex publish_mutex_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm1_pub_, arm2_pub_, arm3_pub_, arm4_pub_, gripper_pub_, head_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr capture_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  geometry_msgs::msg::Twist last_published_vel_;
  std_msgs::msg::Float64 arm1_, arm2_, arm3_, arm4_, gripper_, head_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jupiter2Teleop>());
  rclcpp::shutdown();
  return 0;
}
