#include "hero_chassis_controller/chassis_controller.hpp"

ChassisController::ChassisController(ros::NodeHandle& nh, const std::string& yaml_file)
    : nh_(nh) {
    loadParameters(yaml_file);

    // 订阅 /cmd_vel
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &ChassisController::cmdVelCallback, this);
    // 发布期望速度 /expected_speeds
    expected_speed_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/expected_speeds", 10);
}

void ChassisController::loadParameters(const std::string& yaml_file) {
    YAML::Node config = YAML::LoadFile(yaml_file);
    wheel_radius_ = config["chassis_params"]["wheel_radius"].as<double>();
    wheel_base_ = config["chassis_params"]["wheel_base"].as<double>();
    wheel_track_ = config["chassis_params"]["wheel_track"].as<double>();

  // 检查参数是否有效
  if (wheel_radius_ <= 0 || wheel_base_ <= 0 || wheel_track_ <= 0) {
    ROS_ERROR("Invalid chassis parameters. Please check the YAML file.");
    ros::shutdown();
  }
}


void ChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
 ROS_INFO("cmdVelCallback triggered!");
    ROS_INFO_STREAM("Received /cmd_vel: linear.x=" << msg->linear.x
                    << ", linear.y=" << msg->linear.y
                    << ", angular.z=" << msg->angular.z);
  // 检查 /cmd_vel 数据是否异常
  if (std::abs(msg->linear.x) > 10.0 || std::abs(msg->linear.y) > 10.0 || std::abs(msg->angular.z) > 10.0) {
    ROS_WARN("Unrealistic /cmd_vel input detected. Please verify input.");
  }

    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double omega = msg->angular.z;

    calculateExpectedSpeeds(vx, vy, omega);
}

void ChassisController::calculateExpectedSpeeds(double vx, double vy, double omega) {
  double L = wheel_base_ / 2.0;
  double W = wheel_track_ / 2.0;

  // 计算轮子的期望速度
  float speed_front_left = static_cast<float>((1.0 / wheel_radius_) * (vx - vy - (L + W) * omega));
  float speed_front_right = static_cast<float>((1.0 / wheel_radius_) * (vx + vy + (L + W) * omega));
  float speed_rear_left = static_cast<float>((1.0 / wheel_radius_) * (vx + vy - (L + W) * omega));
  float speed_rear_right = static_cast<float>((1.0 / wheel_radius_) * (vx - vy + (L + W) * omega));

  // 打印调试日志
  ROS_INFO_STREAM("Calculated expected speeds: ["
                  << "FL: " << speed_front_left << ", "
                  << "FR: " << speed_front_right << ", "
                  << "RL: " << speed_rear_left << ", "
                  << "RR: " << speed_rear_right << "]");

  // 发布期望速度
  std_msgs::Float32MultiArray expected_speeds;
  expected_speeds.data = {speed_front_left, speed_front_right, speed_rear_left, speed_rear_right};

  expected_speed_pub_.publish(expected_speeds);
}

