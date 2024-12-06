#ifndef CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <yaml-cpp/yaml.h>
#include <string>

class ChassisController {
public:
  ChassisController(ros::NodeHandle& nh, const std::string& yaml_file);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

private:
  void loadParameters(const std::string& yaml_file);
  void calculateExpectedSpeeds(double vx, double vy, double omega);

  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher expected_speed_pub_;

  double wheel_radius_;
  double wheel_base_;
  double wheel_track_;
};

#endif // CHASSIS_CONTROLLER_HPP
