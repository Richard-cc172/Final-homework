#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <dynamic_reconfigure/server.h>
#include <hero_chassis_controller/PIDConfig.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>


namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  // 动态参数回调函数
  void dynamicReconfigureCallback(hero_chassis_controller::PIDConfig& config, uint32_t level);

  // 回调函数
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

  // PID 控制器
  std::vector<control_toolbox::Pid> wheel_controllers_{4};

  // 底盘参数
  double wheel_radius_, wheel_base_, wheel_track_;

  // 关节句柄
  hardware_interface::JointHandle left_front_joint_, right_front_joint_, left_back_joint_, right_back_joint_;

  // 当前速度
  std::vector<double> current_velocities_{4, 0.0};

  // 最新的 /cmd_vel 指令
  geometry_msgs::Twist current_cmd_;

  // 订阅器和动态参数服务器
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber joint_states_sub_;
  std::shared_ptr<dynamic_reconfigure::Server<hero_chassis_controller::PIDConfig>> dynamic_reconfigure_server_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_H
