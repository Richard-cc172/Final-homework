#include <ros/ros.h>
#include "hero_chassis_controller/hero_chassis_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
 // 初始化 wheel_controllers_
    wheel_controllers_.resize(4);  // 确保有4个PID控制器
    // 获取关节句柄
    try {
    left_front_joint_ = hw->getHandle("left_front_wheel_joint");
    ROS_INFO("Successfully got handle for left_front_wheel_joint");
    right_front_joint_ = hw->getHandle("right_front_wheel_joint");
    ROS_INFO("Successfully got handle for right_front_wheel_joint");
    left_back_joint_ = hw->getHandle("left_back_wheel_joint");
    ROS_INFO("Successfully got handle for left_back_wheel_joint");
    right_back_joint_ = hw->getHandle("right_back_wheel_joint");
    ROS_INFO("Successfully got handle for right_back_wheel_joint");
} catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR("Failed to get joint handle: %s", e.what());
    return false;
}

    // 加载底盘参数
    if (!root_nh.getParam("chassis_params/wheel_radius", wheel_radius_)) {
    ROS_ERROR("Failed to load chassis parameter: wheel_radius.");
    return false;
}
if (!root_nh.getParam("chassis_params/wheel_base", wheel_base_)) {
    ROS_ERROR("Failed to load chassis parameter: wheel_base.");
    return false;
}
if (!root_nh.getParam("chassis_params/wheel_track", wheel_track_)) {
    ROS_ERROR("Failed to load chassis parameter: wheel_track.");
    return false;
}


    // 初始化 PID 参数
    double p, i, d, i_clamp;
    controller_nh.param("pid/p", p, 1.0);
    controller_nh.param("pid/i", i, 0.1);
    controller_nh.param("pid/d", d, 0.01);
    controller_nh.param("pid/i_clamp", i_clamp, 1.0);

    for (auto& pid : wheel_controllers_) {
        pid.initPid(p, i, d, i_clamp, -i_clamp);
    }

    // 设置动态参数服务器
    dynamic_reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<hero_chassis_controller::PIDConfig>>(controller_nh);
    if (!dynamic_reconfigure_server_) {
    ROS_ERROR("Failed to initialize dynamic reconfigure server.");
    return false;
    }

    dynamic_reconfigure::Server<hero_chassis_controller::PIDConfig>::CallbackType cb = boost::bind(&HeroChassisController::dynamicReconfigureCallback, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(cb);

    // 订阅 /cmd_vel 和 /joint_states 话题
    //将订阅的话题从 /cmd_vel 改为 /transformed_cmd_vel
    cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("/transformed_cmd_vel", 10, &HeroChassisController::cmdVelCallback, this);

    joint_states_sub_ = root_nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &HeroChassisController::jointStatesCallback, this);

    ROS_INFO("HeroChassisController initialized successfully.");
    return true;
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
    // 计算目标速度
    double vx = current_cmd_.linear.x;
    double vy = current_cmd_.linear.y;
    double omega = current_cmd_.angular.z;

    std::vector<double> target_velocities(4);
    target_velocities[0] = (vx - vy - omega * (wheel_base_ + wheel_track_)) / wheel_radius_; // 前左轮
    target_velocities[1] = (vx + vy + omega * (wheel_base_ + wheel_track_)) / wheel_radius_; // 前右轮
    target_velocities[2] = (vx + vy - omega * (wheel_base_ + wheel_track_)) / wheel_radius_; // 后左轮
    target_velocities[3] = (vx - vy + omega * (wheel_base_ + wheel_track_)) / wheel_radius_; // 后右轮

    // 用实际速度修正 Effort
    double efforts[4];
    for (size_t i = 0; i < 4; ++i) {
        double error = target_velocities[i] - current_velocities_[i];
        efforts[i] = wheel_controllers_[i].computeCommand(error, period);
    }

    // 设置 Effort
    left_front_joint_.setCommand(efforts[0]);
    right_front_joint_.setCommand(efforts[1]);
    left_back_joint_.setCommand(efforts[2]);
    right_back_joint_.setCommand(efforts[3]);

    ROS_INFO_THROTTLE(1, "Efforts set: [%f, %f, %f, %f]", efforts[0], efforts[1], efforts[2], efforts[3]);
}

void HeroChassisController::dynamicReconfigureCallback(hero_chassis_controller::PIDConfig& config, uint32_t level) {
    for (auto& pid : wheel_controllers_) {
        pid.setGains(config.p, config.i, config.d, config.i_clamp, -config.i_clamp);
    }
    ROS_INFO("Dynamic reconfigure updated: P=%.2f, I=%.2f, D=%.2f, I_clamp=%.2f",
             config.p, config.i, config.d, config.i_clamp);
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
     if (!msg) {
        ROS_WARN("Received null message in cmdVelCallback.");
        return;
    }
    current_cmd_ = *msg;
}

void HeroChassisController::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
   if (!msg) {
        ROS_WARN("Received null message in jointStatesCallback.");
        return;
    }
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "left_front_wheel_joint") {
            current_velocities_[0] = msg->velocity[i];
        } else if (msg->name[i] == "right_front_wheel_joint") {
            current_velocities_[1] = msg->velocity[i];
        } else if (msg->name[i] == "left_back_wheel_joint") {
            current_velocities_[2] = msg->velocity[i];
        } else if (msg->name[i] == "right_back_wheel_joint") {
            current_velocities_[3] = msg->velocity[i];
        }
    }
}

}  // namespace hero_chassis_controller

// 注册控制器插件
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
