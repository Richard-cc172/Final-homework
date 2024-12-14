#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>
#include <algorithm>

// 全局参数
double wheel_radius = 0.07625;
double wheel_base = 0.4;
double wheel_track = 0.4;
std::string speed_mode = "local"; // 默认速度模式
const double deadband_threshold = 0.01; // 速度死区
const double max_wheel_speed = 10.0;    // 轮子最大允许速度 (rad/s)

// 发布器
ros::Publisher odom_pub, expected_speed_pub, transformed_cmd_vel_pub, joint_states_pub;

// 位姿变量
double x = 0.0, y = 0.0, theta = 0.0;
ros::Time last_time;

// 时间戳用于健康检查
ros::Time last_cmd_vel_time;
ros::Time last_joint_state_time;

// 工具函数：处理死区逻辑
inline double applyDeadband(double value, double threshold) {
    return (std::fabs(value) < threshold) ? 0.0 : value;
}

// 工具函数：归一化轮速逻辑
void normalizeWheelSpeeds(double fl, double fr, double rl, double rr, double max_speed) {
    double max_calculated_speed = std::max({std::fabs(fl), std::fabs(fr), std::fabs(rl), std::fabs(rr)});
    if (max_calculated_speed > max_speed) {
        double scaling_factor = max_speed / max_calculated_speed;
        fl *= scaling_factor;
        fr *= scaling_factor;
        rl *= scaling_factor;
        rr *= scaling_factor;
    }
}


// 健康检查函数
void healthCheck() {
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_cmd_vel_time).toSec() > 2.0) {
    ROS_WARN_THROTTLE(2.0, "/cmd_vel not received for over 2 seconds");
	}
	if ((current_time - last_joint_state_time).toSec() > 2.0) {
 	   ROS_WARN_THROTTLE(2.0, "/joint_states not received for over 2 seconds");
	}
}

// 里程计更新函数
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    last_joint_state_time = ros::Time::now(); // 更新时间戳

    if (msg->velocity.size() < 4) {
        ROS_WARN_THROTTLE(1.0, "Not enough velocity data for Mecanum wheels. Skipping this callback.");
        return;
    }

    // 四轮线速度
    double v_fl = msg->velocity[0] * wheel_radius;  // 前左轮
    double v_fr = msg->velocity[1] * wheel_radius;  // 前右轮
    double v_bl = msg->velocity[2] * wheel_radius;  // 后左轮
    double v_br = msg->velocity[3] * wheel_radius;  // 后右轮

    // 时间计算
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    if (dt <= 0) return;
    last_time = current_time;

// 机器人速度
double vx = (v_bl + v_br) / 2.0;
double vy = (v_bl - v_fl) / 2.0;
double vtheta = (v_fr - v_bl) / (wheel_base + wheel_track);


    // 应用速度死区
    vx = applyDeadband(vx, deadband_threshold);
    vy = applyDeadband(vy, deadband_threshold);
    vtheta = applyDeadband(vtheta, deadband_threshold);

    // 更新位姿
    x += (vx * cos(theta) - vy * sin(theta)) * dt;
    y += (vx * sin(theta) + vy * cos(theta)) * dt;
    theta += vtheta * dt;
    theta = fmod(theta + M_PI, 2 * M_PI) - M_PI;

    // 发布 TF
    static tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
    odom_broadcaster.sendTransform(odom_trans);

    // 发布里程计
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vtheta;

    odom_pub.publish(odom);

    // 调试信息
    ROS_INFO_STREAM_THROTTLE(1.0, "Odometry - x: " << x << ", y: " << y << ", theta: " << theta);
}

// cmd_vel回调函数
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    last_cmd_vel_time = ros::Time::now(); // 更新时间戳

    geometry_msgs::Twist transformed_vel = *msg;

    // 全局坐标模式下的速度转换
    if (speed_mode == "global") {
        static tf::TransformListener tf_listener;
        try {
            geometry_msgs::Vector3Stamped vel_global, vel_local;
            vel_global.header.frame_id = "odom"; // 全局速度
            vel_global.vector.x = msg->linear.x;
            vel_global.vector.y = msg->linear.y;
            vel_global.vector.z = 0.0;

            // 转换到 base_link 坐标系
            tf_listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));
            tf_listener.transformVector("base_link", vel_global, vel_local);

            // 更新速度
            transformed_vel.linear.x = vel_local.vector.x;
            transformed_vel.linear.y = vel_local.vector.y;
            transformed_vel.angular.z = msg->angular.z;
        } catch (tf::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "TF transform failed: %s", ex.what());
            return;
        }
    }

    // 发布转换后的速度
    transformed_cmd_vel_pub.publish(transformed_vel);

    // 调试信息
    ROS_INFO_THROTTLE(1.0, "Transformed velocity - Linear: x=%.3f, y=%.3f, Angular: z=%.3f",
                      transformed_vel.linear.x, transformed_vel.linear.y, transformed_vel.angular.z);

    // 期望轮速计算
    double vx = transformed_vel.linear.x;
    double vy = transformed_vel.linear.y;
    double omega = transformed_vel.angular.z;

    double L = wheel_base / 2.0;
    double W = wheel_track / 2.0;

    float speed_fl = (1.0 / wheel_radius) * (vx - vy - (L + W) * omega);
    float speed_fr = (1.0 / wheel_radius) * (vx + vy + (L + W) * omega);
    float speed_rl = (1.0 / wheel_radius) * (vx + vy - (L + W) * omega);
    float speed_rr = (1.0 / wheel_radius) * (vx - vy + (L + W) * omega);

    // 对轮速归一化
    normalizeWheelSpeeds(speed_fl, speed_fr, speed_rl, speed_rr, max_wheel_speed);

    // 发布期望轮速
    std_msgs::Float32MultiArray expected_speeds;
    expected_speeds.data = {speed_fl, speed_fr, speed_rl, speed_rr};
    expected_speed_pub.publish(expected_speeds);


    // 调试信息
    ROS_INFO_STREAM_THROTTLE(1.0, "Wheel speeds (rad/s) - FL: " << speed_fl
                                  << ", FR: " << speed_fr << ", RL: " << speed_rl << ", RR: " << speed_rr);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_transform_node");
    ros::NodeHandle nh;

     // 初始化时间变量
    last_time = ros::Time::now();
    last_cmd_vel_time = ros::Time::now();
    last_joint_state_time = ros::Time::now();

    // 加载参数
    nh.param<double>("chassis_params/wheel_radius", wheel_radius, 0.07625);
    nh.param<double>("chassis_params/wheel_base", wheel_base, 0.4);
    nh.param<double>("chassis_params/wheel_track", wheel_track, 0.4);
    nh.param<std::string>("speed_mode", speed_mode, "local");

     // 打印 speed_mode 参数
    ROS_INFO("Loaded speed_mode: %s", speed_mode.c_str());

    // 初始化发布器
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    expected_speed_pub = nh.advertise<std_msgs::Float32MultiArray>("/expected_speeds", 10);
    transformed_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/transformed_cmd_vel", 10);
    //joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // 初始化订阅器
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 10, jointStatesCallback);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);

    last_time = ros::Time::now();

    // 使用多线程 Spinner
    ros::AsyncSpinner spinner(4); // 使用 4 个线程
    spinner.start();

    while (ros::ok()) {
        healthCheck(); // 健康检查
    }

    spinner.stop();
    return 0;
}
