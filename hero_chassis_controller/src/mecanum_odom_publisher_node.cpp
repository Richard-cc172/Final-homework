#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// 发布器全局声明
ros::Publisher odom_pub;

// 全局参数
double wheel_radius = 0.07625; // 轮子半径 (m)
double wheel_base = 0.4;       // 中心到轮子的距离 (m)
double x = 0.0, y = 0.0, theta = 0.0; // 机器人位姿
ros::Time last_time;

// JointState 回调函数
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->velocity.size() < 4) {
    ROS_WARN("Not enough velocity data for Mecanum wheels. Skipping this callback.");
    return;
    }


    // 四个轮子的线速度 (m/s)
    double v_fl = msg->velocity[0] * wheel_radius;  // 前左轮
    double v_fr = msg->velocity[1] * wheel_radius;  // 前右轮
    double v_bl = msg->velocity[2] * wheel_radius;  // 后左轮
    double v_br = msg->velocity[3] * wheel_radius;  // 后右轮

    // 当前时间和时间差
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    if (dt <= 0) {
    ROS_WARN("Non-positive time interval detected, skipping this iteration.");
    return;
    }
    last_time = current_time;

    // 计算机器人速度
    double vx = (v_fl + v_fr + v_bl + v_br) / 4.0;
    double vy = (-v_fl + v_fr + v_bl - v_br) / 4.0;
    double vtheta = (-v_fl + v_fr - v_bl + v_br) / (4.0 * wheel_base);

    // 添加调试日志，打印轮子的速度和机器人计算得到的速度
    ROS_INFO_STREAM("Wheel speeds (m/s): front_left=" << v_fl
                  << ", front_right=" << v_fr
                  << ", rear_left=" << v_bl
                  << ", rear_right=" << v_br);
    ROS_INFO_STREAM("Calculated robot velocities (m/s): vx=" << vx
                  << ", vy=" << vy
                  << ", angular velocity (rad/s)=" << vtheta);

    // 更新机器人位姿
    x += (vx * cos(theta) - vy * sin(theta)) * dt;
    y += (vx * sin(theta) + vy * cos(theta)) * dt;
    theta += vtheta * dt;
    theta = fmod(theta + M_PI, 2 * M_PI) - M_PI; // 将 theta 限制在 [-pi, pi] 范围内

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

    // 发布里程计数据
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

    // 使用全局的 odom_pub 发布
    odom_pub.publish(odom);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mecanum_odom_publisher_node");
    ros::NodeHandle nh;

    // 设置日志级别为 DEBUG，便于调试
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
    }

    // 从参数服务器获取机器人参数并检查是否成功
    if (!nh.getParam("chassis_params/wheel_radius", wheel_radius)) {
    ROS_ERROR("Failed to get chassis_params/wheel_radius parameter!");
    return -1;
    }
    if (!nh.getParam("chassis_params/wheel_base", wheel_base)) {
    ROS_ERROR("Failed to get chassis_params/wheel_base parameter!");
    return -1;
    }



    // 初始化时间
    last_time = ros::Time::now();

    // 初始化发布器
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

    // 订阅 JointState 消息
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 10, jointStatesCallback);

    // ROS 回调循环
    ros::spin();
    return 0;
}
