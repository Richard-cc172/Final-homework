#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <yaml-cpp/yaml.h>

std::string speed_mode = "local"; // 默认底盘坐标系模式

// 新增发布器，用于发布处理后的速度指令
ros::Publisher transformed_cmd_vel_pub;

// 发布速度到 /joint_states
ros::Publisher joint_states_pub;

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    geometry_msgs::Twist transformed_vel = *msg;

    // 如果模式为全局坐标系，需要通过 TF 转换速度
    if (speed_mode == "global") {
        static tf::TransformListener tf_listener;
        try {
            geometry_msgs::Vector3Stamped vel_global, vel_local;
            vel_global.header.frame_id = "odom"; // 全局坐标系速度
            vel_global.vector.x = msg->linear.x;
            vel_global.vector.y = msg->linear.y;
            vel_global.vector.z = 0.0;

            // 转换到 base_link 坐标系
            tf_listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));
            tf_listener.transformVector("base_link", vel_global, vel_local);

            transformed_vel.linear.x = vel_local.vector.x;
            transformed_vel.linear.y = vel_local.vector.y;
        } catch (tf::TransformException& ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
            return;
        }
    }

  // 发布处理后的速度
  transformed_cmd_vel_pub.publish(transformed_vel);

    // 转换后的速度用于逆运动学计算四个轮子的期望速度
    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = ros::Time::now();
    joint_states.velocity.resize(4);

    double wheel_radius = 0.07625; // 轮子半径
    double wheel_base = 0.4;       // 中心到轮子距离

    double v_x = transformed_vel.linear.x;
    double v_y = transformed_vel.linear.y;
    double v_theta = transformed_vel.angular.z;

    // 计算麦克纳姆轮速度
    joint_states.velocity[0] = (v_x - v_y - v_theta * wheel_base) / wheel_radius; // 前左轮
    joint_states.velocity[1] = (v_x + v_y + v_theta * wheel_base) / wheel_radius; // 前右轮
    joint_states.velocity[2] = (v_x + v_y - v_theta * wheel_base) / wheel_radius; // 后左轮
    joint_states.velocity[3] = (v_x - v_y + v_theta * wheel_base) / wheel_radius; // 后右轮

    joint_states_pub.publish(joint_states);
    
  ROS_INFO("Global vel: linear.x=%f, linear.y=%f, angular.z=%f",
           msg->linear.x, msg->linear.y, msg->angular.z);
  ROS_INFO("Local vel: linear.x=%f, linear.y=%f, angular.z=%f",
           transformed_vel.linear.x, transformed_vel.linear.y, transformed_vel.angular.z);
  ROS_INFO("Wheel speeds: FL=%.2f, FR=%.2f, BL=%.2f, BR=%.2f", joint_states.velocity[0], joint_states.velocity[1], joint_states.velocity[2], joint_states.velocity[3]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_transform_node");
    ros::NodeHandle nh;

    // 加载配置文件
    std::string config_file;
    nh.getParam("config_file", config_file); // 获取配置文件路径
    YAML::Node config = YAML::LoadFile(config_file);
    speed_mode = config["speed_mode"].as<std::string>();

    // 订阅 /cmd_vel
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);

    // 新增发布器初始化
    transformed_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/transformed_cmd_vel", 10);

    // 发布 /joint_states
    joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::spin();
    return 0;
}
