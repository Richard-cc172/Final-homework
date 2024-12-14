#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <map>
#include <iostream>

// 工具函数：设置终端模式为非阻塞
char getKey() {
    char key = 0;
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    read(STDIN_FILENO, &key, 1);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return key;
}

// 打印控制提示信息
void printInstructions() {
    std::cout << "Control your robot using the keyboard:" << std::endl;
    std::cout << "--------------------------------------" << std::endl;
    std::cout << "Move forward:        w" << std::endl;
    std::cout << "Move backward:       s" << std::endl;
    std::cout << "Move left:           a" << std::endl;
    std::cout << "Move right:          d" << std::endl;
    std::cout << "Rotate counterclockwise:  z" << std::endl;
    std::cout << "Rotate clockwise:         c" << std::endl;
    std::cout << "Stop:                any other key" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_control_node");
    ros::NodeHandle nh;

    // 参数加载
    double linear_speed, angular_speed;
    nh.param("linear_speed", linear_speed, 0.5);   // 默认线速度
    nh.param("angular_speed", angular_speed, 0.5); // 默认角速度

    ROS_INFO("Loaded linear_speed: %.2f, angular_speed: %.2f", linear_speed, angular_speed);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    geometry_msgs::Twist twist;
    char key;
    printInstructions();

    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        key = getKey(); // 获取键盘输入

        // 根据输入设置速度
        switch (key) {
            case 'w': // 前进
                twist.linear.x = linear_speed;
                twist.linear.y = 0.0;
                twist.angular.z = 0.0;
                break;
            case 's': // 后退
                twist.linear.x = -linear_speed;
                twist.linear.y = 0.0;
                twist.angular.z = 0.0;
                break;
            case 'a': // 左移
                twist.linear.x = 0.0;
                twist.linear.y = linear_speed;
                twist.angular.z = 0.0;
                break;
            case 'd': // 右移
                twist.linear.x = 0.0;
                twist.linear.y = -linear_speed;
                twist.angular.z = 0.0;
                break;
            case 'z': // 逆时针旋转
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.angular.z = angular_speed;
                break;
            case 'c': // 顺时针旋转
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.angular.z = -angular_speed;
                break;
            default: // 停止
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.angular.z = 0.0;
                break;
        }

        // 发布速度指令
        cmd_vel_pub.publish(twist);

        // 输出调试信息
        ROS_INFO("Published cmd_vel: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
                 twist.linear.x, twist.linear.y, twist.angular.z);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
