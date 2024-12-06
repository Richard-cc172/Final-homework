#include "hero_chassis_controller/chassis_controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "chassis_controller_node");
  ros::NodeHandle nh;

  // 获取 YAML 文件路径
  std::string yaml_file;
  nh.param<std::string>("chassis_param_file", yaml_file, "config/chassis_params.yaml");

  // 启动控制器
  ChassisController controller(nh, yaml_file);

  ros::spin();
  return 0;
}
