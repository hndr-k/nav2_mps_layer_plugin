#include "nav2_mps_layer_plugin/mps_layer.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace nav2_mps_layer_plugin {

MPSLayer::~MPSLayer() {}
void MPSLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  mps_poses_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "/off_field/mps_infos", 1,
      std::bind(&MPSLayer::incomingPoses, this, std::placeholders::_1));
}
void MPSLayer::incomingPoses(
    const geometry_msgs::msg::PoseArray::SharedPtr mps_info) {
  RCLCPP_INFO(rclcpp::get_logger("mpslayer"), "Poses received");
  mps_poses_received_ = true;
  mps_poses_.header = mps_info->header;
  mps_poses_.poses = mps_info->poses;
}
void MPSLayer::updateBounds(double /*robot_x*/, double /*robot_y*/,
                            double /*robot_yaw*/, double *min_x, double *min_y,
                            double *max_x, double *max_y) {}

void MPSLayer::activate() {}

void MPSLayer::deactivate() {}

void MPSLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i,
                           int min_j, int max_i, int max_j) {
  RCLCPP_INFO(rclcpp::get_logger("mpslayer"), "Pose added");

  /*geometry_msgs::msg::PoseArray message;
  geometry_msgs::msg::Pose pose2;

  double x = 9.5;
  for (int i = 0; i < 4; i++) {
    pose2.position.x = x + 0.5;
    pose2.position.y = 5.0;
    pose2.position.z = 0.0;
    pose2.orientation.x = 0.0;
    pose2.orientation.y = 0.0;
    pose2.orientation.z = 0.0;
    pose2.orientation.w = 0.0;
    message.poses.push_back(pose2);
  }*/
  for (auto &pose : message.poses) {
    master_grid.setCost(pose.position.x, pose.position.y, LETHAL_OBSTACLE);
    RCLCPP_INFO(rclcpp::get_logger("mpslayer"), "Pose added");
  }
}

void MPSLayer::matchSize() {}

void MPSLayer::reset() {}

bool MPSLayer::isClearable() {}

} // namespace nav2_mps_layer_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mps_layer_plugin::MPSLayer, nav2_costmap_2d::Layer)
