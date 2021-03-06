#include "nav2_mps_layer_plugin/mps_layer.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace nav2_mps_layer_plugin {

MPSLayer::~MPSLayer() {}
void MPSLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  declareParameter("mps_topic", rclcpp::ParameterValue("/off_field/mps"));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  node->get_parameter(name_ + "." + "mps_topic", mps_topic_);
  node->get_parameter(name_ + "." + "transform_tolerance", tranform_tolerance_);
  mps_poses_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
      mps_topic_, 1,
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
                            double *max_x, double *max_y) {

    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
  /*} else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }*/
}

void MPSLayer::activate() {}

void MPSLayer::deactivate() {}

void MPSLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i,
                           int min_j, int max_i, int max_j) {
  RCLCPP_INFO(rclcpp::get_logger("mpslayer"), "Pose added_new");
  unsigned char *master_array = master_grid.getCharMap();
  for (auto &pose : mps_poses_.poses) {
    unsigned int index = master_grid.getIndex(pose.position.x, pose.position.y);

    master_array[index] = LETHAL_OBSTACLE;
    RCLCPP_INFO(rclcpp::get_logger("mpslayer"), "Pose added");
  }
}

void MPSLayer::matchSize() {}

void MPSLayer::reset() {}

bool MPSLayer::isClearable() {}

} // namespace nav2_mps_layer_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mps_layer_plugin::MPSLayer, nav2_costmap_2d::Layer)
