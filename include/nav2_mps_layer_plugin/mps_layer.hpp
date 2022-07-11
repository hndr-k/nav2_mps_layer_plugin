#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_mps_layer_plugin {

class MPSLayer : public nav2_costmap_2d::Layer {
public:
  MPSLayer()
      : last_min_x_(std::numeric_limits<double>::lowest()),
        last_min_y_(std::numeric_limits<double>::lowest()),
        last_max_x_(std::numeric_limits<double>::max()),
        last_max_y_(std::numeric_limits<double>::max()) {}

  virtual ~MPSLayer();

  virtual void activate();

  virtual void deactivate();

  virtual void onInitialize();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double *min_x, double *min_y, double *max_x,
                            double *max_y);

  virtual void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i,
                           int min_j, int max_i, int max_j);

  void matchSize();

  virtual void reset();

  virtual bool isClearable();

  void incomingPoses(const geometry_msgs::msg::PoseArray::SharedPtr mps_info);

  geometry_msgs::msg::PoseArray mps_poses_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr mps_poses_sub_;
  std::string mps_topic_;
  double tranform_tolerance_;
  bool mps_poses_received_{false};
  bool test_bounds_{false};
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
};

} // namespace  nav2_mps_layer_plugin
