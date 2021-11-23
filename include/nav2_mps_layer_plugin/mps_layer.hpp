#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

namespace nav2_mps_layer_plugin
{

    class MPSLayer : public nav2_costmap_2d::Layer
    {
    public:
        MPSLayer() {}

        virtual ~MPSLayer();

        virtual void activate();

        virtual void deactivate();

        virtual void onInitialize();

        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double *min_x,
                                  double *min_y,
                                  double *max_x,
                                  double *max_y);

        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i, int min_j, int max_i, int max_j);

        void matchSize();

        virtual void reset();

        virtual bool isClearable();

        void incomingPoses(const geometry_msgs::msg::PoseArray::SharedPtr mps_info);

        geometry_msgs::msg::PoseArray mps_poses_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr mps_poses_sub_;
        std::string map_topic_;
        bool mps_poses_received_{false};
    };

} // namespace  nav2_mps_layer_plugin
