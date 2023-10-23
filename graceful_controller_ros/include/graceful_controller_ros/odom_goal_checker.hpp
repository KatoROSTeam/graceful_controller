
#ifndef NAV2_CONTROLLER__PLUGINS__ODOM_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__ODOM_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace nav2_controller
{

    /**
     * @class OdomGoalChecker
     * @brief Goal Checker plugin that only checks the position difference
     *
     * This class can be stateful if the stateful parameter is set to true (which it is by default).
     * This means that the goal checker will not check if the xy position matches again once it is found to be true.
     */
    class OdomGoalChecker : public nav2_core::GoalChecker
    {
    public:
        OdomGoalChecker();
        // Standard GoalChecker Interface
        void initialize(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            const std::string &plugin_name,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        void reset() override;
        bool isGoalReached(
            const geometry_msgs::msg::Pose &query_pose, const geometry_msgs::msg::Pose &goal_pose,
            const geometry_msgs::msg::Twist &velocity) override;
        bool getTolerances(
            geometry_msgs::msg::Pose &pose_tolerance,
            geometry_msgs::msg::Twist &vel_tolerance) override;

    protected:
        double xy_goal_tolerance_, yaw_goal_tolerance_;
        bool stateful_, check_xy_;
        std::shared_ptr<tf2_ros::Buffer> buffer_;
        // Cached squared xy_goal_tolerance_
        double xy_goal_tolerance_sq_;
        // Dynamic parameters handler
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
        std::string plugin_name_;

        /**
         * @brief Callback executed when a paramter change is detected
         * @param parameters list of changed parameters
         */
        rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
    };

} // namespace nav2_controller

#endif // NAV2_CONTROLLER__PLUGINS__ODOM_GOAL_CHECKER_HPP_
