#include <memory>
#include <string>
#include <limits>
#include <vector>
#include "graceful_controller_ros/odom_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

    OdomGoalChecker::OdomGoalChecker()
        : xy_goal_tolerance_(0.25),
          yaw_goal_tolerance_(0.25),
          stateful_(true),
          check_xy_(true),
          xy_goal_tolerance_sq_(0.0625)
    {
    }

    void OdomGoalChecker::initialize(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        const std::string &plugin_name,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
    {
        plugin_name_ = plugin_name;
        auto node = parent.lock();

        nav2_util::declare_parameter_if_not_declared(
            node,
            plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
        nav2_util::declare_parameter_if_not_declared(
            node,
            plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
        nav2_util::declare_parameter_if_not_declared(
            node,
            plugin_name + ".stateful", rclcpp::ParameterValue(true));

        node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
        node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
        node->get_parameter(plugin_name + ".stateful", stateful_);

        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

        // Add callback for dynamic parameters
        dyn_params_handler_ = node->add_on_set_parameters_callback(
            std::bind(&OdomGoalChecker::dynamicParametersCallback, this, _1));
    }

    void OdomGoalChecker::reset()
    {
        check_xy_ = true;
    }

    bool OdomGoalChecker::isGoalReached(
        const geometry_msgs::msg::Pose &query_pose, const geometry_msgs::msg::Pose &goal_pose,
        const geometry_msgs::msg::Twist &)
    {

        // Transform the robot's pose from the global frame to the odom frame
        geometry_msgs::msg::PoseStamped odom_robot_pose;
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = buffer_->lookupTransform("odom", robot_pose.header.frame_id, tf2::TimePointZero);

            tf2::doTransform(robot_pose, odom_robot_pose, transformStamped);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(LOGGER, "Failed to transform robot's pose to odom frame: %s", ex.what());
            return false;
        }

        if (check_xy_)
        {
            double dx = query_pose.position.x - goal_pose.position.x,
                   dy = query_pose.position.y - goal_pose.position.y;
            if (dx * dx + dy * dy > xy_goal_tolerance_sq_)
            {
                return false;
            }
            // We are within the window
            // If we are stateful, change the state.
            if (stateful_)
            {
                check_xy_ = false;
            }
        }
        double dyaw = angles::shortest_angular_distance(
            tf2::getYaw(query_pose.orientation),
            tf2::getYaw(goal_pose.orientation));
        return fabs(dyaw) < yaw_goal_tolerance_;
    }

    bool OdomGoalChecker::getTolerances(
        geometry_msgs::msg::Pose &pose_tolerance,
        geometry_msgs::msg::Twist &vel_tolerance)
    {
        double invalid_field = std::numeric_limits<double>::lowest();

        pose_tolerance.position.x = xy_goal_tolerance_;
        pose_tolerance.position.y = xy_goal_tolerance_;
        pose_tolerance.position.z = invalid_field;
        pose_tolerance.orientation =
            nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

        vel_tolerance.linear.x = invalid_field;
        vel_tolerance.linear.y = invalid_field;
        vel_tolerance.linear.z = invalid_field;

        vel_tolerance.angular.x = invalid_field;
        vel_tolerance.angular.y = invalid_field;
        vel_tolerance.angular.z = invalid_field;

        return true;
    }

    rcl_interfaces::msg::SetParametersResult
    OdomGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        for (auto &parameter : parameters)
        {
            const auto &type = parameter.get_type();
            const auto &name = parameter.get_name();

            if (type == ParameterType::PARAMETER_DOUBLE)
            {
                if (name == plugin_name_ + ".xy_goal_tolerance")
                {
                    xy_goal_tolerance_ = parameter.as_double();
                    xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
                }
                else if (name == plugin_name_ + ".yaw_goal_tolerance")
                {
                    yaw_goal_tolerance_ = parameter.as_double();
                }
            }
            else if (type == ParameterType::PARAMETER_BOOL)
            {
                if (name == plugin_name_ + ".stateful")
                {
                    stateful_ = parameter.as_bool();
                }
            }
        }
        result.successful = true;
        return result;
    }

} // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::OdomGoalChecker, nav2_core::GoalChecker)
