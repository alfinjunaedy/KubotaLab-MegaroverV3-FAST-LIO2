#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

class FastLio2PoseNode : public rclcpp::Node
{
public:
    FastLio2PoseNode()
    : Node("fastlio2_pose_node")
    {
        // Subscribe to FAST-LIO2 odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry",
            10,
            std::bind(
                &FastLio2PoseNode::odomCallback,
                this,
                std::placeholders::_1));

        // Publish converted pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
            "/fastlio2_pose",
            10);

        RCLCPP_INFO(
            this->get_logger(),
            "FAST-LIO2 pose converter started.");
    }

private:

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        /*
         * FAST-LIO2 position
         *
         * p_fastlio = [x, y, z]
         */
        Eigen::Vector3d p_fastlio(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        /*
         * Coordinate conversion:
         *
         * FAST-LIO2 frame
         *     ↓
         * robot frame
         *
         * Robot convention:
         *   x+ = forward
         *   y+ = left
         *   z+ = up
         */
        Eigen::Matrix3d R_convert;

        R_convert <<
            -0.939693,  0.0,       0.342020,
             0.0,      -1.0,       0.0,
             0.342020,  0.0,       0.939693;

        /*
         * Convert position
         */
        Eigen::Vector3d p_robot =
            R_convert * p_fastlio;

        /*
         * FAST-LIO2 quaternion
         */
        Eigen::Quaterniond q_fastlio(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);

        Eigen::Matrix3d R_fastlio =
            q_fastlio.toRotationMatrix();

        /*
         * Convert orientation using the same
         * coordinate convention as your ORB-SLAM3 node.
         */
        Eigen::Matrix3d R_robot =
            R_convert *
            R_fastlio *
            R_convert.transpose();

        Eigen::Quaterniond q_robot(R_robot);

        /*
         * Publish geometry_msgs/Pose
         */
        geometry_msgs::msg::Pose pose_msg;

        pose_msg.position.x = p_robot.x();
        pose_msg.position.y = p_robot.y();
        pose_msg.position.z = p_robot.z();

        pose_msg.orientation.x = q_robot.x();
        pose_msg.orientation.y = q_robot.y();
        pose_msg.orientation.z = q_robot.z();
        pose_msg.orientation.w = q_robot.w();

        pose_pub_->publish(pose_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FastLio2PoseNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}