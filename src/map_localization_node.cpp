#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class MapLocalizationNode : public rclcpp::Node
{
public:
    MapLocalizationNode() : Node("map_localization_node"), x_(0.0), y_(0.0), theta_(0.0) 
    {
        // Subscribe to command velocities
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/diff_cont/cmd_vel_unstamped", 10,
            std::bind(&MapLocalizationNode::cmdVelCallback, this, std::placeholders::_1));

        // TF broadcaster (map -> odom)
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Timer to update pose and publish transform
        timer_ = create_wall_timer(50ms, std::bind(&MapLocalizationNode::update, this));

        last_time_ = now();
    }

private:

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        v_ = msg->linear.x;
        w_ = msg->angular.z;
    }

    void update()
    {
        // zasad ovako fiksno, dok ne bude senzora
        // posle moze da se subskribe na /odom i radi korekcija pomocu senzora
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "map";
        t.child_frame_id = "odom";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(t);
        // // Compute time delta
        // rclcpp::Time current_time = now();
        // double dt = (current_time - last_time_).seconds();
        // last_time_ = current_time;

        // // Dead reckoning pose update from velocity commands
        // x_ += v_ * std::cos(theta_) * dt;
        // y_ += v_ * std::sin(theta_) * dt;
        // theta_ += w_ * dt;

        // // Normalize theta
        // if (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        // else if (theta_ < -M_PI) theta_ += 2.0 * M_PI;

        // // Create transform message (map -> odom)
        // geometry_msgs::msg::TransformStamped t;
        // t.header.stamp = current_time;
        // t.header.frame_id = "map";
        // t.child_frame_id = "odom";

        // t.transform.translation.x = x_;
        // t.transform.translation.y = y_;
        // t.transform.translation.z = 0.0;

        // tf2::Quaternion q;
        // q.setRPY(0, 0, theta_);
        // t.transform.rotation.x = q.x();
        // t.transform.rotation.y = q.y();
        // t.transform.rotation.z = q.z();
        // t.transform.rotation.w = q.w();

        // // Publish transform
        // tf_broadcaster_->sendTransform(t);
    }

    double x_, y_, theta_;
    double v_ = 0.0, w_ = 0.0;
    rclcpp::Time last_time_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}