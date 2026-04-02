#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Core>
#include <algorithm>

class MPCController
{
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_sub_;
    ros::Publisher attitude_pub_;

    Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_vel_{Eigen::Vector3d::Zero()};
    double kp_pos_;
    double kd_vel_;
    double hover_thrust_;
    bool has_odom_{false};
    bool has_cmd_{false};

public:
    MPCController(ros::NodeHandle &nh) : nh_(nh)
    {
        nh_.param("kp_pos", kp_pos_, 2.0);
        nh_.param("kd_vel", kd_vel_, 0.8);
        nh_.param("hover_thrust", hover_thrust_, 0.50);

        odom_sub_ = nh_.subscribe("/Odometry", 10, &MPCController::odomCallback, this);
        cmd_sub_ = nh_.subscribe("/position_cmd", 10, &MPCController::cmdCallback, this);
        attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 20);
    }

    void odomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        pos_ << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;
        vel_ << msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z;
        has_odom_ = true;
        publishControl();
    }

    void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg)
    {
        target_pos_ << msg->position.x, msg->position.y, msg->position.z;
        target_vel_ << msg->velocity.x, msg->velocity.y, msg->velocity.z;
        has_cmd_ = true;
        publishControl();
    }

    void publishControl()
    {
        if (!has_odom_ || !has_cmd_)
        {
            return;
        }

        const Eigen::Vector3d e_pos = target_pos_ - pos_;
        const Eigen::Vector3d e_vel = target_vel_ - vel_;
        const Eigen::Vector3d acc_cmd = kp_pos_ * e_pos + kd_vel_ * e_vel;

        mavros_msgs::AttitudeTarget out;
        out.header.stamp = ros::Time::now();
        out.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        out.body_rate.x = std::clamp(acc_cmd.y(), -1.0, 1.0);  // 近似映射：pitch rate
        out.body_rate.y = std::clamp(-acc_cmd.x(), -1.0, 1.0); // 近似映射：roll rate
        out.body_rate.z = 0.0;
        out.thrust = std::clamp(static_cast<float>(hover_thrust_ + 0.05 * acc_cmd.z()), 0.0f, 1.0f);
        attitude_pub_.publish(out);

        ROS_INFO_THROTTLE(1.0, "mpc_controller: thrust=%.3f, body_rate=(%.3f, %.3f)",
                          out.thrust, out.body_rate.x, out.body_rate.y);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_controller_node");
    ros::NodeHandle nh("~");
    MPCController node(nh);
    ros::spin();
    return 0;
}
