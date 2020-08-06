#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <detector_msgs/GlobalCoord.h>
#include <mast_finder/Setpoint.h>
#include <mast_finder/libmast_finder.hpp>

namespace iarc2020::mast_locator {
class MastLocatorNode {
  public:
    void init(ros::NodeHandle& nh);
    void run();
    void publishSetpoint();
    void publishYaw();
    void publishMsg();

  private:
    MastLocator locate_;

    mast_finder::Setpoint setpoint_;
    mast_finder::Setpoint ship_centre_;
    detector_msgs::GlobalCoord front_coord_;
    nav_msgs::Odometry odom_;
    geometry_msgs::PoseStamped next_setpt_;
    float radius_;
    int n_sides_;
    float transition_rate_;
    double yaw_change_;
    bool scouting_done_;
    int sides_done_;

    ros::Subscriber odom_sub_;
    ros::Subscriber front_coord_sub_;

    void odomCallback(const nav_msgs::Odometry& msg);
    void frontCallback(const detector_msgs::GlobalCoord& msg);

    ros::Publisher setpoint_pub_;
};

}  // namespace iarc2020::mast_locator
