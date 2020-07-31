#pragma once

#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <mast_finder/Setpoint.h>
#include <mast_finder/libmast_finder.hpp>

namespace iarc2020::mast_locator {
class MastLocatorNode {
  public:
    void init(ros::NodeHandle& nh);
    void run();

  private:
    MastLocator locate_;

    mast_finder::Setpoint setpoint_;
    mast_finder::Setpoint ship_centre_;
    nav_msgs::Odometry odom_;
    float radius_;
    int n_sides_;

    ros::Subscriber odom_sub_;

    void odomCallback(const nav_msgs::Odometry& msg);

    ros::Publisher setpoint_pub_;
};

}  // namespace iarc2020::mast_locator
