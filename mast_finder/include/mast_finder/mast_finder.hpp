#pragma once

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <tf/tf.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <detector_msgs/Centre.h>
#include <detector_msgs/GlobalCoord.h>
#include <mast_finder/Setpoint.h>
#include <mast_finder/libmast_finder.hpp>

namespace iarc2020::mast_locator {
class MastLocatorNode {
  public:
    void init(ros::NodeHandle& nh);
    void run();
    //* Manual setpoint publishing, takes ~20 sec to reach the mast
    void publishSetpoint();
    void publishYaw();
    void publishMsg();
    void isMastDetected();
    void ifMastDetected();
    //* Using mav_trajectory_generation
    void test();
    void planTrajectory();
    void getTrajectory();
    void publishTrajectory();

  private:
    MastLocator locate_;

    mast_finder::Setpoint setpoint_;
    mast_finder::Setpoint ship_centre_;
    detector_msgs::GlobalCoord front_coord_;
    detector_msgs::GlobalCoord pose_;
    detector_msgs::Centre centre_coord_;
    nav_msgs::Odometry odom_;
    geometry_msgs::PoseStamped next_setpt_;
    //* mav_traj_gen variables -> start
    Eigen::Vector4d traj_point_;
    Eigen::Vector4d traj_vel_;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex::Vector vertices_;
    mav_trajectory_generation::Trajectory traj_;
    std::vector<double> segment_times_;
    mav_trajectory_generation::NonlinearOptimizationParameters parameters_;
    trajectory_msgs::MultiDOFJointTrajectory generated_traj_;
    float a_max_;
    float v_max_;
    bool flag = false;
    //* mav_traj_gen variables -> end
    float radius_;
    int n_sides_;
    float transition_rate_;
    double yaw_change_;
    bool scouting_done_;  //* Flag for after mast is detected
    int sides_done_;

    ros::Subscriber odom_sub_;
    ros::Subscriber front_coord_sub_;
    ros::Subscriber centre_sub_;
    ros::Subscriber pose_sub_;

    void odomCallback(const nav_msgs::Odometry& msg);
    void frontCallback(const detector_msgs::GlobalCoord& msg);
    void centreCallback(const detector_msgs::Centre& msg);
    void poseCallback(const detector_msgs::GlobalCoord& msg);

    ros::Publisher setpoint_pub_;
    ros::Publisher traj_pub_;
};

}  // namespace iarc2020::mast_locator
