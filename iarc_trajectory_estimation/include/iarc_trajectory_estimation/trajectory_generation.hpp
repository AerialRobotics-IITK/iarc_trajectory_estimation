#pragma once

#include <Eigen/Dense>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace ariitk::trajectory_generation {

class PolynomialTrajectoryGeneration {
  
  public:
    PolynomialTrajectoryGeneration(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();

  private:
    void mavOdometryCallback(const nav_msgs::Odometry& msg);
    void computePoints();
    void generateTrajectory();
    bool commandServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

    bool visualize_;
    bool command_;

    double v_max_;
    double a_max_;
    double distance_;

    int dimension_;
    int derivative_to_optimize_;

    bool publish_;

    std::vector<double> segment_times_;

    mav_trajectory_generation::Vertex::Vector vertices_;
    mav_trajectory_generation::Trajectory trajectory_;

    nav_msgs::Odometry mav_odom_;
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points_;
    trajectory_msgs::MultiDOFJointTrajectory generated_trajectory_;
    visualization_msgs::MarkerArray markers_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber mav_odom_sub_;
    ros::Publisher mav_odom_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher trajectory_pub_;
    ros::ServiceServer server_;
    ros::ServiceClient client_;

    // std_srvs::Trigger srv_;
};

}  // namespace ariitk::trajectory_generation