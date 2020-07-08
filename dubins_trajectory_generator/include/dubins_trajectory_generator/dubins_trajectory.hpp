#pragma once

#include <eigen3/Eigen/Dense>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>

namespace ariitk::trajectory_generation {

struct Point {
    Point(const Eigen::Vector3d& pos, const double& yaw = 0.0)
        : position(pos)
        , yaw(yaw) {
    }

    Point(const double& x, const double& y, const double& z, const double& yaw = 0.0)
        : Point(Eigen::Vector3d(x, y, z), yaw) {
    }

    Point()
        : position(Eigen::Vector3d(0, 0, 0))
        , yaw(0.0) {
    }

    Eigen::Vector3d position;
    double yaw = 0.0;
};

class DubinsTrajectory {
  public:
    DubinsTrajectory(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void generateTrajectory();
    void run();
    bool publishStartingTrajectory(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
    bool publishReturnTrajectory(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

  private:
    void loadParams(ros::NodeHandle& nh_private);
    void computeTangencyPoints();
    void computeFirstHalfLoop(const uint& lap_number);
    void computeSecondHalfLoop(const uint& lap_number);
    void computePoints();
    bool commandServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

    double v_max_;
    double a_max_;
    double arc_angle_;
    double arc_radius_;
    double distance_;
    double interpylon_distance_;
    double delta_angle_;
    double delta_distance_;
    double tangency_angle_;
    double initial_vel_;
    int num_arc_points_;
    int num_linear_points_;
    int derivative_to_optimize_;
    uint num_laps_;

    bool visualize_;
    bool command_;
    bool starting_command_;
    bool return_command_;

    Point loop_entry_point_;
    Point loop_exit_point_;
    Eigen::Vector3d left_pylon_;
    Eigen::Vector3d right_pylon_;
    Eigen::Vector3d hunter_killer_;
    Eigen::Vector3d launch_pos_;
    mav_trajectory_generation::Vertex::Vector vertices_;
    mav_trajectory_generation::Vertex::Vector starting_vertices_;
    mav_trajectory_generation::Vertex::Vector reverse_vertices_;
    mav_trajectory_generation::Trajectory trajectory_;
    mav_trajectory_generation::Trajectory starting_trajectory_;
    mav_trajectory_generation::Trajectory return_trajectory_;

    visualization_msgs::MarkerArray markers_;
    visualization_msgs::MarkerArray starting_markers_;
    visualization_msgs::MarkerArray return_markers_;
    ros::Publisher marker_pub_;
    ros::Publisher starting_marker_pub_;
    ros::Publisher return_marker_pub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher starting_trajectory_pub_;
    ros::Publisher return_trajectory_pub_;
    ros::ServiceServer publish_trajectory_server_;
    ros::ServiceServer publish_starting_traj_srv_;
    ros::ServiceServer publish_return_traj_srv_;
    ros::ServiceClient publish_trajectory_client_;
    ros::ServiceClient publish_starting_traj_client_;
    ros::ServiceClient publish_return_traj_client_;
};

}  // namespace ariitk::trajectory_generation
