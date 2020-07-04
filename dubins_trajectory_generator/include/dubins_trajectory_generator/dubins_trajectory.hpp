#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <mav_planning_common/utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_visualization/helpers.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>


namespace ariitk::trajectory_generation {

struct Point {
    Point(const Eigen::Vector3d& pos, const double& yaw = 0.0)
        : position(pos)
        , yaw(yaw) {}
        
    Point(const double&x, const double& y, const double& z, const double& yaw = 0.0) 
       : Point(Eigen::Vector3d(x,y,z), yaw) {}
    
    Eigen::Vector3d position;
    double yaw = 0.0;
};

class DubinsTrajectory {
    public:
    DubinsTrajectory(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void generateTrajectory();
    void run();

    private:
    void computeTangencyPoints();
    void computeFirstHalfLoop(uint flag);
    void computeSecondHalfLoop(uint flag);
    void computePoints();
    bool commandServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);  

    double v_max_;
    double a_max_;
    double arc_angle_;
    double arc_radius_; 
    double size_factor_;
    double interpylon_distance_;
    double delta_angle_;
    double delta_distance_;
    double tangency_angle_;
    double initial_vel_;
    int num_arc_points_; 
    int num_linear_points_;
    int dimension_;
    int derivative_to_optimize_;

    bool visualize_;
    bool command_;

    Point arc_entry_point_;
    Point arc_exit_point_;
    Eigen::Vector3d left_pylon_;
    Eigen::Vector3d right_pylon_;
    Eigen::Vector3d hunter_killer_;
    Eigen::Vector3d launch_pos_;
    mav_trajectory_generation::Vertex::Vector vertices_;
    mav_trajectory_generation::Vertex::Vector reverse_vertices_;
    mav_trajectory_generation::Trajectory trajectory_;
    std::vector<double> segment_times_;

    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points_;
    trajectory_msgs::MultiDOFJointTrajectory generated_trajectory_;
    visualization_msgs::MarkerArray markers_;
    ros::Publisher marker_pub_;
    ros::Publisher trajectory_pub_;
    ros::ServiceServer server_;
    ros::ServiceClient client_;

};

} //namespace ariitk::trajectory_generation
