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
    Eigen::Vector3d position_;
    double heading_angle_;
};

class DubinsTrajectory {
    public:
    DubinsTrajectory(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private);
    void generateTrajectory();
    void run();

    private:
    void computeTangencyPoints();
    void computeFirstHalfLoop(uint flag);
    void computeSecondHalfLoop(uint flag);
    void computePoints();
    bool commandServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;    

    double v_max_;
    double a_max_;
    double turn_in_one_segment_;
    double curvature_; 
    double size_factor_;
    double seperation_pylons_; //distance between the centres of two pylons.
    double distance_;
    double small_change_in_angle_;
    double small_change_in_distance_;
    double tangency_angle_;
    int num_arc_; 
    int num_straight_;
    int dimension_;
    int derivative_to_optimize_;

    bool visualize_;
    bool command_;

    Point start_,end_;
    Point first_tangency_point_;
    Point second_tangency_point_;
    Eigen::Vector3d pylon_one_;
    Eigen::Vector3d pylon_two_;
    Eigen::Vector3d hunter_killer_;
    mav_trajectory_generation::Vertex::Vector vertices_;
    mav_trajectory_generation::Vertex::Vector reverse_vertices_;
    mav_trajectory_generation::Trajectory trajectory_;
    mav_trajectory_generation::Trajectory reverse_trajectory_;
    std::vector<double> segment_times_;
    std::vector<double> reverse_segment_times_;

    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points_;
    mav_msgs::EigenTrajectoryPoint::Vector reverse_trajectory_points_;
    trajectory_msgs::MultiDOFJointTrajectory generated_trajectory_;
    trajectory_msgs::MultiDOFJointTrajectory reverse_generated_trajectory_;
    visualization_msgs::MarkerArray markers_;
    visualization_msgs::MarkerArray reverse_markers_;
    ros::Publisher marker_pub_;
    ros::Publisher reverse_marker_pub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher reverse_trajectory_pub_;
    ros::ServiceServer server_;
    ros::ServiceClient client_;

};

} //namespace ariitk::trajectory_generation

