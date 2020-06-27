#pragma once

#include <ros/ros.h>
#include <memory>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>

namespace ariitk::trajectory_generation {

struct Point {
    Eigen::Vector3d position_;
    double heading_angle_;
    typedef std::shared_ptr<struct Point> Ptr;
    typedef Point::Ptr Node;
};

class DubinsTrajectory {
    public:
    DubinsTrajectory(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private);
    void generateTrajectory(std::vector<Point> trajectory);
    void init();
    void run();

    private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;    

    double v_max_;
    double a_max_;
    double turn_in_one_segment_;
    double curvature_; 
    double distance_; //distance between the centres of two pylons.
    int num_arc_; 
    int num_straight_;

    bool visualize_;

    Eigen::Vector3d pylon_one_;
    Eigen::Vector3d pylon_two_;
    Point start_;
    Point end_;
    visualization_msgs::MarkerArray markers_;
    std::vector<Point> trajectory_;
    ros::Publisher marker_pub_;

};

} //namespace ariitk::trajectory_generation

