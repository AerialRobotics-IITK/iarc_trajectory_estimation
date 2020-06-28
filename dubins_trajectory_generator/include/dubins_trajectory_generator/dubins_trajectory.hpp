#pragma once

#include <ros/ros.h>
#include <memory>
#include <mav_planning_common/utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_msgs/conversions.h>
#include <mav_visualization/helpers.h>
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

class Color : public std_msgs::ColorRGBA {
    public:
    Color() : std_msgs::ColorRGBA() {}
    Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
    Color(double red, double green, double blue, double alpha) : Color() {
        r = red;
        g = green;
        b = blue;
        a = alpha;
    }

    static const Color White() { return Color(1.0, 1.0, 1.0); }
    static const Color Black() { return Color(0.0, 0.0, 0.0); }
    static const Color Gray() { return Color(0.5, 0.5, 0.5); }
    static const Color Red() { return Color(1.0, 0.0, 0.0); }
    static const Color Green() { return Color(0.0, 1.0, 0.0); }
    static const Color Blue() { return Color(0.0, 0.0, 1.0); }
    static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
    static const Color Orange() { return Color(1.0, 0.5, 0.0); }
    static const Color Purple() { return Color(0.5, 0.0, 1.0); }
    static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
    static const Color Teal() { return Color(0.0, 1.0, 1.0); }
    static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

class DubinsTrajectory {
    public:
    DubinsTrajectory(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private);
    void generateTrajectory(Point start, Point end);
    enum class ColorType{WHITE, BLACK, GRAY, RED, GREEN, BLUE, YELLOW, ORANGE, PURPLE, CHARTREUSE, TEAL, PINK};
    void visualizeTrajectory(const std::string& topic_name, std::vector<Point> trajectory, 
                                    const std::string& frame_id = "world", const ColorType& color = ColorType::PINK, const double& size_factor= 0.5);
    void init(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();

    private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;    

    double v_max_;
    double a_max_;
    double turn_in_one_segment_;
    double curvature_; 
    double size_factor_;
    double distance_; //distance between the centres of two pylons.
    int num_arc_; 
    int num_straight_;

    bool visualize_;

    Eigen::Vector3d pylon_one_;
    Eigen::Vector3d pylon_two_;
    Point start_;
    Point end_;
    std::unordered_map<ColorType, Color> color_map_;
    // visualization_msgs::MarkerArray markers_;
    std::vector<Point> trajectory_;
    ros::Publisher marker_pub_;

};

} //namespace ariitk::trajectory_generation

