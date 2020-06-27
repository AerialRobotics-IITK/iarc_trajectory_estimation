#include <dubins_trajectory_generator/dubins_trajectory.hpp>

const double PI=3.14159265358979323846;
namespace ariitk::trajectory_generation {

    DubinsTrajectory::DubinsTrajectory(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
      : nh_(nh),
        nh_private_(nh_private) {

            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",
                                                                             10);
            nh_private_.param("v_max", v_max_, 2.0);
            nh_private_.param("a_max", a_max_, 2.0);
            nh_private_.param("visualize", visualize_, true);
            nh_private_.param("curvature", curvature_, 2.0);
            nh_private_.param("num_arc", num_arc_, 50);
            nh_private_.param("num_straight", num_straight_, 100);
        
            pylon_one_.x() = 1.0;
            pylon_one_.y() = 1.0;
            pylon_one_.z() = 10.0;
            pylon_two_.x() = 401.0;
            pylon_two_.y() = 1.0;
            pylon_two_.z() = 10.0;
    }

    void DubinsTrajectory::init() {
        start_.position_.x()=1.0;
        start_.position_.y()=1.0;
        start_.position_.z()=1.0;
        
        end_.position_.x()=5.0;
        end_.position_.y()=5.0;
        end_.position_.z()=1.0;
    }

    void DubinsTrajectory::generateTrajectory(std::vector<Point> trajectory) {
        turn_in_one_segment_=PI/2 - atan((end_.position_.y() - 
                                start_.position_.y())/(end_.position_.x() - start_.position_.x()));

        distance_ = sqrt((pylon_two_.x() - pylon_one_.x()) * (pylon_two_.x() - pylon_one_.x())
                            + (pylon_two_.y() - pylon_one_.y()) * (pylon_two_.y() - pylon_one_.y()));

        double small_change_in_angle = (turn_in_one_segment_ )/num_arc_;
        double small_change_in_distance = distance_/num_straight_; 

        start_.heading_angle_ = 0.0;
        Point prev = start_;
        Point curr = start_;
        curr.heading_angle_ = prev.heading_angle_ + small_change_in_angle ;
        trajectory_.push_back(curr);

        while(prev.heading_angle_ <= turn_in_one_segment_) {
            curr.position_.x()=pylon_one_.x() + curvature_ * sin(prev.heading_angle_);
            curr.position_.y()=pylon_one_.y() + curvature_ * cos(prev.heading_angle_);
            curr.heading_angle_ = prev.heading_angle_ + small_change_in_angle ;
            trajectory_.push_back(curr);
            prev = curr;
        }

        double distance = 0.0;

        while(distance <= distance_) {
            curr.position_.x() = prev.position_.x() + small_change_in_distance * sin(turn_in_one_segment_);
            curr.position_.y() = prev.position_.y() + small_change_in_distance * cos(turn_in_one_segment_);
            distance = distance + small_change_in_distance;
            trajectory_.push_back(curr);
            prev = curr;
        }

        while(prev.heading_angle_ <= end_.heading_angle_) {
            curr.position_.x()=pylon_two_.x() + curvature_ * sin(prev.heading_angle_);
            curr.position_.y()=pylon_two_.y() + curvature_ * cos(prev.heading_angle_);
            curr.heading_angle_ = prev.heading_angle_ + small_change_in_angle ;
            trajectory_.push_back(curr);
            prev = curr;
        }

    }

    void DubinsTrajectory::run() {
        if(visualize_) {
            marker_pub_.publish(markers_);
        }
    }

} //namespace ariitk::trajectory_generation
