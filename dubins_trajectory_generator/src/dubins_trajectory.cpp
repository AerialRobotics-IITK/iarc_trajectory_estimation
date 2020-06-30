#include <dubins_trajectory_generator/dubins_trajectory.hpp>

const double PI=3.14159265358979323846;
namespace ariitk::trajectory_generation {

    DubinsTrajectory::DubinsTrajectory(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
      : nh_(nh),
        nh_private_(nh_private) {

            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",
                                                                             10);
            reverse_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",
                                                                             10);                                                                 
            trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory",
                                                                                  10);
            reverse_trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory",
                                                                                  10);                                                                      
            server_ = nh_.advertiseService("command",&DubinsTrajectory::commandServiceCallback,this);
            client_ = nh_.serviceClient<std_srvs::Trigger>("command");    

            nh_private_.param("v_max", v_max_, 2.0);
            nh_private_.param("a_max", a_max_, 2.0);
            nh_private_.param("visualize", visualize_, true);
            nh_private_.param("curvature", curvature_, 2.0);
            nh_private_.param("num_arc", num_arc_, 50);
            nh_private_.param("num_straight", num_straight_, 100);
            nh_private_.param("size_factor", size_factor_, 0.05);
            nh_private_.param("command", command_, true);
            nh_private_.param("distance", distance_, 1.0);
            nh_private_.param("dimension", dimension_, 3);
        
            pylon_one_.x() = 0.0;
            pylon_one_.y() = 8.66;
            pylon_one_.z() = 1.0;
            pylon_two_.x() = 0.0;
            pylon_two_.y() = 12.66;
            pylon_two_.z() = 1.0;

            start_.position_.x()=-5.0;
            start_.position_.y()=0.0;
            start_.position_.z()=1.0;
            start_.heading_angle_= PI;

            end_.position_.x()=0.0;
            end_.position_.y()=6.66;
            end_.position_.z()=1.0;
            end_.heading_angle_ = 0.0;

            turn_in_one_segment_=PI/2 - atan((pylon_two_.x() - 
                                pylon_one_.x())/(pylon_two_.y() - pylon_one_.y()));

            seperation_pylons_ = sqrt((pylon_two_.x() - pylon_one_.x()) * (pylon_two_.x() - pylon_one_.x())
                            + (pylon_two_.y() - pylon_one_.y()) * (pylon_two_.y() - pylon_one_.y()));

            small_change_in_angle_ = (turn_in_one_segment_ )/num_arc_;
            small_change_in_distance_ = seperation_pylons_/num_straight_; 

            DubinsTrajectory::computePoints();
            DubinsTrajectory::generateTrajectory();
    }

    void DubinsTrajectory::computeFirstHalfLoop() {

        start_.position_.x()=0.0;
        start_.position_.y()=6.66;
        start_.position_.z()=1.0;
        start_.heading_angle_= PI;

        end_.position_.x()=0.0;
        end_.position_.y()=14.66;
        end_.position_.z()=1.0;
        end_.heading_angle_ = 0.0;

        Point prev_pos,curr_pos;
        prev_pos.position_ = start_.position_;
        prev_pos.heading_angle_ = start_.heading_angle_;
        curr_pos.position_ = start_.position_;
        curr_pos.heading_angle_ = start_.heading_angle_ ;

        mav_trajectory_generation::Vertex prev(dimension_), curr(dimension_);

        while(prev_pos.heading_angle_ >= turn_in_one_segment_) {
            curr_pos.position_.y()=pylon_one_.y() + curvature_ * cos(prev_pos.heading_angle_);
            curr_pos.position_.x()=pylon_one_.x() + curvature_ * sin(prev_pos.heading_angle_);
            curr_pos.position_.z()=start_.position_.z();
            curr_pos.heading_angle_ = prev_pos.heading_angle_ - small_change_in_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }

        double distance = 0.0;

        while(distance <= seperation_pylons_) {
            curr_pos.position_.y() = prev_pos.position_.y() + small_change_in_distance_ * sin(turn_in_one_segment_);
            curr_pos.position_.x() = prev_pos.position_.x() + small_change_in_distance_ * cos(turn_in_one_segment_);
            curr_pos.position_.z()=start_.position_.z();
            distance = distance + small_change_in_distance_;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }

        prev_pos.heading_angle_ = turn_in_one_segment_ - small_change_in_angle_;
        curr_pos.heading_angle_ = turn_in_one_segment_ - small_change_in_angle_;

        while(prev_pos.heading_angle_ >= end_.heading_angle_) {
            curr_pos.position_.y()=pylon_two_.y() + curvature_ * cos(prev_pos.heading_angle_);
            curr_pos.position_.x()=pylon_two_.x() + curvature_ * sin(prev_pos.heading_angle_);
            curr_pos.position_.z()=start_.position_.z();
            curr_pos.heading_angle_ = prev_pos.heading_angle_ - small_change_in_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }
    }

    void DubinsTrajectory::computeSecondHalfLoop() {


        start_.position_.x()=0.0;
        start_.position_.y()=14.66;
        start_.position_.z()=1.0;
        start_.heading_angle_ = PI;

        end_.position_.x()=0.0;
        end_.position_.y()=6.66;
        end_.position_.z()=1.0;
        end_.heading_angle_= 0.0;

        Point prev_pos,curr_pos;
        prev_pos.position_ = start_.position_;
        prev_pos.heading_angle_ = start_.heading_angle_;
        curr_pos.position_ = start_.position_;
        curr_pos.heading_angle_ = start_.heading_angle_ ;

        mav_trajectory_generation::Vertex prev(dimension_), curr(dimension_);

        while(prev_pos.heading_angle_ >= turn_in_one_segment_) {
            curr_pos.position_.y()=pylon_two_.y() - curvature_ * cos(prev_pos.heading_angle_);
            curr_pos.position_.x()=pylon_two_.x() - curvature_ * sin(prev_pos.heading_angle_);
            curr_pos.position_.z()=start_.position_.z();
            curr_pos.heading_angle_ = prev_pos.heading_angle_ - small_change_in_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }

        double distance = 0.0;

        while(distance <= seperation_pylons_) {
            curr_pos.position_.y() = prev_pos.position_.y() - small_change_in_distance_ * sin(turn_in_one_segment_);
            curr_pos.position_.x() = prev_pos.position_.x() + small_change_in_distance_ * cos(turn_in_one_segment_);
            curr_pos.position_.z()=start_.position_.z();
            distance = distance + small_change_in_distance_;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }

        prev_pos.heading_angle_ = turn_in_one_segment_ - small_change_in_angle_;
        curr_pos.heading_angle_ = turn_in_one_segment_ - small_change_in_angle_;

        while(prev_pos.heading_angle_ >= end_.heading_angle_) {
            curr_pos.position_.y()=pylon_one_.y() - curvature_ * cos(prev_pos.heading_angle_);
            curr_pos.position_.x()=pylon_one_.x() - curvature_ * sin(prev_pos.heading_angle_);
            curr_pos.position_.z()=start_.position_.z();
            curr_pos.heading_angle_ = prev_pos.heading_angle_ - small_change_in_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }

    }

    void DubinsTrajectory::computePoints() {
        ROS_INFO("Calculating waypoints for the victorious forward journey"); //Please don't remove this line.Thanks!! :)

        mav_trajectory_generation::Vertex start(dimension_), end(dimension_);        
        derivative_to_optimize_ = mav_trajectory_generation::derivative_order::VELOCITY;

        start.makeStartOrEnd(Eigen::Vector3d(start_.position_.x(), start_.position_.y(), start_.position_.z()), derivative_to_optimize_);
        start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                            Eigen::Vector3d(5.0/8.3878, 6.66/8.3878, -1.0/8.3878));
        end.makeStartOrEnd(Eigen::Vector3d(end_.position_.x(), end_.position_.y(), end_.position_.z()), derivative_to_optimize_);

        vertices_.push_back(start);
        vertices_.push_back(end);

        ROS_INFO("size of vertices_: %d \n", vertices_.size());
        for(uint i=1;i<=8;i++) {
            ROS_INFO("i= %d\n",i);
            computeFirstHalfLoop();
            computeSecondHalfLoop();
        }

        mav_trajectory_generation::Vertex end2(dimension_);   
        end2.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(4.0, 1.505, 1.0));
        vertices_.push_back(end2);

        ROS_INFO("Reversing waypoints for the victorious return journey"); //Please don't remove this line also.Thanks!! :)
        auto it=vertices_.end();
        it--;
        for( ;it>=vertices_.begin();it--) {
            reverse_vertices_.push_back(*it);
        }

    }

    void DubinsTrajectory::generateTrajectory() {

        segment_times_ = mav_trajectory_generation::estimateSegmentTimes(vertices_, v_max_, a_max_);

        mav_trajectory_generation::PolynomialOptimization<10> opt(dimension_);
        opt.setupFromVertices(vertices_, segment_times_, derivative_to_optimize_);
        opt.solveLinear();
        opt.getTrajectory(&trajectory_);

        std::string frame_id = "world";
        mav_trajectory_generation::drawMavTrajectory(trajectory_, distance_, frame_id, &markers_);



        // reverse_segment_times_ = mav_trajectory_generation::estimateSegmentTimes(reverse_vertices_, v_max_, a_max_);

        // mav_trajectory_generation::PolynomialOptimization<10> reverse_opt(dimension_);
        // reverse_opt.setupFromVertices(reverse_vertices_, reverse_segment_times_, derivative_to_optimize_);
        // reverse_opt.solveLinear();
        // reverse_opt.getTrajectory(&reverse_trajectory_);

        // std::string frame_id = "world";
        // mav_trajectory_generation::drawMavTrajectory(reverse_trajectory_, distance_, frame_id, &reverse_markers_);

    }

    bool DubinsTrajectory::commandServiceCallback(std_srvs::Trigger::Request &req,
                                                                std_srvs::Trigger::Response &resp)
    {

        mav_trajectory_generation::sampleWholeTrajectory(trajectory_, 0.1, &trajectory_points_);
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points_, &generated_trajectory_);

        // mav_trajectory_generation::sampleWholeTrajectory(reverse_trajectory_, 0.1, &reverse_trajectory_points_);
        // mav_msgs::msgMultiDofJointTrajectoryFromEigen(reverse_trajectory_points_, &reverse_generated_trajectory_);

        if (command_)
        {
            trajectory_pub_.publish(generated_trajectory_);
            // reverse_trajectory_pub_.publish(reverse_generated_trajectory_);
        }

        resp.success = true;
        resp.message = "Trajectory given as command";
        ROS_INFO("%s\n",resp.message.c_str());
        return true;
    }

    void DubinsTrajectory::run() {

         if (visualize_)
        {
            marker_pub_.publish(markers_);
            // reverse_marker_pub_.publish(reverse_markers_);
        }
        
    }

} //namespace ariitk::trajectory_generation
