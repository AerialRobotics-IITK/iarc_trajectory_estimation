#include <dubins_trajectory_generator/dubins_trajectory.hpp>

const double PI=3.14159265358979323846;
namespace ariitk::trajectory_generation {

    DubinsTrajectory::DubinsTrajectory(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {

        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);                                                               
        trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);                                                                      
        server_ = nh.advertiseService("command",&DubinsTrajectory::commandServiceCallback,this);
        client_ = nh.serviceClient<std_srvs::Trigger>("command");    

        nh_private.param("initial_vel", initial_vel_, 1.0);
        nh_private.param("v_max", v_max_, 2.0);
        nh_private.param("a_max", a_max_, 2.0);
        nh_private.param("visualize", visualize_, true);
        nh_private.param("curvature", curvature_, 2.0);
        nh_private.param("num_arc", num_arc_, 5);
        nh_private.param("num_straight", num_straight_, 4);
        nh_private.param("command", command_, true);
        nh_private.param("distance", distance_, 1.0);
        nh_private.param("dimension", dimension_, 3);
        
        pylon_one_.x() = 0.0;
        pylon_one_.y() = 8.66;
        pylon_one_.z() = 1.0;
        pylon_two_.x() = 0.0;
        pylon_two_.y() = 408.66;
        pylon_two_.z() = 1.0;

        launch_pos_.x()=-5.0;
        launch_pos_.y()=0.0;
        launch_pos_.z()=0.0;

        hunter_killer_.x() = 5.0;
        hunter_killer_.y() = 1.505;    // Kept it a bit away from the Hunter Killer just to avoid any collision. It can be changed as per need.
        hunter_killer_.z() = 1.0;

        turn_in_one_segment_= PI/2 - atan((pylon_two_.x() - pylon_one_.x())/(pylon_two_.y() - pylon_one_.y()));  // Determines the angle our MAV turns in one single curved part.

        seperation_pylons_ = sqrt((pylon_two_.x() - pylon_one_.x()) * (pylon_two_.x() - pylon_one_.x())
                                        + (pylon_two_.y() - pylon_one_.y()) * (pylon_two_.y() - pylon_one_.y()));   // Distance between the two pylons.

        small_change_in_angle_ = (turn_in_one_segment_ )/num_arc_;
        small_change_in_distance_ = seperation_pylons_/num_straight_; 
            small_change_in_distance_ = seperation_pylons_/num_straight_; 
        small_change_in_distance_ = seperation_pylons_/num_straight_; 

        DubinsTrajectory::computeTangencyPoints();
        DubinsTrajectory::computePoints();
        DubinsTrajectory::generateTrajectory();

    }

    void DubinsTrajectory::computeTangencyPoints() {

        double a,b,c;    // Declaring these values just to copy some other values in order to make the expression of tangency_angle_ (calculated below) shorter.
        a = launch_pos_.x();
        b = pylon_one_.y();
        c = curvature_;
        // Calculating tangency_angle_ assuming that launch position is on x axis and pylon_one_ is on the y axis.
        tangency_angle_ = acos((b*c + sqrt(pow(a,4) + a*a*b*b - c*c*a*a))/(b*b + a*a)); 

        first_tangency_point_.position_.x() = curvature_ * sin(tangency_angle_);
        first_tangency_point_.position_.y() = pylon_one_.y() - curvature_ * cos(tangency_angle_);
        first_tangency_point_.position_.z() = pylon_one_.z();
        first_tangency_point_.heading_angle_ = PI/2 - tangency_angle_;

        //Position of second_tangency_point_ is computed assuming that the position of hunter_killer_ is mirror image of the launch position along the y axis.
        second_tangency_point_.position_.x() = (-1) * first_tangency_point_.position_.x();
        second_tangency_point_.position_.y() = pylon_one_.y() - curvature_ * cos(tangency_angle_);
        second_tangency_point_.position_.z() = pylon_one_.z();
        second_tangency_point_.heading_angle_ = tangency_angle_;

    }

    void DubinsTrajectory::computeFirstHalfLoop(uint flag) {

        Point start, end;

        // Flag has been used to incorporate changes as the first and last loops are different from the rest ones due to the points of tangency.
        if(flag==1) {
            start = first_tangency_point_;
            turn_in_one_segment_ = first_tangency_point_.heading_angle_;
        }
        else {
            start.position_.x() = pylon_one_.x();
            start.position_.y() = pylon_one_.y() - curvature_;
            start.position_.z() = pylon_one_.z();
            start.heading_angle_= PI;
            tangency_angle_ = 0.0;
        }

        end.position_.x() = pylon_two_.x();
        end.position_.y() = pylon_two_.y() + curvature_;
        end.position_.z() = pylon_two_.z();
        end.heading_angle_ = 0.0;

        Point prev_pos,curr_pos;
        prev_pos.position_ = start.position_;
        prev_pos.heading_angle_ = start.heading_angle_;
        curr_pos.position_ = start.position_;
        curr_pos.heading_angle_ = start.heading_angle_ ;
        if(flag==1) {
            prev_pos.heading_angle_ = start.heading_angle_ + PI/2;
            curr_pos.heading_angle_ = start.heading_angle_ + PI/2;
        }
        double angle_to_move = PI - tangency_angle_ - turn_in_one_segment_;   // Determines how much angle still needs to be turned before the current half loop ends.

        mav_trajectory_generation::Vertex prev(dimension_), curr(dimension_);

        while(prev_pos.heading_angle_ >= angle_to_move) {
            curr_pos.position_.y()=pylon_one_.y() + curvature_ * cos(prev_pos.heading_angle_);
            curr_pos.position_.x()=pylon_one_.x() + curvature_ * sin(prev_pos.heading_angle_);
            curr_pos.position_.z()=start.position_.z();
            curr_pos.heading_angle_ = prev_pos.heading_angle_ - small_change_in_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }

        double distance = 0.0;

        while(distance <= seperation_pylons_) {
            curr_pos.position_.y() = prev_pos.position_.y() + small_change_in_distance_ * sin(angle_to_move);
            curr_pos.position_.x() = prev_pos.position_.x() + small_change_in_distance_ * cos(angle_to_move);
            curr_pos.position_.z()=start.position_.z();
            distance = distance + small_change_in_distance_;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }

        prev_pos.heading_angle_ = angle_to_move - small_change_in_angle_;
        curr_pos.heading_angle_ = angle_to_move - small_change_in_angle_;

        while(prev_pos.heading_angle_ >= end.heading_angle_) {
            curr_pos.position_.y()=pylon_two_.y() + curvature_ * cos(prev_pos.heading_angle_);
            curr_pos.position_.x()=pylon_two_.x() + curvature_ * sin(prev_pos.heading_angle_);
            curr_pos.position_.z()=start.position_.z();
            curr_pos.heading_angle_ = prev_pos.heading_angle_ - small_change_in_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }
    }

    void DubinsTrajectory::computeSecondHalfLoop(uint flag) {

        Point start, end;

        start.position_.x() = pylon_two_.x();
        start.position_.y() = pylon_two_.y() + curvature_;
        start.position_.z() = pylon_two_.z();
        start.heading_angle_ = PI;

        // Flag has been used to incorporate changes as the first and last loops are different from the rest ones due to the points of tangency.
        if(flag==8) {
            end = second_tangency_point_;
        }
        else {
            end.position_.x() = pylon_one_.x();
            end.position_.y() = pylon_one_.y() - curvature_;
            end.position_.z() = pylon_one_.z();
            end.heading_angle_= 0.0;
        }
        tangency_angle_ = 0.0;          // Re-calculated just because when the flag is 1 this value is changed in the function computeFirstHalfLoop.
        turn_in_one_segment_=PI/2 - atan((pylon_two_.x() - 
                                pylon_one_.x())/(pylon_two_.y() - pylon_one_.y()));  // Re-calculated just because when the flag is 1 this value is changed in the function computeFirstHalfLoop.

        Point prev_pos,curr_pos;
        prev_pos.position_ = start.position_;
        prev_pos.heading_angle_ = start.heading_angle_;
        curr_pos.position_ = start.position_;
        curr_pos.heading_angle_ = start.heading_angle_ ;

        double angle_to_move = PI - tangency_angle_ - turn_in_one_segment_;
        mav_trajectory_generation::Vertex prev(dimension_), curr(dimension_);

        while(prev_pos.heading_angle_ >= angle_to_move) {
            curr_pos.position_.y()=pylon_two_.y() - curvature_ * cos(prev_pos.heading_angle_);
            curr_pos.position_.x()=pylon_two_.x() - curvature_ * sin(prev_pos.heading_angle_);
            curr_pos.position_.z()=start.position_.z();
            curr_pos.heading_angle_ = prev_pos.heading_angle_ - small_change_in_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }

        double distance = 0.0;

        while(distance <= seperation_pylons_) {
            curr_pos.position_.y() = prev_pos.position_.y() - small_change_in_distance_ * sin(angle_to_move);
            curr_pos.position_.x() = prev_pos.position_.x() + small_change_in_distance_ * cos(angle_to_move);
            curr_pos.position_.z() = start.position_.z();
            distance = distance + small_change_in_distance_;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }

        prev_pos.heading_angle_ = angle_to_move - small_change_in_angle_;
        curr_pos.heading_angle_ = angle_to_move - small_change_in_angle_;

        while(prev_pos.heading_angle_ >= end.heading_angle_) {
            curr_pos.position_.y() = pylon_one_.y() - curvature_ * cos(prev_pos.heading_angle_);
            curr_pos.position_.x() = pylon_one_.x() - curvature_ * sin(prev_pos.heading_angle_);
            curr_pos.position_.z() = start.position_.z();
            curr_pos.heading_angle_ = prev_pos.heading_angle_ - small_change_in_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position_.x(), curr_pos.position_.y(), curr_pos.position_.z()));
            vertices_.push_back(curr);
        }

    }

    void DubinsTrajectory::computePoints() {
        ROS_INFO("Calculating waypoints for the victorious forward journey"); //Please don't remove this line.Thanks!! :)

        mav_trajectory_generation::Vertex start(dimension_), end(dimension_);        
        derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;

        start.makeStartOrEnd(Eigen::Vector3d(launch_pos_.x(), launch_pos_.y(), launch_pos_.z()), derivative_to_optimize_);

        double a,b,c;    // Declaring these values just to copy some other values in order to make the expression of norm (calculated below) shorter.
        a = first_tangency_point_.position_.x() - launch_pos_.x();
        b = first_tangency_point_.position_.y() - launch_pos_.y();
        c = first_tangency_point_.position_.z() - launch_pos_.z();

        //Calculating norm for the direction of initial velocity of the MAV.
        double norm = sqrt(a*a + b*b + c*c);
        start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                            Eigen::Vector3d(initial_vel_ * a/norm, initial_vel_ * b/norm, initial_vel_ * c/norm));
        end.makeStartOrEnd(Eigen::Vector3d(first_tangency_point_.position_.x(), first_tangency_point_.position_.y(), first_tangency_point_.position_.z()), derivative_to_optimize_);

        vertices_.push_back(start);
        vertices_.push_back(end);

        for(uint i=1;i<=8;i++) {
            ROS_INFO("i= %d",i);
            ROS_INFO("size of vertices_: %d \n", vertices_.size());
            computeFirstHalfLoop(i);
            computeSecondHalfLoop(i);
        }

        mav_trajectory_generation::Vertex hunter_killer(dimension_);   
        hunter_killer.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(hunter_killer_.x(), hunter_killer_.y(), hunter_killer_.z()));
        vertices_.push_back(hunter_killer);

        ROS_INFO("Reversing waypoints for the victorious return journey"); //Please don't remove this line also.Thanks!! :)
        auto it=vertices_.end();
        it--;
        for( ;it>=vertices_.begin();it--) {
            reverse_vertices_.push_back(*it);
        }
        auto it2 = reverse_vertices_.begin();
        it2++;   //For proper segment_times.
        for( ;it2!=reverse_vertices_.end();it2++) {
            vertices_.push_back(*it2);
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

    }

    bool DubinsTrajectory::commandServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {

        mav_trajectory_generation::sampleWholeTrajectory(trajectory_, 0.1, &trajectory_points_);
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points_, &generated_trajectory_);

        if (command_) {
            trajectory_pub_.publish(generated_trajectory_);
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
        }
        
    }

} //namespace ariitk::trajectory_generation
