#include <dubins_trajectory_generator/dubins_trajectory.hpp>

namespace ariitk::trajectory_generation {

    DubinsTrajectory::DubinsTrajectory(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
    : nh_(nh),
      nh_private_(nh_private) {

        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);                                                               
        trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);                                                                      
        publish_trajectory_server_ = nh_.advertiseService("command",&DubinsTrajectory::commandServiceCallback,this);
        publish_trajectory_client_ = nh_.serviceClient<std_srvs::Trigger>("command");    

        DubinsTrajectory::loadParams(nh_private);

        arc_angle_= M_PI/2 - atan((right_pylon_.x() - left_pylon_.x())/(right_pylon_.y() - left_pylon_.y()));  // Determines the angle our MAV turns in one single curved part.

        interpylon_distance_ = (left_pylon_ - right_pylon_).norm();  // Distance between the two pylons

        delta_angle_ = (arc_angle_ )/num_arc_points_;
        delta_distance_ = interpylon_distance_/num_linear_points_;  

        DubinsTrajectory::computeTangencyPoints();
        DubinsTrajectory::computePoints();
        DubinsTrajectory::generateTrajectory();

    }

    void DubinsTrajectory::loadParams(ros::NodeHandle& nh_private) {
        nh_private_ = nh_private;

        nh_private_.param("initial_vel", initial_vel_, 1.0);
        nh_private_.param("v_max", v_max_, 2.0);
        nh_private_.param("a_max", a_max_, 2.0);
        nh_private_.param("visualize", visualize_, true);
        nh_private_.param("arc_radius", arc_radius_, 2.0);
        nh_private_.param("num_laps", num_laps_, 8);
        nh_private_.param("num_arc_points", num_arc_points_, 5);
        nh_private_.param("num_linear_points", num_linear_points_, 4);
        nh_private_.param("command", command_, true);
        nh_private_.param("distance", distance_, 1.0);
        nh_private_.param("launch_position_x", launch_pos_.x(), -5.0);
        nh_private_.param("launch_position_y", launch_pos_.y(), 0.0);
        nh_private_.param("launch_position_z", launch_pos_.z(), 0.0);
        nh_private_.param("left_pylon_x", left_pylon_.x(), 0.0);
        nh_private_.param("left_pylon_y", left_pylon_.y(), 8.66);
        nh_private_.param("left_pylon_z", left_pylon_.z(), 1.0);
        nh_private_.param("right_pylon.x", right_pylon_.x(), 0.0);
        nh_private_.param("right_pylon.y", right_pylon_.y(), 408.66);
        nh_private_.param("right_pylon.z", right_pylon_.z(), 1.0);
        nh_private_.param("hunter_killer.x", hunter_killer_.x(), 5.0);
        nh_private_.param("hunter_killer.y", hunter_killer_.y(), 1.505);
        nh_private_.param("hunter_killer.z", hunter_killer_.z(), 1.0);
    }

    void DubinsTrajectory::computeTangencyPoints() {

        // Declaring these values just to copy some other values in order to make the expression of tangency_angle_ (calculated below) shorter.
        double start_x = launch_pos_.x();
        double left_y = left_pylon_.y();
        double r = arc_radius_;

        //I got the expression below just by solving a quadratic equation in cos(theta) while calculating distance between loop_entry_point_ and launch_pos_ using coordinate geometry.
        //The expression was as below (0.0 has been used at some places just because launch position is on x axis and left_pylon_ is on the y axis.):
        //(start_x-0.0)*(start_x-0.0) + (left_y - 0.0)*(left_y - 0.0) - r*r = (r*sin(tangency_angle_) + a)*(r*sin(tangency_angle_) + a) + (left_y - r*cos(tangency_angle_))*(left_y - r*cos(tangency_angle_)).
        tangency_angle_ = acos((left_y*r + sqrt(pow(start_x,4) + start_x*start_x*left_y*left_y - r*r*start_x*start_x))/(left_y*left_y + start_x*start_x)); 

        loop_entry_point_.position.x() = arc_radius_ * sin(tangency_angle_);
        loop_entry_point_.position.y() = left_pylon_.y() - arc_radius_ * cos(tangency_angle_);
        loop_entry_point_.position.z() = left_pylon_.z();
        loop_entry_point_.yaw = M_PI/2 - tangency_angle_;

        //Position of loop_exit_point_ is computed assuming that the position of hunter_killer_ is mirror image of the launch position along the y axis.
        loop_exit_point_.position.x() = (-1) * loop_entry_point_.position.x();
        loop_exit_point_.position.y() = left_pylon_.y() - arc_radius_ * cos(tangency_angle_);
        loop_exit_point_.position.z() = left_pylon_.z();
        loop_exit_point_.yaw = tangency_angle_;

    }

    void DubinsTrajectory::computeFirstHalfLoop(uint lap_number) {

        Point start(left_pylon_.x(), left_pylon_.y() - arc_radius_, left_pylon_.z(), M_PI);
        Point end(right_pylon_.x(), right_pylon_.y() + arc_radius_, right_pylon_.z(), 0.0);

        // lap_number has been used to incorporate changes as the first and last loops are different from the rest ones due to the points of tangency.
        if(lap_number==1) {
            start = loop_entry_point_;
            arc_angle_ = loop_entry_point_.yaw;
        }
        else {
            tangency_angle_ = 0.0;
        }

        Point prev_pos(start.position, start.yaw);
        Point curr_pos(start.position, start.yaw);
        if(lap_number==1) {
            prev_pos.yaw = start.yaw + M_PI/2;
            curr_pos.yaw = start.yaw + M_PI/2;
        }
        double angle_to_move = M_PI - tangency_angle_ - arc_angle_;   // Determines how much angle still needs to be turned before the current half loop ends.

        mav_trajectory_generation::Vertex prev(3), curr(3);

        while(prev_pos.yaw >= angle_to_move) {
            curr_pos.position.y()=left_pylon_.y() + arc_radius_ * cos(prev_pos.yaw);
            curr_pos.position.x()=left_pylon_.x() + arc_radius_ * sin(prev_pos.yaw);
            curr_pos.position.z()=start.position.z();
            curr_pos.yaw = prev_pos.yaw - delta_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position.x(), curr_pos.position.y(), curr_pos.position.z()));
            vertices_.push_back(curr);
        }

        double distance = 0.0;

        while(distance <= interpylon_distance_) {
            curr_pos.position.y() = prev_pos.position.y() + delta_distance_ * sin(angle_to_move);
            curr_pos.position.x() = prev_pos.position.x() + delta_distance_ * cos(angle_to_move);
            curr_pos.position.z()=start.position.z();
            distance = distance + delta_distance_;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position.x(), curr_pos.position.y(), curr_pos.position.z()));
            vertices_.push_back(curr);
        }

        prev_pos.yaw = angle_to_move - delta_angle_;
        curr_pos.yaw = angle_to_move - delta_angle_;

        while(prev_pos.yaw >= end.yaw) {
            curr_pos.position.y()=right_pylon_.y() + arc_radius_ * cos(prev_pos.yaw);
            curr_pos.position.x()=right_pylon_.x() + arc_radius_ * sin(prev_pos.yaw);
            curr_pos.position.z()=start.position.z();
            curr_pos.yaw = prev_pos.yaw - delta_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position.x(), curr_pos.position.y(), curr_pos.position.z()));
            vertices_.push_back(curr);
        }
    }

    void DubinsTrajectory::computeSecondHalfLoop(uint lap_number) {

        Point start(right_pylon_.x(), right_pylon_.y() + arc_radius_, right_pylon_.z(), M_PI);
        Point end(left_pylon_.x(), left_pylon_.y() - arc_radius_, left_pylon_.z(), 0.0);

        // lap_number has been used to incorporate changes as the first and last loops are different from the rest ones due to the points of tangency.
        if(lap_number==num_laps_) {
            end = loop_exit_point_;
        }

        tangency_angle_ = 0.0;          // Re-calculated just because when the lap_number is 1 this value is changed in the function computeFirstHalfLoop.
        arc_angle_ = M_PI/2 - atan((right_pylon_.x() - 
                                left_pylon_.x())/(right_pylon_.y() - left_pylon_.y()));  // Re-calculated just because when the lap_number is 1 this value is changed in the function computeFirstHalfLoop.

        Point prev_pos(start.position, start.yaw);
        Point curr_pos(start.position, start.yaw);

        double angle_to_move = M_PI - tangency_angle_ - arc_angle_;
        mav_trajectory_generation::Vertex prev(3), curr(3);

        while(prev_pos.yaw >= angle_to_move) {
            curr_pos.position.y()=right_pylon_.y() - arc_radius_ * cos(prev_pos.yaw);
            curr_pos.position.x()=right_pylon_.x() - arc_radius_ * sin(prev_pos.yaw);
            curr_pos.position.z()=start.position.z();
            curr_pos.yaw = prev_pos.yaw - delta_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position.x(), curr_pos.position.y(), curr_pos.position.z()));
            vertices_.push_back(curr);
        }

        double distance = 0.0;

        while(distance <= interpylon_distance_) {
            curr_pos.position.y() = prev_pos.position.y() - delta_distance_ * sin(angle_to_move);
            curr_pos.position.x() = prev_pos.position.x() + delta_distance_ * cos(angle_to_move);
            curr_pos.position.z() = start.position.z();
            distance = distance + delta_distance_;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position.x(), curr_pos.position.y(), curr_pos.position.z()));
            vertices_.push_back(curr);
        }

        prev_pos.yaw = angle_to_move - delta_angle_;
        curr_pos.yaw = angle_to_move - delta_angle_;

        while(prev_pos.yaw >= end.yaw) {
            curr_pos.position.y() = left_pylon_.y() - arc_radius_ * cos(prev_pos.yaw);
            curr_pos.position.x() = left_pylon_.x() - arc_radius_ * sin(prev_pos.yaw);
            curr_pos.position.z() = start.position.z();
            curr_pos.yaw = prev_pos.yaw - delta_angle_ ;
            prev_pos = curr_pos;

            curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(curr_pos.position.x(), curr_pos.position.y(), curr_pos.position.z()));
            vertices_.push_back(curr);
        }

    }

    void DubinsTrajectory::computePoints() {
        ROS_INFO("Calculating waypoints for the victorious forward journey"); //Please don't remove this line.Thanks!! :)

        mav_trajectory_generation::Vertex start(3), end(3);        
        derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;

        start.makeStartOrEnd(Eigen::Vector3d(launch_pos_.x(), launch_pos_.y(), launch_pos_.z()), derivative_to_optimize_);
        start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                            initial_vel_ * (loop_entry_point_.position - launch_pos_).normalized());
        end.makeStartOrEnd(loop_entry_point_.position, derivative_to_optimize_);

        vertices_.push_back(start);
        vertices_.push_back(end);

        for(uint i=1;i<=num_laps_;i++) {
            ROS_INFO("i= %d",i);
            ROS_INFO("size of vertices_: %d \n", vertices_.size());
            computeFirstHalfLoop(i);
            computeSecondHalfLoop(i);
        }

        mav_trajectory_generation::Vertex hunter_killer(3);   
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

        mav_trajectory_generation::PolynomialOptimization<10> opt(3);
        opt.setupFromVertices(vertices_, segment_times_, derivative_to_optimize_);
        opt.solveLinear();
        opt.getTrajectory(&trajectory_);

        std::string frame_id = "world";
        mav_trajectory_generation::drawMavTrajectory(trajectory_, distance_, frame_id, &markers_);

    }

    bool DubinsTrajectory::commandServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {

        mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
        trajectory_msgs::MultiDOFJointTrajectory generated_trajectory;

        mav_trajectory_generation::sampleWholeTrajectory(trajectory_, 0.1, &trajectory_points);
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &generated_trajectory);

        if (command_) {
            trajectory_pub_.publish(generated_trajectory);
        }

        resp.success = true;
        resp.message = "Trajectory given as command";
        ROS_INFO("%s\n",resp.message.c_str());
        return true;
    }

    void DubinsTrajectory::run() {

         if (visualize_) {
            marker_pub_.publish(markers_);
        }       
    }

} //namespace ariitk::trajectory_generation
