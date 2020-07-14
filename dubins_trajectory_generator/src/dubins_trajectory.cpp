#include <dubins_trajectory_generator/dubins_trajectory.hpp>

const double g = 9.8;
namespace ariitk::trajectory_generation {

DubinsTrajectory::DubinsTrajectory(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    starting_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    return_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);
    starting_trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);
    return_trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);

    publish_trajectory_server_ = nh.advertiseService("command", &DubinsTrajectory::commandServiceCallback, this);
    publish_starting_traj_srv_ = nh.advertiseService("starting_command", &DubinsTrajectory::publishStartingTrajectory, this);
    publish_return_traj_srv_ = nh.advertiseService("return_command", &DubinsTrajectory::publishReturnTrajectory, this);
    publish_trajectory_client_ = nh.serviceClient<std_srvs::Trigger>("command");
    publish_starting_traj_client_ = nh.serviceClient<std_srvs::Trigger>("starting_command");
    publish_return_traj_client_ = nh.serviceClient<std_srvs::Trigger>("return_command");

    DubinsTrajectory::loadParams(nh_private);

    arc_angle_ = M_PI / 2 - atan((right_pylon_.x() - left_pylon_.x()) / (right_pylon_.y() - left_pylon_.y()));  // Determines the angle our MAV turns
                                                                                                                // in one single curved part.

    interpylon_distance_ = (left_pylon_ - right_pylon_).norm();  // Distance between the two pylons

    delta_angle_ = (arc_angle_) / num_arc_points_;
    delta_distance_ = (interpylon_distance_) / num_linear_points_;

    /*
      We can simply derive the formula used below by applying Newton's third law on the MAV while it's turning around one of the pylons.
      The two forces that we consider acting on the MAV while deriving it are gravitational force and centrifugal force.
      We can parameterize the value of arc_radius_ or turning_vel_ depending on the limitations of the MAV and accordingly calculate the other one.
    */
    turning_vel_ = sqrt(arc_radius_ * g * tan(max_roll_angle_)); 
    DubinsTrajectory::computeTangencyPoints();
    DubinsTrajectory::computePoints();
    DubinsTrajectory::generateTrajectory();
}

void DubinsTrajectory::loadParams(ros::NodeHandle& nh_private) {
    nh_private.param("visualize", visualize_, true);
    nh_private.param("command", command_, true);
    nh_private.param("starting_command", starting_command_, true);
    nh_private.param("return_command", return_command_, true);

    nh_private.param("distance", distance_, 1.0);
    nh_private.param("initial_vel", initial_vel_, 1.0);
    nh_private.param("v_max", v_max_, 2.0);
    nh_private.param("a_max", a_max_, 2.0);

    int num_laps;
    nh_private.param("num_laps", num_laps, 8);  // can't take uint params directly
    num_laps_ = num_laps;

    nh_private.param("arc_radius", arc_radius_, 2.0);
    nh_private.param("num_arc_points", num_arc_points_, 5);
    nh_private.param("num_linear_points", num_linear_points_, 4);
    nh_private.param("max_roll_angle", max_roll_angle_, 0.2);

    nh_private.param("launch_position_x", launch_pos_.x(), -5.0);
    nh_private.param("launch_position_y", launch_pos_.y(), 0.0);
    nh_private.param("launch_position_z", launch_pos_.z(), 0.0);
    nh_private.param("hunter_killer_x", hunter_killer_.x(), 5.0);
    nh_private.param("hunter_killer_y", hunter_killer_.y(), 1.505);
    nh_private.param("hunter_killer_z", hunter_killer_.z(), 1.0);

    nh_private.param("left_pylon_x", left_pylon_.x(), 0.0);
    nh_private.param("left_pylon_y", left_pylon_.y(), 8.66);
    nh_private.param("left_pylon_z", left_pylon_.z(), 1.0);
    nh_private.param("right_pylon_x", right_pylon_.x(), 0.0);
    nh_private.param("right_pylon_y", right_pylon_.y(), 408.66);
    nh_private.param("right_pylon_z", right_pylon_.z(), 1.0);
}

void DubinsTrajectory::computeTangencyPoints() {
    // Declaring these values just to copy some other values in order to make the
    // expression of tangency_angle_ (calculated below) shorter.
    double start_x = launch_pos_.x();
    double left_y = left_pylon_.y();
    double r = arc_radius_;

    /*
      tangency_angle_ is the angle the point of tangency makes with the centre of the arc with the y-axis as the base.
      The launch position, centre of the arc and point of tangency form a right angled triangle.
      Launch pos: (-start_x, 0); Centre of the arc is the left pylon (0, left_y)
      Coordinates of the point of tangency hence: (rsin(x), left_y - rcos_x), x is the tangency angle
      Distance between the launch position and point of tangency is the length of the hypotenuse, using this we solve for x as below:
    */
    double l = sqrt(start_x * start_x + left_y * left_y);  // launch position to left pylon distance
    double zen_angle = acos(left_y / l);                   // angle between y-axis and line joining launch and pylon
    tangency_angle_ = acos(r / l) - zen_angle;

    loop_entry_point_ = Point(r * sin(tangency_angle_), left_y - r * cos(tangency_angle_), left_pylon_.z(), M_PI / 2 - tangency_angle_);

    // Position of loop_exit_point_ is computed assuming that the position of
    // hunter_killer_ is mirror image of the launch position along the y axis.
    loop_exit_point_ = Point(-r * sin(tangency_angle_), left_y - r * cos(tangency_angle_), left_pylon_.z(), tangency_angle_);
}

void DubinsTrajectory::computeFirstHalfLoop(const uint& lap_number) {
    Point start(left_pylon_.x(), left_pylon_.y() - arc_radius_, left_pylon_.z(), M_PI);
    Point end(right_pylon_.x(), right_pylon_.y() + arc_radius_, right_pylon_.z(), 0.0);

    // lap_number has been used to incorporate changes as the first and last loops
    // are different from the rest ones due to the points of tangency.
    if (lap_number == 1) {
        start = loop_entry_point_;
        arc_angle_ = loop_entry_point_.yaw;
    } else {
        tangency_angle_ = 0.0;
    }

    Point prev_pos(start.position, start.yaw);
    Point curr_pos(start.position, start.yaw);
    if (lap_number == 1) {
        prev_pos.yaw = start.yaw + M_PI / 2;
        curr_pos.yaw = start.yaw + M_PI / 2;
    }
    double angle_to_move = M_PI - tangency_angle_ - arc_angle_;  // Determines how much angle still needs to be turned before
                                                                 // the current half loop ends.

    mav_trajectory_generation::Vertex prev(3), curr(3);

    while (prev_pos.yaw >= angle_to_move) {
        curr_pos = Point(left_pylon_.x() + arc_radius_ * sin(prev_pos.yaw),
            left_pylon_.y() + arc_radius_ * cos(prev_pos.yaw),
            start.position.z(),
            prev_pos.yaw - delta_angle_);

        Eigen::Vector3d turning_vel(turning_vel_ * sin(prev_pos.yaw - M_PI / 2), turning_vel_ * cos(prev_pos.yaw - M_PI / 2), 0.0); // Using simple geometry and trigonometry and assuming no velocity in the z direction.
        curr.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, turning_vel);
        curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, curr_pos.position);    
        prev_pos = curr_pos;

        vertices_.push_back(curr);
    }

    double distance = 0.0;

    while (distance <= interpylon_distance_) {
        curr_pos = Point(prev_pos.position.x() + delta_distance_ * cos(angle_to_move),
            prev_pos.position.y() + delta_distance_ * sin(angle_to_move),
            start.position.z(),
            prev_pos.yaw);
        distance = distance + delta_distance_;
        prev_pos = curr_pos;

        curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, curr_pos.position);
        vertices_.push_back(curr);
    }

    prev_pos.yaw = angle_to_move - delta_angle_;
    curr_pos.yaw = angle_to_move - delta_angle_;

    while (prev_pos.yaw >= end.yaw) {
        curr_pos = Point(right_pylon_.x() + arc_radius_ * sin(prev_pos.yaw),
            right_pylon_.y() + arc_radius_ * cos(prev_pos.yaw),
            start.position.z(),
            prev_pos.yaw - delta_angle_);

        Eigen::Vector3d turning_vel(-1.0 * turning_vel_ * cos(prev_pos.yaw), turning_vel_ * sin(prev_pos.yaw), 0.0); // Using simple geometry and trigonometry and assuming no velocity in the z direction.
        curr.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, turning_vel);
        curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, curr_pos.position);
        prev_pos = curr_pos;

        vertices_.push_back(curr);
    }
}

void DubinsTrajectory::computeSecondHalfLoop(const uint& lap_number) {
    Point start(right_pylon_.x(), right_pylon_.y() + arc_radius_, right_pylon_.z(), M_PI);
    Point end(left_pylon_.x(), left_pylon_.y() - arc_radius_, left_pylon_.z(), 0.0);

    // lap_number has been used to incorporate changes as the first and last loops
    // are different from the rest ones due to the points of tangency.
    if (lap_number == num_laps_) {
        end = loop_exit_point_;
    }

    tangency_angle_ = 0.0;  // Re-calculated just because when the lap_number is 1 this value is
                            // changed in the function computeFirstHalfLoop.
    arc_angle_ = M_PI / 2 - atan((right_pylon_.x() - left_pylon_.x()) / (right_pylon_.y() - left_pylon_.y()));  // Re-calculated just because when the
                                                                                                                // lap_number is 1 this value is changed
                                                                                                                // in the function computeFirstHalfLoop.

    Point prev_pos(start.position, start.yaw);
    Point curr_pos(start.position, start.yaw);

    double angle_to_move = M_PI - tangency_angle_ - arc_angle_;
    mav_trajectory_generation::Vertex prev(3), curr(3);

    while (prev_pos.yaw >= angle_to_move) {
        curr_pos = Point(right_pylon_.x() - arc_radius_ * sin(prev_pos.yaw),
            right_pylon_.y() - arc_radius_ * cos(prev_pos.yaw),
            start.position.z(),
            prev_pos.yaw - delta_angle_);

        Eigen::Vector3d turning_vel(turning_vel_ * cos(prev_pos.yaw), (-1.0) * turning_vel_ * sin(prev_pos.yaw), 0.0); // Using simple geometry and trigonometry and assuming no velocity in the z direction.
        curr.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, turning_vel);
        curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, curr_pos.position);
        prev_pos = curr_pos;

        vertices_.push_back(curr);
    }

    double distance = 0.0;

    while (distance <= interpylon_distance_) {
        curr_pos = Point(prev_pos.position.x() + delta_distance_ * cos(angle_to_move),
            prev_pos.position.y() - delta_distance_ * sin(angle_to_move),
            start.position.z(),
            prev_pos.yaw);
        distance = distance + delta_distance_;
        prev_pos = curr_pos;

        curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, curr_pos.position);
        vertices_.push_back(curr);
    }

    prev_pos.yaw = angle_to_move - delta_angle_;
    curr_pos.yaw = angle_to_move - delta_angle_;

    while (prev_pos.yaw >= end.yaw) {
        curr_pos = Point(left_pylon_.x() - arc_radius_ * sin(prev_pos.yaw),
            left_pylon_.y() - arc_radius_ * cos(prev_pos.yaw),
            start.position.z(),
            prev_pos.yaw - delta_angle_);
        
        Eigen::Vector3d turning_vel(turning_vel_ * cos(curr_pos.yaw), (-1.0) * turning_vel_ * sin(curr_pos.yaw), 0.0); // Using simple geometry and trigonometry and assuming no velocity in the z direction.
        curr.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, turning_vel);
        curr.addConstraint(mav_trajectory_generation::derivative_order::POSITION, curr_pos.position);
        prev_pos = curr_pos;

        vertices_.push_back(curr);
    }
}

void DubinsTrajectory::computePoints() {
    ROS_INFO("Calculating waypoints for the victorious forward journey");

    mav_trajectory_generation::Vertex start(3), end(3);
    derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;

    start.makeStartOrEnd(launch_pos_, derivative_to_optimize_);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, initial_vel_ * (loop_entry_point_.position - launch_pos_).normalized());
    end.makeStartOrEnd(loop_entry_point_.position, derivative_to_optimize_);

    vertices_.push_back(start);
    vertices_.push_back(end);

    for (uint i = 1; i <= num_laps_; i++) {
        ROS_INFO("i= %d", i);
        ROS_INFO("size of vertices_: %d \n", (int) vertices_.size());
        computeFirstHalfLoop(i);
        computeSecondHalfLoop(i);
    }

    mav_trajectory_generation::Vertex hunter_killer(3);
    hunter_killer.addConstraint(mav_trajectory_generation::derivative_order::POSITION, hunter_killer_);
    vertices_.push_back(hunter_killer);

    ROS_INFO("Reversing waypoints for the victorious return journey");
    reverse_vertices_ = vertices_;
    starting_vertices_ = vertices_;
    std::reverse(reverse_vertices_.begin(), reverse_vertices_.end());
    vertices_.insert(vertices_.end(), ++reverse_vertices_.begin(), reverse_vertices_.end());  // For proper segment_times.
    auto it = reverse_vertices_.begin();
    reverse_vertices_.erase(it);
}

void DubinsTrajectory::generateTrajectory() {
    //Calculating the complete trajectory including both starting and return journeys.
    std::vector<double> segment_times;
    segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices_, v_max_, a_max_);

    mav_trajectory_generation::PolynomialOptimization<10> opt(3);
    opt.setupFromVertices(vertices_, segment_times, derivative_to_optimize_);
    opt.solveLinear();
    opt.getTrajectory(&trajectory_);

    std::string frame_id = "world";
    mav_trajectory_generation::drawMavTrajectory(trajectory_, distance_, frame_id, &markers_);

    //Calculating the starting trajectory only.
    std::vector<double> starting_segment_times;
    starting_segment_times = mav_trajectory_generation::estimateSegmentTimes(starting_vertices_, v_max_, a_max_);

    mav_trajectory_generation::PolynomialOptimization<10> starting_opt(3);
    starting_opt.setupFromVertices(starting_vertices_, starting_segment_times, derivative_to_optimize_);
    starting_opt.solveLinear();
    starting_opt.getTrajectory(&starting_trajectory_);

    mav_trajectory_generation::drawMavTrajectory(starting_trajectory_, distance_, frame_id, &starting_markers_);

    //Calculating the return trajectory only.
    std::vector<double> return_segment_times;
    return_segment_times = mav_trajectory_generation::estimateSegmentTimes(reverse_vertices_, v_max_, a_max_);

    mav_trajectory_generation::PolynomialOptimization<10> return_opt(3);
    return_opt.setupFromVertices(reverse_vertices_, return_segment_times, derivative_to_optimize_);
    return_opt.solveLinear();
    return_opt.getTrajectory(&return_trajectory_);

    mav_trajectory_generation::drawMavTrajectory(return_trajectory_, distance_, frame_id, &return_markers_);
}

bool DubinsTrajectory::commandServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    trajectory_msgs::MultiDOFJointTrajectory generated_trajectory;

    mav_trajectory_generation::sampleWholeTrajectory(trajectory_, 0.1, &trajectory_points);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &generated_trajectory);

    if (command_) {
        trajectory_pub_.publish(generated_trajectory);
    }

    resp.success = true;
    resp.message = "Command given to the MAV to follow the complete trajectory";
    ROS_INFO("%s\n", resp.message.c_str());
    return true;
}

bool DubinsTrajectory::publishStartingTrajectory(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    trajectory_msgs::MultiDOFJointTrajectory generated_trajectory;

    mav_trajectory_generation::sampleWholeTrajectory(starting_trajectory_, 0.1, &trajectory_points);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &generated_trajectory);

    if (starting_command_) {
        starting_trajectory_pub_.publish(generated_trajectory);
    }

    resp.success = true;
    resp.message = "Command given to the MAV to follow the starting trajectory";
    ROS_INFO("%s\n", resp.message.c_str());
    return true;
}

bool DubinsTrajectory::publishReturnTrajectory(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    trajectory_msgs::MultiDOFJointTrajectory generated_trajectory;

    mav_trajectory_generation::sampleWholeTrajectory(return_trajectory_, 0.1, &trajectory_points);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &generated_trajectory);

    if (return_command_) {
        return_trajectory_pub_.publish(generated_trajectory);
    }

    resp.success = true;
    resp.message = "Command given to the MAV to follow the return trajectory";
    ROS_INFO("%s\n", resp.message.c_str());
    return true;
}

void DubinsTrajectory::run() {
    if (visualize_) {
        // marker_pub_.publish(markers_);
        starting_marker_pub_.publish(starting_markers_);
        return_marker_pub_.publish(return_markers_);
    }
}

}  // namespace ariitk::trajectory_generation
