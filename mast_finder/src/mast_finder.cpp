#include <mast_finder/mast_finder.hpp>
#define sq(x) (x) * (x)

namespace ariitk::mast_locator {

void MastLocatorNode::init(ros::NodeHandle& nh) {
    odom_sub_ = nh.subscribe("odom", 10, &MastLocatorNode::odomCallback, this);
    front_coord_sub_ = nh.subscribe("plate_pose_estimator_node/front_coord", 10, &MastLocatorNode::frontCallback, this);
    pose_sub_ = nh.subscribe("plate_pose_estimator_node/estimated_coord", 10, &MastLocatorNode::poseCallback, this);
    centre_sub_ = nh.subscribe("plate_detector_node/centre_coord", 10, &MastLocatorNode::centreCallback, this);
    plate_front_vec_sub_ = nh.subscribe("plate_pose_estimator_node/plate_front_vec", 10, &MastLocatorNode::plateFrontVecCallback, this);
    yaw_correction_sub_ = nh.subscribe("plate_pose_estimator_node/yaw_correction", 10, &MastLocatorNode::yawCorrectionCallback, this);

    ros::NodeHandle nh_private("~");

    nh_private.getParam("ship_centre_x", ship_centre_.x);
    nh_private.getParam("ship_centre_y", ship_centre_.y);
    nh_private.getParam("ship_centre_z", ship_centre_.z);
    nh_private.getParam("radius", radius_);
    nh_private.getParam("n_sides", n_sides_);
    nh_private.getParam("transition_rate", transition_rate_);
    nh_private.getParam("max_velocity", v_max_);
    nh_private.getParam("max_accleration", a_max_);
    nh_private.getParam("min_distance", min_distance_);

    setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 10);
    traj_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("trajectory", 10);

    setpoint_.x = 0.0;
    setpoint_.y = 0.0;
    setpoint_.z = 1.5;
    yaw_change_ = 0;
    scouting_done_ = false;

    locate_.setSetpoint(setpoint_);
    locate_.setShipcentre(ship_centre_);
    locate_.setRadius(radius_);
    locate_.setNsides(n_sides_);

    ros::Rate rate(0.75);
    rate.sleep();
}

void MastLocatorNode::run() {
    if (scouting_done_ == true) {
        // auto temp = system("rosnode kill mast_locator_node");  //* Uncomment this if you want to kill the node
        return;
    }

    isMastDetected();

    if (scouting_done_ == true) {
        ros::Rate rate(0.75);
        for (int i = 0; i <= 4; i++) {
            rate.sleep();
            ros::spinOnce();
        }

        correctYaw();
        std::cout << "Corrected " << yaw_correction_.z * 360 / M_PI << " degrees" << std::endl << std::endl;

        for (double j = min_distance_ + 2; j > 0; j--) {
            for (int i = 0; i <= 1; i++) {
                rate.sleep();
                ros::spinOnce();
            }
            std::cout << "Beep " << j << std::endl;
            goNearMast(j);
        }
    }

    if (traj_published_ == true) {
        return;
    }

    planTrajectory();
    getTrajectory();
    publishTrajectory();
}

void MastLocatorNode::planTrajectory() {
    sides_done_ = locate_.getSidesDone();
    float yaw = 0;

    while (sides_done_ < n_sides_) {
        sides_done_ = locate_.getSidesDone();
        locate_.updateSetpoint();
        setpoint_ = locate_.getSetpoint();

        mav_trajectory_generation::Vertex traj_vertex(4);

        traj_point_(0) = setpoint_.x;
        traj_point_(1) = setpoint_.y;
        traj_point_(2) = setpoint_.z;
        traj_point_(3) = yaw;

        std::cout << "Trajectory Setpoint = " << setpoint_.x << ' ' << setpoint_.y << ' ' << setpoint_.z << ' ' << yaw << '\n';

        traj_vertex.makeStartOrEnd(traj_point_, derivative_to_optimize);
        vertices_.push_back(traj_vertex);

        yaw += (2 * M_PI) / n_sides_;
    }
}

void MastLocatorNode::getTrajectory() {
    segment_times_ = mav_trajectory_generation::estimateSegmentTimes(vertices_, v_max_, a_max_);

    mav_trajectory_generation::PolynomialOptimizationNonLinear<10> opt(4, parameters_);
    opt.setupFromVertices(vertices_, segment_times_, derivative_to_optimize);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max_);
    opt.optimize();
    opt.getTrajectory(&traj_);

    mav_msgs::EigenTrajectoryPoint::Vector traj_points;
    mav_trajectory_generation::sampleWholeTrajectory(traj_, 0.1, &traj_points);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(traj_points, &generated_traj_);
}

void MastLocatorNode::publishTrajectory() {
    if (traj_published_ != true) {
        traj_pub_.publish(generated_traj_);
        traj_published_ = true;
    }
}

void MastLocatorNode::isMastDetected() {
    ros::Rate rate(transition_rate_);

    if (centre_coord_.x != -1 || centre_coord_.y != -1) {
        scouting_done_ = true;
        ros::spinOnce();

        next_setpt_.pose.position.x = odom_.pose.pose.position.x;
        next_setpt_.pose.position.y = odom_.pose.pose.position.y;
        next_setpt_.pose.position.z = odom_.pose.pose.position.z;
        next_setpt_.pose.orientation = odom_.pose.pose.orientation;
        setpoint_pub_.publish(next_setpt_);

        for (int j = 0; j < 30; j++) {  //* wait for quad to stabilize after initial detecton to get proper pose
            rate.sleep();
        }
        ros::spinOnce();
        std::cout << '\n' << "Centre at " << centre_coord_.x << ' ' << centre_coord_.y << '\n';
        ifMastDetected();
        return;
    }
}

void MastLocatorNode::ifMastDetected() {
    next_setpt_.pose.position.x = odom_.pose.pose.position.x;
    next_setpt_.pose.position.y = odom_.pose.pose.position.y;
    next_setpt_.pose.position.z = odom_.pose.pose.position.z;

    yaw_change_ = tf::getYaw(odom_.pose.pose.orientation);

    double v1x, v1y, v2x, v2y;
    v1x = front_coord_.x - odom_.pose.pose.position.x;  //* Vector poining in front of the drone
    v1y = front_coord_.y - odom_.pose.pose.position.y;
    v2x = pose_.x - odom_.pose.pose.position.x;  //* Vector pointing towards estimated plate centre
    v2y = pose_.y - odom_.pose.pose.position.y;
    double mod_v1 = sqrt(sq(v1x) + sq(v1y));
    double mod_v2 = sqrt(sq(v2x) + sq(v2y));

    if (mod_v1 == 0) {
        mod_v1 = 1;
    }
    if (mod_v2 == 0) {
        mod_v2 = 1;
    }

    double crossp = ((v1x * v2y) - (v1y * v2x)) / (mod_v1 * mod_v2);

    if (crossp < 0) {
        yaw_change_ += asin(crossp);
    } else {
        yaw_change_ -= asin(crossp);
    }

    std::cout << '\n' << "Plate Pose = " << pose_.x << ' ' << pose_.y << ' ' << pose_.z << std::endl << std::endl;
    next_setpt_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_change_);
    setpoint_pub_.publish(next_setpt_);
}

void MastLocatorNode::correctYaw() {
    next_setpt_.pose.position.x = odom_.pose.pose.position.x;
    next_setpt_.pose.position.y = odom_.pose.pose.position.y;
    next_setpt_.pose.position.z = odom_.pose.pose.position.z;

    if (yaw_correction_.z < 0) {
        yaw_change_ -= yaw_correction_.z;
    } else {
        yaw_change_ += yaw_correction_.z;
    }

    next_setpt_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_change_);
    setpoint_pub_.publish(next_setpt_);
}

void MastLocatorNode::goNearMast(float dist) {
    correctYaw();
    std::cout << "Corrected " << yaw_correction_.z * 360 / M_PI << " degrees" << std::endl << std::endl;
    Eigen::Vector3d v, w;
    w(0) = plate_front_vec_.x;
    w(1) = plate_front_vec_.y;
    w(2) = plate_front_vec_.z;
    v = w / sqrt(w.dot(w));

    next_setpt_.pose.position.x = pose_.x + (dist * v(0));
    next_setpt_.pose.position.y = pose_.y + (dist * v(1));
    next_setpt_.pose.position.z = pose_.z + (dist * v(2));
    next_setpt_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_change_);
    setpoint_pub_.publish(next_setpt_);

    // std::cout << "[" << next_setpt_.pose.position.x
    //           << " " << next_setpt_.pose.position.y
    //           << " " << next_setpt_.pose.position.z
    //           << "]" << std::endl << std::endl;
}

void MastLocatorNode::odomCallback(const nav_msgs::Odometry& msg) {
    odom_ = msg;
}

void MastLocatorNode::frontCallback(const detector_msgs::GlobalCoord& msg) {
    front_coord_ = msg;
}

void MastLocatorNode::centreCallback(const detector_msgs::Centre& msg) {
    centre_coord_ = msg;
}

void MastLocatorNode::poseCallback(const detector_msgs::GlobalCoord& msg) {
    pose_ = msg;
}

void MastLocatorNode::yawCorrectionCallback(const detector_msgs::GlobalCoord& msg) {
    yaw_correction_ = msg;
}

void MastLocatorNode::plateFrontVecCallback(const detector_msgs::GlobalCoord& msg) {
    plate_front_vec_ = msg;
}

}  // namespace ariitk::mast_locator
