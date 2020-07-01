#include <iarc_trajectory_estimation/trajectory_generation.hpp>

namespace ariitk::trajectory_generation {

PolynomialTrajectoryGeneration::PolynomialTrajectoryGeneration(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh)
    , nh_private_(nh_private) {

    mav_odom_sub_ = nh_.subscribe("ground_truth/odometry", 10, &PolynomialTrajectoryGeneration::mavOdometryCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);
    server_ = nh_.advertiseService("command", &PolynomialTrajectoryGeneration::commandServiceCallback, this);
    client_ = nh_.serviceClient<std_srvs::Trigger>("command");

    nh_private_.param("dimension", dimension_, 3);
    nh_private_.param("visualize", visualize_, true);
    nh_private_.param("distance", distance_, 1.0);
    nh_private_.param("v_max", v_max_, 2.0);
    nh_private_.param("a_max", a_max_, 2.0);
    nh_private_.param("publish_mav_odometry", publish_, true);
    nh_private_.param("command", command_, true);

    PolynomialTrajectoryGeneration::computePoints();
    PolynomialTrajectoryGeneration::generateTrajectory();
}

void PolynomialTrajectoryGeneration::mavOdometryCallback(const nav_msgs::Odometry& msg) {
    mav_odom_ = msg;
}

void PolynomialTrajectoryGeneration::computePoints() {

    mav_trajectory_generation::Vertex start(dimension_), end(dimension_);
    derivative_to_optimize_ = mav_trajectory_generation::derivative_order::VELOCITY;

    start.makeStartOrEnd(Eigen::Vector3d(-5, 0, 2), derivative_to_optimize_);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(1, 1, 1));
    end.makeStartOrEnd(Eigen::Vector3d(-2, 8.66, 2), derivative_to_optimize_);

    vertices_.push_back(start);
    vertices_.push_back(end);
}

void PolynomialTrajectoryGeneration::generateTrajectory() {

    segment_times_ = mav_trajectory_generation::estimateSegmentTimes(vertices_, v_max_, a_max_);

    mav_trajectory_generation::PolynomialOptimization<10> opt(dimension_);
    opt.setupFromVertices(vertices_, segment_times_, derivative_to_optimize_);
    opt.solveLinear();
    opt.getTrajectory(&trajectory_);

    std::string frame_id = "world";
    mav_trajectory_generation::drawMavTrajectory(trajectory_, distance_, frame_id, &markers_);
}

bool PolynomialTrajectoryGeneration::commandServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {

    mav_trajectory_generation::sampleWholeTrajectory(trajectory_, 0.1, &trajectory_points_);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points_, &generated_trajectory_);

    if (command_) {
        trajectory_pub_.publish(generated_trajectory_);
    }

    resp.success = true;
    resp.message = "Trajectory given as command";
    ROS_INFO("%s\n", resp.message.c_str());
    return true;
}

void PolynomialTrajectoryGeneration::run() {
    
    if (visualize_) {
        marker_pub_.publish(markers_);
    }
}
}  // namespace ariitk::trajectory_generation