#include <mast_finder/mast_finder.hpp>

namespace iarc2020::mast_locator {

void MastLocatorNode::init(ros::NodeHandle& nh) {
    odom_sub_ = nh.subscribe("odom", 10, &MastLocatorNode::odomCallback, this);

    ros::NodeHandle nh_private("~");

    nh_private.getParam("ship_centre_x", ship_centre_.x);
    nh_private.getParam("ship_centre_y", ship_centre_.y);
    nh_private.getParam("ship_centre_z", ship_centre_.z);
    nh_private.getParam("radius", radius_);
    nh_private.getParam("n_sides", n_sides_);
    nh_private.getParam("transition_rate", transition_rate_);

    setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);

    setpoint_.x = 0.0;
    setpoint_.y = 0.0;
    setpoint_.z = 2.0;

    locate_.setSetpoint(setpoint_);
    locate_.setShipcentre(ship_centre_);
    locate_.setRadius(radius_);
    locate_.setNsides(n_sides_);

    ros::Rate rate(0.75);
    rate.sleep();
}

void MastLocatorNode::run() {
    publishSetpoint();
}

void MastLocatorNode::publishSetpoint() {
    ros::Rate rate(transition_rate_);
    bool scouting_done = locate_.getScoutingdone();

    if (scouting_done != true) {
        locate_.updateSetpoint();
        setpoint_ = locate_.getSetpoint();
        std::cout << setpoint_.x << ' ' << setpoint_.y << '\n';
    }

    next_setpt_.pose.position.x = setpoint_.x;
    next_setpt_.pose.position.y = setpoint_.y;
    next_setpt_.pose.position.z = setpoint_.z;

    setpoint_pub_.publish(next_setpt_);
    rate.sleep();
}

void MastLocatorNode::odomCallback(const nav_msgs::Odometry& msg) {
    odom_ = msg;
}

}  // namespace iarc2020::mast_locator
