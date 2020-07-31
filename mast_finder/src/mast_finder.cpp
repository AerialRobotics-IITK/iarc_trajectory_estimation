#include <mast_finder/mast_finder.hpp>

namespace iarc2020::mast_locator {

void MastLocatorNode::init(ros::NodeHandle& nh) {
    // odom_sub_ = nh.subscribe("firefly/ground_truth/odometry", 10, &MastLocatorNode::odomCallback, this);

    ros::NodeHandle nh_private("~");

    nh_private.getParam("ship_centre_x", ship_centre_.x);
    nh_private.getParam("ship_centre_y", ship_centre_.y);
    nh_private.getParam("ship_centre_z", ship_centre_.z);
    nh_private.getParam("radius", radius_);
    nh_private.getParam("n_sides", n_sides_);

    setpoint_pub_ = nh_private.advertise<mast_finder::Setpoint>("scouting_setpoint", 10);

    setpoint_.x = 0.0;
    setpoint_.y = 0.0;
    setpoint_.z = 3.0;

    locate_.setSetpoint(setpoint_);
    locate_.setShipcentre(ship_centre_);
    locate_.setRadius(radius_);
    locate_.setNsides(n_sides_);
}

void MastLocatorNode::run() {
    bool scouting_done = locate_.getScoutingdone();

    if (scouting_done != true) {
        locate_.updateSetpoint();
        setpoint_ = locate_.getSetpoint();
        setpoint_pub_.publish(setpoint_);
        std::cout << setpoint_.x << ' ' << setpoint_.y << ' ' << setpoint_.z << '\n';
    }
}

void MastLocatorNode::odomCallback(const nav_msgs::Odometry& msg) {
    odom_ = msg;
}

}  // namespace iarc2020::mast_locator
