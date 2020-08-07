#include <mast_finder/mast_finder.hpp>
#define sq(x) (x) * (x)

namespace iarc2020::mast_locator {

void MastLocatorNode::init(ros::NodeHandle& nh) {
    odom_sub_ = nh.subscribe("odom", 10, &MastLocatorNode::odomCallback, this);
    front_coord_sub_ = nh.subscribe("front_coord", 10, &MastLocatorNode::frontCallback, this);
    centre_sub_ = nh.subscribe("centre_coord", 10, &MastLocatorNode::centreCallback, this);

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
    if (centre_coord_.x != -1 || centre_coord_.y != -1) {
        scouting_done_ = true;
        std::cout << "Centre at " << centre_coord_.x << ' ' << centre_coord_.y << '\n';
        auto temp = system("rosnode kill mast_locator_node");
        return;
    }
    publishYaw();
    publishSetpoint();
    publishMsg();
}

void MastLocatorNode::publishSetpoint() {
    sides_done_ = locate_.getSidesdone();

    if (sides_done_ >= n_sides_ + 1) {
        return;
    }

    locate_.updateSetpoint();
    setpoint_ = locate_.getSetpoint();
    std::cout << "Current Setpoint = " << setpoint_.x << ' ' << setpoint_.y << '\n';

    next_setpt_.pose.position.x = setpoint_.x;
    next_setpt_.pose.position.y = setpoint_.y;
    next_setpt_.pose.position.z = setpoint_.z;
}

void MastLocatorNode::publishYaw() {
    if (sides_done_ >= n_sides_ - 1) {
        return;
    }

    double v1x, v1y, v2x, v2y;

    v1x = front_coord_.x - odom_.pose.pose.position.x;  //* Vector poining in front of the drone
    v1y = front_coord_.y - odom_.pose.pose.position.y;

    v2x = setpoint_.x - odom_.pose.pose.position.x;     //* Vector pointing toards ship centre
    v2y = setpoint_.y - odom_.pose.pose.position.y;

    double mod_v1 = sqrt(sq(v1x) + sq(v1y));
    double mod_v2 = sqrt(sq(v2x) + sq(v2y));

    if (mod_v1 == 0) {
        mod_v1 = 1;
    }

    if (mod_v2 == 0) {
        mod_v2 = 1;
    }

    double crossp = ((v1x * v2y) - (v1y * v2x)) / (mod_v1 * mod_v2);

    yaw_change_ -= asin(crossp);

    next_setpt_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_change_);

    std::cout << "Current Yaw = " << yaw_change_ << "\n";
}

void MastLocatorNode::publishMsg() {
    ros::Rate rate(transition_rate_);
    setpoint_pub_.publish(next_setpt_);
    rate.sleep();
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

}  // namespace iarc2020::mast_locator
