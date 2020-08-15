#include <mast_finder/mast_finder.hpp>
#define sq(x) (x) * (x)

namespace iarc2020::mast_locator {

void MastLocatorNode::init(ros::NodeHandle& nh) {
    odom_sub_ = nh.subscribe("odom", 10, &MastLocatorNode::odomCallback, this);
    front_coord_sub_ = nh.subscribe("front_coord", 10, &MastLocatorNode::frontCallback, this);
    pose_sub_ = nh.subscribe("estimated_coord", 10, &MastLocatorNode::poseCallback, this);
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
    if (scouting_done_ == true) {
        // auto temp = system("rosnode kill mast_locator_node");  //* Uncomment this if you want to kill the node
        return;
    }
    isMastDetected();
    if (scouting_done_ == true) {
        return;
    }
    publishYaw();
    publishSetpoint();
    publishMsg();
}

void MastLocatorNode::publishSetpoint() {
    sides_done_ = locate_.getSidesDone();

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

    v2x = setpoint_.x - odom_.pose.pose.position.x;  //* Vector pointing towards ship centre
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
    setpoint_pub_.publish(next_setpt_);
}

void MastLocatorNode::isMastDetected() {
    ros::Rate rate(transition_rate_);

    for (int i = 0; i < 50; i++) {
        ros::spinOnce();

        if (centre_coord_.x != -1 || centre_coord_.y != -1) {
            scouting_done_ = true;

            for (int j = 0; j < 30; j++) {  //* wait for quad to stabilize after initial detecton to get proper pose
                rate.sleep();
            }
            ros::spinOnce();
            std::cout << "Centre at " << centre_coord_.x << ' ' << centre_coord_.y << '\n';
            ifMastDetected();
            return;
        }
        rate.sleep();
    }
}

void MastLocatorNode::ifMastDetected() {
    next_setpt_.pose.position.x = odom_.pose.pose.position.x;
    next_setpt_.pose.position.y = odom_.pose.pose.position.y;
    next_setpt_.pose.position.z = odom_.pose.pose.position.z;

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

    std::cout << "Plate Pose = " << pose_.x << ' ' << pose_.y << "\n";

    next_setpt_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_change_);

    setpoint_pub_.publish(next_setpt_);

    // while (centre_coord_.d > 3) {    //TODO: Loop for going near the mast tbd after vel control
    //     ros::spinOnce();
    //     detector_msgs::GlobalCoord temp = pose_;
    //     next_setpt_.pose.position.x = temp.x;
    //     next_setpt_.pose.position.y = temp.y;
    //     next_setpt_.pose.position.z = odom_.pose.pose.position.z;
    //     next_setpt_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_change_);
    //     setpoint_pub_.publish(next_setpt_);
    // }
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

}  // namespace iarc2020::mast_locator
