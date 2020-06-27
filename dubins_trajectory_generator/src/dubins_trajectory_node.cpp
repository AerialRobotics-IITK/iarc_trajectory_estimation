#include <dubins_trajectory_generator/dubins_trajectory.hpp>

using namespace ariitk::trajectory_generation;

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    DubinsTrajectory trajectory(nh, nh_private);
    int rate;
    nh_private.param("rate",rate,10);

    ros::Rate loop_rate(rate);

    while (ros::ok()) {
        trajectory.run();
        loop_rate.sleep();
    }

    return 0;

}