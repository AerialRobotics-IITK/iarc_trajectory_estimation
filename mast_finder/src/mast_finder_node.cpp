#include <mast_finder/mast_finder.hpp>

using namespace iarc2020::mast_locator;

int main(int argc, char** argv) {
    ros::init(argc, argv, "mast_locator_node");
    ros::NodeHandle nh;

    MastLocatorNode locate;

    locate.init(nh);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        ros::spinOnce();
        locate.run();
        loop_rate.sleep();
    }

    return 0;
}
