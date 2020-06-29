#include <iarc_trajectory_estimation/trajectory_generation.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    ariitk::trajectory_generation::TrajectoryGenerationPolynomial trajectory(nh, nh_private);
    int rate;
    nh_private.param("rate", rate, 10);
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        trajectory.run();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}