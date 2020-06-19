#pragma once

#include <ros/ros.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>


namespace ariitk::trajectory_generation {

    class trajectoryGeneratinPolynomial {
        public : 
            trajectoryGenerationPolynomial();
            ~trajectoryGenerationPolynomial() {};

        private :
            void mavOdometryCallback(const nav_msgs::Odometry& msg);
            mav_trajectory_generation::Vertex::Vector computePoints();
            void generateTrajectory(std::vector<mav_trajectory_generation::Vertex> vertices);

            double v_max_:
            double a_max_;

            int dimension_;
            int derivative_to_optimize_;

            std::vector<double> segment_times_;
            mav_trajectory_generation::Vertex start_, middle_one_, end_;
            mav_trajectory_generation::Vertex::Vector vertices_;
        
            ros::NodeHandle nh_;
            ros::NodeHandle nh_private_;

            ros::Subscriber mav_odom_sub_;
            ros::Publisher trajectory_pub_;
    };

}   // namespace ariitk::trajectory_generation