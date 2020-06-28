#include <dubins_trajectory_generator/dubins_trajectory.hpp>

const double PI=3.14159265358979323846;
namespace ariitk::trajectory_generation {

    DubinsTrajectory::DubinsTrajectory(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
      : nh_(nh),
        nh_private_(nh_private) {

            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",
                                                                             10);
            nh_private_.param("v_max", v_max_, 2.0);
            nh_private_.param("a_max", a_max_, 2.0);
            nh_private_.param("visualize", visualize_, true);
            nh_private_.param("curvature", curvature_, 2.0);
            nh_private_.param("num_arc", num_arc_, 50);
            nh_private_.param("num_straight", num_straight_, 100);
            nh_private_.param("size_factor", size_factor_, 0.05);
        
            pylon_one_.x() = 1.0;
            pylon_one_.y() = 1.0;
            pylon_one_.z() = 10.0;
            pylon_two_.x() = 31.0;
            pylon_two_.y() = 1.0;
            pylon_two_.z() = 10.0;
    }

    void DubinsTrajectory::init(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) {
        nh_ = nh; 
        nh_private_ = nh_private;
    
        nh_private_.param("visualize", visualize_, true);

        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::WHITE,      Color::White()));
        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::BLACK,      Color::Black()));
        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::BLUE,       Color::Blue()));
        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::ORANGE,     Color::Orange()));
        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::YELLOW,     Color::Yellow()));
        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::RED,        Color::Red()));
        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::PINK,       Color::Pink()));
        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::GREEN,      Color::Green()));
        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::GRAY,       Color::Gray()));
        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::TEAL,       Color::Teal()));
        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::CHARTREUSE, Color::Chartreuse()));
        color_map_.insert(std::make_pair(DubinsTrajectory::ColorType::PURPLE,     Color::Purple()));

        start_.position_.x()=-1.0;
        start_.position_.y()=1.0;
        start_.position_.z()=10.0;

        end_.position_.x()=33.0;
        end_.position_.y()=1.0;
        end_.position_.z()=10.0;
    }

    void DubinsTrajectory::generateTrajectory(Point start, Point end) {
        trajectory_.clear();
 
        start.position_.x()=start_.position_.x();
        start.position_.y()=start_.position_.y();
        start.position_.z()=start_.position_.z();

        end.position_.x()=end_.position_.x();
        end.position_.y()=end_.position_.y();
        end.position_.z()=end_.position_.z();

        turn_in_one_segment_=PI/2 - atan((end.position_.y() - 
                                start.position_.y())/(end.position_.x() - start.position_.x()));

        distance_ = sqrt((pylon_two_.x() - pylon_one_.x()) * (pylon_two_.x() - pylon_one_.x())
                            + (pylon_two_.y() - pylon_one_.y()) * (pylon_two_.y() - pylon_one_.y()));

        double small_change_in_angle = (turn_in_one_segment_ )/num_arc_;
        double small_change_in_distance = distance_/num_straight_; 

        start_.heading_angle_ = 0.0;
        Point prev = start;
        Point curr = start;
        curr.heading_angle_ = prev.heading_angle_ + small_change_in_angle ;
        trajectory_.push_back(curr);

        while(prev.heading_angle_ <= turn_in_one_segment_) {
            curr.position_.x()=pylon_one_.x() + curvature_ * sin(prev.heading_angle_);
            curr.position_.y()=pylon_one_.y() + curvature_ * cos(prev.heading_angle_);
            curr.heading_angle_ = prev.heading_angle_ + small_change_in_angle ;
            trajectory_.push_back(curr);
            prev = curr;
        }

        double distance = 0.0;

        while(distance <= distance_) {
            curr.position_.x() = prev.position_.x() + small_change_in_distance * sin(turn_in_one_segment_);
            curr.position_.y() = prev.position_.y() + small_change_in_distance * cos(turn_in_one_segment_);
            distance = distance + small_change_in_distance;
            trajectory_.push_back(curr);
            prev = curr;
        }

        while(prev.heading_angle_ <= end_.heading_angle_) {
            curr.position_.x()=pylon_two_.x() + curvature_ * sin(prev.heading_angle_);
            curr.position_.y()=pylon_two_.y() + curvature_ * cos(prev.heading_angle_);
            curr.heading_angle_ = prev.heading_angle_ + small_change_in_angle ;
            trajectory_.push_back(curr);
            prev = curr;
        }

    }

    void DubinsTrajectory::visualizeTrajectory(const std::string& topic_name, std::vector<Point> trajectory, 
                                                     const std::string& frame_id , const ColorType& color, const double& size_factor) {
        if(trajectory_.empty()) {
            return;
        }
        for(auto it=trajectory_.begin();it!=trajectory_.end();it++) {
            trajectory.push_back(*it);
        }
        // size_factor = size_factor_;

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = topic_name;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.scale.x = marker.scale.y = marker.scale.y = size_factor;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color = color_map_[color];
        
        geometry_msgs::Point prev_center;
        prev_center.x = trajectory[0].position_.x();
        prev_center.y = trajectory[0].position_.y();
        prev_center.z = trajectory[0].position_.z();

        for(uint i = 1; i < trajectory.size(); i++) {
            marker.points.push_back(prev_center);
            geometry_msgs::Point center;
            center.x = trajectory[i].position_.x();
            center.y = trajectory[i].position_.y();
            center.z = trajectory[i].position_.z();
            marker.points.push_back(center);
            prev_center = center;
        }
    
        // visualization_msgs::MarkerArray markers;
        // markers.markers.push_back(marker);
        // marker_pub_.publish(markers);


        visualization_msgs::MarkerArray marker_array;
        mav_msgs::EigenTrajectoryPointVector diagram_path;

        for (const Point path_point : trajectory_) {
            mav_msgs::EigenTrajectoryPoint point;
            point.position_W = path_point.position_;
            diagram_path.push_back(point);
        }

        double path_length = mav_planning::computePathLength(diagram_path);
        int num_vertices = diagram_path.size();

        if (visualize_) {
            marker_array.markers.push_back(mav_planning::createMarkerForPath(diagram_path, 
                                               frame_id, mav_visualization::Color::Purple(),"dubins_traje", 0.1));
        }

        ROS_INFO("Path length: %f Vertices: %d", path_length, num_vertices);

        if (visualize_) {
            marker_pub_.publish(marker_array);
        }

    }

    void DubinsTrajectory::run() {
        
        generateTrajectory(start_, end_);
        if(visualize_) {
            visualizeTrajectory("visualization_marker_array", trajectory_, "world", DubinsTrajectory::ColorType::TEAL, size_factor_);
        }
    }

} //namespace ariitk::trajectory_generation
