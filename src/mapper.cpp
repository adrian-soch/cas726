/**
 * @file mapper.cpp
 * @brief Create and update 2D occupancy grid map 
 * @version 0.1
 * @date 2023-02-05
 * 
 * @todo - Optimize ray search algo
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "cas726/mapper.hh"
#include <iostream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

int closest_value(std::vector<float> const& vec, float value) {
    auto const it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end()) { return -1; }
    int idx = it - vec.begin();
    
    return idx;
}

void cas726::Mapper::laser_callback(const sensor_msgs::msg::LaserScan &scan) {
    RCLCPP_INFO(this->get_logger(), "Got laser message with %ld ranges", scan.ranges.size());  
    
    // 0. lookup transform on TF
    geometry_msgs::msg::TransformStamped tf;
    try
    {
        tf = tf_buffer_->lookupTransform(map_frame_, scan.header.frame_id,
                                                    tf2::TimePointZero, tf2::durationFromSec(3));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    
    // 1. calculate transform as a Pose2 object
    Pose2 transform(tf.transform.translation.x, tf.transform.translation.y,
        tf.transform.rotation.z);
    transform.normalize();
    // std::cout << transform.x << transform.y << std::endl;

    // 2.a iterate through all map cells and project them to the laser frame
    const float epsilon {0.01};

    for (int i=0; i<width_x_; ++i) {
        for(int j=0; j<height_y_; ++j) {
            
            
            // Transform coordinates
            Eigen::Vector2f m_coord {i*map_resolution_, j*map_resolution_};
            
            Pose2 map_cell(m_coord[0], m_coord[1], 0.0);
            Pose2 m2l = map_cell + transform;

            Eigen::Vector2f s_coord {m2l.x, m2l.y};

            // Convert to polar
            auto r = s_coord.norm();
            auto theta = std::atan2(s_coord[1], s_coord[0]);

            // look up ray @ theta
            static std::vector<float> angles(scan.ranges.size(), 0);
            for (size_t i=0; i < angles.size(); ++i) {
                angles.at(i) = scan.angle_min + i*scan.angle_increment;
            }
            int idx = closest_value(angles, theta);
            auto l = scan.ranges.at(idx);
            
            // std::cout << "\nr: " << r << " theta: " << theta << " l: " << l << " sx " << s_coord[0] << " sy " << s_coord[1] << std::endl;

            int prob = *at(i,j);
            if(std::abs(l - r) < epsilon) {
                prob += 1;
                // std::cout << j << " adding 1" << std::endl;
            }
            else if(r > (l+epsilon)) {
                ; //add zero
                // std::cout << j << " adding 0" << std::endl;
            }
            else {
                prob -= 1;
                // std::cout << j << " adding -1" << std::endl;
            }
            *at(i,j) = prob;
        }
    }

    // 2.b
    // for (auto s : scan.ranges) {
    //     // Increment laser angle
    //     transform.theta += scan.angle_increment;
    //     transform.normalize();
    // }


    // 3. update map cells
    update_map_msg();

}

//create a message and publish to map update topic      
void cas726::Mapper::map_update_callback() {
    RCLCPP_INFO(this->get_logger(), "Updating map");

    map_msg_.info.map_load_time = now();
    map_msg_.header.stamp = now();
    this->update_map_msg();
    map_publisher_->publish(map_msg_);
}

//update map to message
void cas726::Mapper::update_map_msg() {
    int8_t FREE=0;
    int8_t OCC=100;
    int8_t UNKN=-1;
    map_msg_.data.clear();
  
    for (int i=0; i<width_x_; ++i) {
        for(int j=0; j<height_y_; ++j) {

            auto prob = *at(i,j);
            auto val = UNKN; //Default Unknown occupancy

            if (prob < evd_occ_) {val = OCC;}
            else if (prob > evd_free_) {val = FREE;}

            map_msg_.data.push_back(val);
        }
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<cas726::Mapper>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
