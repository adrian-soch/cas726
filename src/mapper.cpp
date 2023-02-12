/**
 * @file mapper.cpp
 * @brief Create and update 2D occupancy grid map 
 * @version 0.1
 * @date 2023-02-05
 * 
 * @todo - Thread the grid update
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "cas726/mapper.hh"
#include <iostream>
#include <chrono>
#include <algorithm>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

int closest_value(std::vector<float> const& vec, float value) {
    auto const it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end()) { return it - vec.begin() - 1; }
    int idx = it - vec.begin();
    
    return idx;
}

void cas726::Mapper::laser_callback(const sensor_msgs::msg::LaserScan &scan) {
    // RCLCPP_INFO(this->get_logger(), "Got laser message with %ld ranges", scan.ranges.size());
    auto start = std::chrono::high_resolution_clock::now();
    
    // 0. lookup transform on TF
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(map_frame_, scan.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(3));
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
 
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 1. calculate transform as a Pose2 object
    Pose2 pose(tf.transform.translation.x, tf.transform.translation.y, yaw);
    pose.normalize();

    // 2.a iterate through all map cells and project them to the laser frame
    const float epsilon {0.02};
    for (int i=0; i<width_x_; ++i) {
        for(int j=0; j<height_y_; ++j) {

            // Center of mass of the cell m(i,j)           
            Pose2 map_cell( (i+0.5)*map_resolution_ + map_msg_.info.origin.position.x,
                            (j+0.5)*map_resolution_ + map_msg_.info.origin.position.y, 
                            0.0);

            Eigen::Vector2f s_coord {map_cell.x - pose.x, map_cell.y - pose.y};

            // Convert to polar
            auto r = s_coord.norm();
            auto phi = std::atan2(s_coord[1], s_coord[0]) - pose.theta;
            phi = pose.normalize(phi);

            // look up ray @ phi
            static std::vector<float> angles(scan.ranges.size(), 0);
            for(size_t i=0; i < angles.size(); ++i) {
                angles.at(i) = scan.angle_min + i*scan.angle_increment;
            }
            int k = closest_value(angles, phi);
            auto l = scan.ranges.at(k);

            int prob = *at(i,j);
            if(std::abs(l - r) < epsilon) {
                if(prob < INT8_MAX) {prob += 1;}
            }
            else if(r > (l + epsilon)) {
                ; //add zero
            }
            else {
                if(prob > INT8_MIN) {prob -= 1;}
            }
            *at(i,j) = prob;
        }
    }

    // 3. update map cells
    update_map_msg();

    auto stop = std::chrono::high_resolution_clock::now();
    auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    RCLCPP_INFO(get_logger(), "Time to process scan (msec): %ld", t_ms.count());
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
  
    for(int j=0; j<height_y_; ++j) {
        for (int i=0; i<width_x_; ++i) {
            auto prob = *at(i,j);
            auto val = UNKN; //Default Unknown occupancy

            if (prob > evd_free_) {val = FREE;}
            else if (prob < evd_occ_) {val = OCC;}

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
