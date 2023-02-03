#include "cas726/mapper.hh"
#include <iostream>
#include <algorithm>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h> 

void cas726::Mapper::laser_callback(const sensor_msgs::msg::LaserScan &scan) {
    RCLCPP_INFO(this->get_logger(), "Got laser message with %ld ranges", scan.ranges.size());  
    
    // 0. lookup transform on TF
    geometry_msgs::msg::TransformStamped tf;
    try
    {
        tf = tf_buffer_->lookupTransform(map_frame_, scan.header.frame_id,
                                                    tf2::TimePointZero, tf2::durationFromSec(0.5));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }

    // 1. calculate transform as a Pose2 object
    Pose2 transform(tf.transform.translation.x, tf.transform.translation.y,
        tf.transform.rotation.z);
    transform.normalize();

    // 2.a iterate through all map cells and project them to the laser frame
    const float epsilon = 0.001;

    for (int i=0; i<width_x_; ++i) {
        for(int j=0; j<height_y_; ++j) {
            int prob = *at(i,j);

            // Transform coordinates

            // Convert to polar

            // look up ray @ theta
            if(std::abs(l -r) < epsilon) {
                prob += 1;
            }
            else if(r > (l+epsilon)) {
                ; //add zero
            }
            else {
                prob -= 1;
            }

            *at(i,j) = std::clamp(prob, INT8_MIN, INT8_MAX);
        }
    }

    // 2.b
    // for (auto s : scan.ranges) {
    //     // Increment laser angle
    //     transform.theta += scan.angle_increment;
    //     transform.normalize();
    // }


    // 3. update map cells

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
