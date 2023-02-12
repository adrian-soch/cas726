#ifndef CAS726_MAPPER_HH
#define CAS726_MAPPER_HH

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "cas726/pose.hh"

using std::placeholders::_1;

namespace cas726 {

    /** 
     * Mapper is our main class that subscribes to laser and odometry messages
     * and produces a map of the environment. It is derived from the Node class
     */
    class Mapper : public rclcpp::Node {
    public:
        //here implement constructor
        Mapper() : Node ("cas726_mapper") {
            //first declare parameters
            laser_topic_ = this->declare_parameter("scan_topic","/laser");
            map_topic_ = this->declare_parameter("map_topic","/map");
            odom_frame_ = this->declare_parameter("odom_frame","odom");
            map_frame_ = this->declare_parameter("map_frame","map");
            base_frame_ = this->declare_parameter("base_frame","base_frame");

            map_update_interval_ = this->declare_parameter("map_update_interval",0.5);
            map_resolution_ = this->declare_parameter("resolution",0.05);
            max_laser_range_ = this->declare_parameter("max_laser_range",8.0);
            map_x_ = this->declare_parameter("map_width",20.0);
            map_y_ = this->declare_parameter("map_height",30.0);
            beam_width_ = this->declare_parameter("beam_width",0.05);

            width_x_ = (unsigned int) ceil(map_x_/map_resolution_);
            height_y_ = (unsigned int) ceil(map_y_/map_resolution_);

            //next set up subscribers
            laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan> (
                laser_topic_, 10, std::bind(&Mapper::laser_callback,this,_1));	

            //tf listner 
            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            //set up automatic timer for map update publish
            std::chrono::milliseconds update_duration((int)(1000*map_update_interval_));
                timer_ = this->create_wall_timer(update_duration, 
                    std::bind(&Mapper::map_update_callback, this));	

            //set up publishers for the map
            map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 
                    rclcpp::QoS(1).reliable().transient_local());

            //set up map metadata
            map_msg_.header.frame_id = map_frame_;
            map_msg_.info.resolution = map_resolution_;
            map_msg_.info.width = width_x_;
            map_msg_.info.height = height_y_;

            map_msg_.info.origin.position.x = -map_x_/2;
            map_msg_.info.origin.position.y = -map_y_/2;

            //allocate map
            map_ = new int8_t[width_x_*height_y_];
            memset(map_,0,width_x_*height_y_);
            evd_occ_ = -10;    //once evidence is lower than threshold, consider occupied
            evd_free_ = 30;    //once above threshold consider free
        }

        virtual ~Mapper() {
            //de-allocate map
            delete []map_;
        }
    private:

    
        //laser callback
        void laser_callback(const sensor_msgs::msg::LaserScan &scan);
        //map update callback
        void map_update_callback();

        //publishers and subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        //tf stuff
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        
        //parameters
        std::string laser_topic_, map_topic_, odom_frame_, map_frame_, base_frame_ ;
        float map_update_interval_, map_resolution_, max_laser_range_, map_x_, map_y_, beam_width_;

        //intenral variables
        nav_msgs::msg::OccupancyGrid map_msg_;
        int width_x_, height_y_; //size of the map in pixels
        int8_t evd_occ_, evd_free_; //thresholds for occupied and free

        Pose2 map2sensor_;

        //the actual map unrolled into a 1D array
        int8_t *map_;

        //helper function for getting a map cell
        int8_t* at(int x, int y) {

            int index = x + width_x_*y;
            if(index >= width_x_*height_y_) {
                throw std::out_of_range ("Requested i,j outside of map.");
            }
            else {
                return &map_[index];
            }
        }

        //update map to message
        void update_map_msg();
    }; 
}
#endif
