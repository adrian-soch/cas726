#ifndef CAS726_LANDMARK_MAPPER_HH
#define CAS726_LANDMARK_MAPPER_HH

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <thread>
#include <condition_variable>

//Message types
#include "cas726_interfaces/msg/bounding_box.hpp"
#include "cas726_interfaces/srv/detect_objects.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

//For message filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

//for TF
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include<Eigen/Core>
#include<Eigen/Geometry>

namespace cas726 {

// Helper Class
class Landmarks {
    public:
        void append(const std::string &label, 
            const geometry_msgs::msg::Point &point) {
            labels.push_back(label);
            points.push_back(point);
        }

        /**
         * @brief Simple data association. Only add a landmark
         *        if it has not been seen before based on distance
         *        threshold.
         * 
         * @param label 
         * @param point 
         */
        void appendNewLandmarkOnly(const std::string &label, 
            const geometry_msgs::msg::Point &point) {

            int size = points.size();
            if(size == 0) {
                labels.push_back(label);
                points.push_back(point);

                return;
            }

            bool seen = false;
            for(int i=0; i < size; ++i) {
                if(abs(point.x - points[i].x) < x_thresh_ &&
                abs(point.y - points[i].y) < y_thresh_ &&
                label == labels[i]) {
                    seen = true;
                }

            }
            if(!seen) {
                labels.push_back(label);
                points.push_back(point);
            }
        }
        
        /**
         * @brief Removes all landmarks
         * 
         */
        void clear() {
            labels.clear();
            points.clear();
        }

        /**
         * @brief Set the Thresh object for data association
         * 
         * @param x_thresh 
         * @param y_thresh 
         */
        void setThresh(float x_thresh, float y_thresh) {
            x_thresh_ = abs(x_thresh);
            y_thresh_ = abs(y_thresh);
        }

        // Struct of Vectors used so we can
        //  pass the points to other functions
        std::vector<std::string> labels;
        std::vector<geometry_msgs::msg::Point> points;

    private:
        float x_thresh_ = 0.6; //meters
        float y_thresh_ = 0.6; //meters
};

/**
 * Landmark Mapper is a class that creates a map of landmarks. Each landmark is 
 * detected in the RGB images as an object of interest using a service call to a
 * python node running a standard model.
 */
class LandmarkMapper : public rclcpp::Node {
    public:
    //here implement constructor worker_thread_(&LandmarkMapper::run,this), 
    LandmarkMapper() : Node ("cas726_landmark_mapper"), quit_(false) {
        //first declare parameters
        image_topic_ = this->declare_parameter("image_topic","/front/color_image");
        depth_cloud_topic_ = this->declare_parameter("depth_cloud_topic","/front/points");

        detect_srv_topic_ = this->declare_parameter("detect_srv_topic","/object_detector/detect");
        odom_frame_ = this->declare_parameter("odom_frame","odom");
        map_frame_ = this->declare_parameter("map_frame","map");
        base_frame_ = this->declare_parameter("base_frame","base_link");

        last_pose_ = Eigen::Affine3d::Identity();
        //define the quality of service profiles
        rclcpp::QoS qos(10);
        auto rmw_qos_profile = qos.get_rmw_qos_profile();
        
        //next set up subscriber filters
        image_subscriber_.subscribe(this, image_topic_, rmw_qos_profile);
        depth_cloud_subscriber_.subscribe(this, depth_cloud_topic_, rmw_qos_profile);

        //synchronized callback
        image_sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, 
                        sensor_msgs::msg::PointCloud2>>(image_subscriber_, depth_cloud_subscriber_, 1);

        image_sync_->registerCallback(std::bind(&LandmarkMapper::image_callback, this, std::placeholders::_1, 
            std::placeholders::_2));

        //tf listner 
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        //set up automatic timer for update publish 1 per second
        std::chrono::milliseconds update_duration(1000);
        timer_ = this->create_wall_timer(update_duration, 
        std::bind(&LandmarkMapper::update_callback, this));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obj_cloud", 1);
        cloud_pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obj_cloud2", 1);

        // set up marker Viz publisher
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("markers", 1);
        
        //Client to detect objects
        detect_client_ = this->create_client<cas726_interfaces::srv::DetectObjects>(detect_srv_topic_);

    }

    virtual ~LandmarkMapper() {}

    //signals to shutdown and joins thread
    void quit() {
        {
            //make sure we have access to the mutex before changing the quit flag
            std::lock_guard<std::mutex> lg (data_mutex_);
                quit_=true;
        }
        //notify worker thread to wake up and check condition flag
        data_cv_.notify_all();
    }

    //program logic goes in this function
    void run();

private:
    Landmarks landmarks_;
    //should the program exit?
    bool quit_;
    //synchronization variables
    std::mutex data_mutex_;              //mutex variable for image data
    std::condition_variable data_cv_;    //condition variable associated with images

    std::mutex res_mutex_;              //mutex variable to wait for response on service
    std::condition_variable res_cv_;    //condition variable associated with service rsponse
    
    //image data copies
    sensor_msgs::msg::Image color_image_;
    sensor_msgs::msg::PointCloud2 depth_cloud_;

    template <typename PointT>
    void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
        pcl::PointCloud<PointT> point_cloud)
    {
        sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(point_cloud, *pc2_cloud);
        pc2_cloud->header.frame_id = odom_frame_;
        pc2_cloud->header.stamp = this->get_clock()->now();
        publisher->publish(*pc2_cloud);
    }

    //synchronized image callback. copies data to local member variables and returns
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image, 
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &points);
    
    //callback that publishes visualization markers
    void update_callback();

    //publishers and subscribers
    message_filters::Subscriber<sensor_msgs::msg::Image> image_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> depth_cloud_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_, cloud_pub2_;
    
    //synchronization policy 
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>> image_sync_;

    //Service client
    rclcpp::Client<cas726_interfaces::srv::DetectObjects>::SharedPtr detect_client_;

    rclcpp::TimerBase::SharedPtr timer_;
    //tf stuff
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    //parameters
    std::string image_topic_, depth_cloud_topic_, detect_srv_topic_, odom_frame_, map_frame_, base_frame_; 

    //Last pose we updated at
    Eigen::Affine3d last_pose_;
}; 
}
#endif
