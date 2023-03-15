#include "cas726/landmark_mapper.hh"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <rclcpp/executor.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std::chrono_literals;

//image callback
void cas726::LandmarkMapper::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image, 
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &depth_cloud) {

    std::cerr<<"Got a pair of images\n";
    //TODO
    //1. get current odom pose (base link in odom frame) as an Eigen::Affine3d
    geometry_msgs::msg::TransformStamped stransform;
    try
    {
        stransform = tf_buffer_->lookupTransform(odom_frame_, base_frame_,
                                                    tf2::TimePointZero, tf2::durationFromSec(3));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    
    Eigen::Affine3d T; //
    T = tf2::transformToEigen(stransform);

    //2. compute relative transform
    Eigen::Affine3d movement; //
    movement = last_pose_.inverse() * T;

    // std::cout << "Base-Odom" << T.matrix() << "\n" <<std::endl;
    // std::cout << "Move" << movement.matrix()<< "\n" << std::endl;
    
    //3. threshold for enough motion
    double transl = movement.translation().norm();
    Eigen::AngleAxisd rot(movement.rotation());
    double angl = rot.angle();
    //4. if we moved enough, lock the mutex and load up data
    if(transl > 1 || fabsf(angl) > M_PI/4) {
        {
        std::cerr<<"Movement since last pose is "<<transl<<" "<<angl<<std::endl;
        std::cerr<<"Acquiring lock... ";
        std::lock_guard<std::mutex> lg(data_mutex_); //acquire lock
        std::cerr<<"done. Copying images\n";
        color_image_ = *image;
        depth_cloud_ = *depth_cloud;
        }
        //5. signal worker thread to wake up
        data_cv_.notify_all();

        //6. set new last pose
        last_pose_ = T;
    }
}

/** This is what the worker thread runs:
 *  1. Loop until asked to quit
 *  2. When woken up, assemble service request and check for object detections
 *  3. Take detected objects and use them as landmarks
 */
void cas726::LandmarkMapper::run() {
    //image data copies
    sensor_msgs::msg::Image current_color_image;
    sensor_msgs::msg::PointCloud2 current_depth_cloud;
    while(true) {
        {
        //acquire mutex access
        std::unique_lock<std::mutex> mutex_locker(data_mutex_); 
        std::cerr<<"worker thread sleeping\n"; 
        data_cv_.wait(mutex_locker);     //release lock and wait to be woken up
        std::cerr<<"worker thread woke up\n";

        //copy data
        current_color_image = color_image_;
        current_depth_cloud = depth_cloud_;
        } //release lock

        if(!rclcpp::ok()||quit_) break; // check if we are asked to quit
        
        std::cerr<<"worker thread assembling request\n"; 
        //load up color image in service request
        auto request = std::make_shared<cas726_interfaces::srv::DetectObjects::Request>();
        request->color = color_image_;

        //wait for service to exist
        while (!detect_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting...");
        }
    
        //lambda magic to unblock once response is available 
        auto callback =  [this](rclcpp::Client<cas726_interfaces::srv::DetectObjects>::SharedFutureWithRequest future) { 
        std::cerr<<"Result callback\n";
            auto response = future.get();
        std::cerr<<"Response has "<<response.second->detections.size()<<" objects\n";
        this->res_cv_.notify_all(); 
        };
        auto result = detect_client_->async_send_request(request, std::move(callback));
        std::cerr<<"Request sent\n";
        {
        // Wait for the result.
        std::unique_lock<std::mutex> response_locker(res_mutex_);
        std::cerr<<"Waiting for result\n";
        res_cv_.wait(response_locker);
        }

        std::cerr<<"Resuts received, we have "<<result.get().second->detections.size()<<" objects\n";
    
        //TODO: 
        //clear current landmarks
        // landmarks_.clear();

        //iterate over detections
        for(auto det : result.get().second->detections) {
            //For each object create a point cloud iterator (sensor_msgs::PointCloud2Iterator)
            //iterate through point cloud and take out points that are within the bounding box

            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(current_depth_cloud, cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));

            std::cout << "det size " << det.x_min << " " << det.x_max << std::endl;
            std::cout << "det size " << det.y_min << " " << det.y_max << std::endl;

            pcl::PointCloud<pcl::PointXYZ> xyz_filtered_cloud;
            pcl::CropBox<pcl::PointXYZ> crop;
            crop.setInputCloud(cloud_ptr);
            Eigen::Vector4f min_point = Eigen::Vector4f(det.x_min, det.y_min, 0.0, 0);
            Eigen::Vector4f max_point = Eigen::Vector4f(det.x_max, det.y_max, 5.0, 0);
            crop.setMin(min_point);
            crop.setMax(max_point);
            crop.filter(xyz_filtered_cloud);

            std::cout << "Size of crop cloud" << xyz_filtered_cloud.size() << std::endl;

            // Remove outliers
            // pcl::PointCloud<pcl::PointXYZ>::Ptr sor_input_cloud(new pcl::PointCloud<pcl::PointXYZ>(xyz_filtered_cloud));
            // pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            // sor.setInputCloud(sor_input_cloud);
            // sor.setMeanK(50);
            // sor.setStddevMulThresh(1.0);
            // sor.filter(*sor_cloud_filtered);
            
            //compute average x,y,z

            Eigen::Vector4d centroid;
            centroid << 0.0, 0.0, 0.0, 1.0;
            // pcl::compute3DCentroid(xyz_filtered_cloud, centroid);

            //transform point to map frame and save it in the landmark array
            Eigen::Vector4d point_t;
            point_t = last_pose_ * centroid;

            geometry_msgs::msg::Point point;
            point.x = point_t[0];
            point.y = point_t[1];
            point.z = point_t[2];

            landmarks_.append("label", point);

            this->publishPointCloud(cloud_pub_, cloud);
            std::cout << "Cloud published." << std::endl;
        }

    }
}

//callback that publishes visualization markers
void cas726::LandmarkMapper::update_callback() {
    //TODO: publish the current array of landmarks as a visualization marker to display in rviz
    visualization_msgs::msg::Marker::SharedPtr sphere_list(new visualization_msgs::msg::Marker);
    sphere_list->header.frame_id = odom_frame_;
    sphere_list->header.stamp = this->get_clock()->now();
    sphere_list->type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphere_list->action = visualization_msgs::msg::Marker::ADD;
    sphere_list->points = landmarks_.points;
    sphere_list->scale.x = 0.3; // in meters
    sphere_list->scale.y = 0.3;
    sphere_list->scale.z = 0.3;

    // Set green and alpha(opacity)
    sphere_list->color.g = 1.0;
    sphere_list->color.a = 1.0;

    marker_publisher_->publish(*sphere_list);
    std::cout<<"tick tock\n";

    return;
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<cas726::LandmarkMapper>();

    //start up a new thread that spins the node
    std::promise<void> stop_async_spinner;
    std::thread async_spinner_thread(
      [stop_token = stop_async_spinner.get_future(), node]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin_until_future_complete(stop_token);
        //at this point we are done and should signal the node
        node->quit();
      });

    //execute program logic
    node->run();

    stop_async_spinner.set_value();
    async_spinner_thread.join();

    return 0;
}
