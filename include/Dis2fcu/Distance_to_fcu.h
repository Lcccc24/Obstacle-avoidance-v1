#ifndef DISTANCE_TO_FCU_H
#define DISTANCE_TO_FCU_H

#endif

#include <ros/ros.h>
#include "Dis2fcu/histogram.h"
#include "Dis2fcu/common.h"
#include "Dis2fcu/transform_buffer.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>  // transformPointCloud
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>



namespace avoidance {

    struct cameraData {
        std::string topic_;
        ros::Subscriber pointcloud_sub_;

        pcl::PointCloud<pcl::PointXYZ> untransformed_cloud_;
        bool received_;

        pcl::PointCloud<pcl::PointXYZ> transformed_cloud_;
        bool transformed_;

        std::unique_ptr<std::mutex> camera_mutex_;
        std::unique_ptr<std::condition_variable> camera_cv_;

        bool transform_registered_ = false;
        std::thread transform_thread_;

        FOV fov_fcu_frame_;
    };

    class Distance_to_fcu
    {
    private:
        ros::NodeHandle nh_;
        ros::Publisher mavros_obstacle_distance_pub_;
        ros::Subscriber pose_sub_;
        ros::Time last_pointcloud_process_time_;
        pcl::PointCloud<pcl::PointXYZI> final_cloud_;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> original_cloud_vector_;
        Histogram to_fcu_histogram_ = Histogram(ALPHA_RES);
        Eigen::Vector3f position_ = Eigen::Vector3f::Zero();
        Eigen::Quaternionf orientation_;
        sensor_msgs::LaserScan distance_data_ = {};
        std::vector<FOV> fov_fcu_frame_;
        std::vector<std::string> camera_topics = 
                                        {"/camera_front/depth/points", "/camera_back/depth/points", "/camera_left/depth/points", 
                                        "/camera_right/depth/points", "/camera_up/depth/points", "/camera_down/depth/points"};
        
        //std::vector<std::string> camera_topics = {"/stereo/points2"};

        std::atomic<bool> should_exit_{false};
        std::thread pub_update_thread_;
        std::thread worker_tf_listener;
        tf::TransformListener* tf_listener_ = nullptr;

        avoidance::tf_buffer::TransformBuffer tf_buffer_;
        std::condition_variable tf_buffer_cv_;

        std::mutex buffered_transforms_mutex_;
        std::vector<std::pair<std::string, std::string>> buffered_transforms_;

        std::mutex transformed_cloud_mutex_;
        std::condition_variable transformed_cloud_cv_;

        int param_cp_dist_ = 1;
        float min_sensor_range_ = 0.2f;
        float max_sensor_range_ = 15.0f;
        float yaw_fcu_frame_deg_ = 0.0f;
        float pitch_fcu_frame_deg_ = 0.0f;
        float max_point_age_s_ = 20.0f;
        int min_num_points_per_cell_ = 1;

        void pubUpdateThread(void);
        void positionCallback(const geometry_msgs::PoseStamped& msg);
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, int index);
        void pointCloudTransformThread(int index);
        void transformBufferThread();
        void processPointcloud(pcl::PointCloud<pcl::PointXYZI>& final_cloud,
                       const std::vector<pcl::PointCloud<pcl::PointXYZ>>& complete_cloud, const std::vector<FOV>& fov,
                       float yaw_fcu_frame_deg, float pitch_fcu_frame_deg, const Eigen::Vector3f& position,
                       float min_sensor_range, float max_sensor_range, float max_age, float elapsed_s,
                       int min_num_points_per_cell);

    public:
        Distance_to_fcu();
        ~Distance_to_fcu();
        void init();
        std::vector<cameraData> cameras_;

        void generateNewHistogram(Histogram& polar_histogram, const pcl::PointCloud<pcl::PointXYZI>& cropped_cloud,
                          const Eigen::Vector3f& position);
        void publishLaserScan();
        void getObstacleDistanceData(sensor_msgs::LaserScan& obstacle_distance);
        void updateObstacleDistanceMsg(Histogram hist);
        void create2DObstacleRepresentation(const bool send_to_fcu);
        void compressHistogramElevation(Histogram& new_hist, const Histogram& input_hist, const Eigen::Vector3f& position);
        void initializeCameraSubscribers(std::vector<std::string>& camera_topics);
        size_t numTransformedClouds(void);
    };  
    
    Distance_to_fcu::Distance_to_fcu(/* args */)
    {
    }
    
    Distance_to_fcu::~Distance_to_fcu()
    {
        should_exit_ = true;
        {
            std::lock_guard<std::mutex> guard(buffered_transforms_mutex_);
            tf_buffer_cv_.notify_all();
        }

        {
            std::lock_guard<std::mutex> guard(transformed_cloud_mutex_);
            transformed_cloud_cv_.notify_all();
        }

        for (size_t i = 0; i < cameras_.size(); ++i) {
            {
            std::lock_guard<std::mutex> guard(*cameras_[i].camera_mutex_);
            cameras_[i].camera_cv_->notify_all();
            }
            if (cameras_[i].transform_thread_.joinable()) cameras_[i].transform_thread_.join();
        }

        if (pub_update_thread_.joinable()) pub_update_thread_.join();
        if (worker_tf_listener.joinable()) worker_tf_listener.join();

        if (tf_listener_ != nullptr) delete tf_listener_;
    }
    

}