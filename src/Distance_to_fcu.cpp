#include "Dis2fcu/Distance_to_fcu.h"

namespace avoidance {

/**
 * @brief 生成新的极坐标直方图
 *
 * 根据给定的裁剪后的点云和位置，生成新的极坐标直方图。
 *
 * @param polar_histogram 直方图对象，用于存储生成的极坐标直方图
 * @param cropped_cloud 裁剪后的点云数据
 * @param position 位置信息，用于计算极坐标
 */
void Distance_to_fcu::generateNewHistogram(Histogram& polar_histogram, const pcl::PointCloud<pcl::PointXYZI>& cropped_cloud,
                          const Eigen::Vector3f& position) {
  Eigen::MatrixXi counter(GRID_LENGTH_E, GRID_LENGTH_Z);
  counter.fill(0);

  for (auto xyz : cropped_cloud) { 
    Eigen::Vector3f p = toEigen(xyz);
    //std::cout << "p" << ","<< p << std::endl;
    PolarPoint p_pol = cartesianToPolarHistogram(p, position);
    //std::cout << "pol_z" << ","<< p_pol.z << std::endl;
    float dist = p_pol.r;
    Eigen::Vector2i p_ind = polarToHistogramIndex(p_pol, ALPHA_RES);
    counter(p_ind.y(), p_ind.x()) += 1;
    polar_histogram.set_dist(p_ind.y(), p_ind.x(), polar_histogram.get_dist(p_ind.y(), p_ind.x()) + dist);
  }

  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      if (counter(e, z) > 0) {
        polar_histogram.set_dist(e, z, polar_histogram.get_dist(e, z) / counter(e, z));
      } else {
        polar_histogram.set_dist(e, z, 0.f);
      }
    }
  }

}


/**
 * @brief 发布激光扫描数据
 *
 * 此函数用于发布激光扫描数据到无人机飞行控制单元（fcu）。
 *
 * 使用反向逻辑确保默认值如非数字（NAN）默认发送消息。
 *
 * TODO: 完善代码注释和逻辑。
 */
void Distance_to_fcu::publishLaserScan() {
  // inverted logic to make sure values like NAN default to sending the message
  // TODO
  if (!(param_cp_dist_ < 0)) {
    sensor_msgs::LaserScan distance_data_to_fcu;
    getObstacleDistanceData(distance_data_to_fcu);

    // only send message if planner had a chance to fill it with valid data
    if (distance_data_to_fcu.angle_increment > 0.f) {
      mavros_obstacle_distance_pub_.publish(distance_data_to_fcu);
    }
  }
}


/**
 * @brief 获取障碍物的距离数据
 *
 * 从distance_data_中读取障碍物的距离数据，并将其赋值给传入的obstacle_distance参数。
 *
 * @param obstacle_distance 用于存储障碍物距离数据的sensor_msgs::LaserScan类型变量
 */
void Distance_to_fcu::getObstacleDistanceData(sensor_msgs::LaserScan& obstacle_distance) {
  obstacle_distance = distance_data_;
}


/**
 * @brief 更新障碍物距离消息
 *
 * 根据直方图数据更新障碍物距离消息，并将其存储在成员变量distance_data_中。
 *
 * @param hist 直方图数据
 */
void Distance_to_fcu::updateObstacleDistanceMsg(Histogram hist) {
  
  sensor_msgs::LaserScan msg = {};

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.angle_increment = static_cast<double>(ALPHA_RES) * M_PI / 180.0;
  msg.range_min = min_sensor_range_;
  msg.range_max = max_sensor_range_;
  msg.ranges.reserve(GRID_LENGTH_Z);

  for (int i = 0; i < GRID_LENGTH_Z; ++i) {
    // turn idxs 180 degress to point to local north instead of south
    int j = (i + GRID_LENGTH_Z / 2) % GRID_LENGTH_Z;
    float dist = hist.get_dist(0, j);

    // is bin inside FOV?
    //std::cout << "j: " << j << std::endl;
    //前后左右数据共60个
    if (histogramIndexYawInsideFOV(fov_fcu_frame_, j, position_, yaw_fcu_frame_deg_)) {
      msg.ranges.push_back(dist > min_sensor_range_ ? dist : max_sensor_range_ + 0.01f);
    } else {
      msg.ranges.push_back(NAN);
    }
    //msg.ranges.push_back(dist > min_sensor_range_ ? dist : max_sensor_range_ + 0.01f);
  }

  //lc add
  //down obstacle data
  for (int i = 0; i < UP_DOWN_BLOCK; ++i) {
    float min_dist_down = max_sensor_range_ + 0.01f;
    for(int j = 0; j < (GRID_LENGTH_Z / UP_DOWN_BLOCK); ++j){
      int k = (j + i * GRID_LENGTH_Z / UP_DOWN_BLOCK + GRID_LENGTH_Z / 2) % GRID_LENGTH_Z;
      float dist = hist.get_dist(1, k);
      if(dist < min_dist_down && dist != 0.0f)
        min_dist_down = dist;
    }
    msg.ranges.push_back(min_dist_down);
  }

  //up obstacle data
  for (int i = 0; i < UP_DOWN_BLOCK; ++i) {
    float min_dist_down = max_sensor_range_ + 0.01f;
    for(int j = 0; j < (GRID_LENGTH_Z / UP_DOWN_BLOCK); ++j){
      int k = (j + i * GRID_LENGTH_Z / UP_DOWN_BLOCK + GRID_LENGTH_Z / 2) % GRID_LENGTH_Z;
      float dist = hist.get_dist(2, k);
      if(dist < min_dist_down && dist != 0.0f)
        min_dist_down = dist;
    }
    msg.ranges.push_back(min_dist_down);
  }

  distance_data_ = msg;
}


/**
 * @brief 压缩直方图的海拔信息
 *
 * 将输入直方图的海拔信息压缩到新的直方图中，只保留无人机上下一定范围内的障碍物信息。
 *
 * @param new_hist 新的直方图对象，用于存储压缩后的海拔信息
 * @param input_hist 输入的直方图对象，包含原始的海拔信息
 * @param position 无人机的位置信息，用于计算障碍物与无人机的相对位置
 */
void Distance_to_fcu::compressHistogramElevation(Histogram& new_hist, const Histogram& input_hist, const Eigen::Vector3f& position) {
  //need change
  float vertical_FOV_range_sensor = 24.0;
  //need change
  float vertical_cap = 0.8f;  // ignore obstacles, which are more than that above or below the drone.
  PolarPoint p_pol_lower(-1.0f * vertical_FOV_range_sensor / 2.0f, 0.0f, 0.0f);
  PolarPoint p_pol_upper(vertical_FOV_range_sensor / 2.0f, 0.0f, 0.0f);
  Eigen::Vector2i p_ind_lower = polarToHistogramIndex(p_pol_lower, ALPHA_RES);
  Eigen::Vector2i p_ind_upper = polarToHistogramIndex(p_pol_upper, ALPHA_RES);

  for (int e = p_ind_lower.y(); e <= p_ind_upper.y(); e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      if (input_hist.get_dist(e, z) > 0) {
        // check if inside vertical range
        PolarPoint obstacle = histogramIndexToPolar(e, z, ALPHA_RES, input_hist.get_dist(e, z));
        Eigen::Vector3f obstacle_cartesian = polarHistogramToCartesian(obstacle, position);
        float height_difference = std::abs(position.z() - obstacle_cartesian.z());
        if (height_difference < vertical_cap &&
            (input_hist.get_dist(e, z) < new_hist.get_dist(0, z) || new_hist.get_dist(0, z) == 0.f))
          new_hist.set_dist(0, z, input_hist.get_dist(e, z));
      }
    }
  }


  //lc add
  //down obstacle data -90~-60 degree
  PolarPoint d_pol_lower(-90.0f, 0.0f, 0.0f);
  PolarPoint d_pol_upper(-60.0f, 0.0f, 0.0f);
  Eigen::Vector2i d_ind_lower = polarToHistogramIndex(d_pol_lower, ALPHA_RES);
  Eigen::Vector2i d_ind_upper = polarToHistogramIndex(d_pol_upper, ALPHA_RES);

  for (int e = d_ind_lower.y(); e <= d_ind_upper.y(); e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      if (input_hist.get_dist(e, z) > 0) {
        // check if inside vertical range
        PolarPoint obstacle = histogramIndexToPolar(e, z, ALPHA_RES, input_hist.get_dist(e, z));
        Eigen::Vector3f obstacle_cartesian = polarHistogramToCartesian(obstacle, position);
        float height_difference = std::abs(position.z() - obstacle_cartesian.z());
        if (height_difference < 15.0f &&
            (input_hist.get_dist(e, z) < new_hist.get_dist(1, z) || new_hist.get_dist(1, z) == 0.f))
          new_hist.set_dist(1, z, input_hist.get_dist(e, z));
      }
    }
  }

  //up obstacle data 60~90 degree
  PolarPoint u_pol_lower(60.0f, 0.0f, 0.0f);
  PolarPoint u_pol_upper(90.0f, 0.0f, 0.0f);
  Eigen::Vector2i u_ind_lower = polarToHistogramIndex(u_pol_lower, ALPHA_RES);
  Eigen::Vector2i u_ind_upper = polarToHistogramIndex(u_pol_upper, ALPHA_RES);

  for (int e = u_ind_lower.y(); e <= u_ind_upper.y(); e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      if (input_hist.get_dist(e, z) > 0) {
        //check if inside vertical range
        PolarPoint obstacle = histogramIndexToPolar(e, z, ALPHA_RES, input_hist.get_dist(e, z));
        Eigen::Vector3f obstacle_cartesian = polarHistogramToCartesian(obstacle, position);
        float height_difference = std::abs(position.z() - obstacle_cartesian.z());
        if (height_difference < 15.0f &&
            (input_hist.get_dist(e, z) < new_hist.get_dist(2, z) || new_hist.get_dist(2, z) == 0.f))
          new_hist.set_dist(2, z, input_hist.get_dist(e, z));
      }
    }
  }

}


/**
 * @brief 创建一个二维障碍物表示，并决定是否将其发送到飞控单元（FCU）
 *
 * 根据需要或飞控单元的要求，构造直方图。
 *
 * @param send_to_fcu 是否将直方图发送到飞控单元
 */
void Distance_to_fcu::create2DObstacleRepresentation(const bool send_to_fcu) {
  // construct histogram if it is needed
  // or if it is required by the FCU
  Histogram new_histogram = Histogram(ALPHA_RES);
  to_fcu_histogram_.setZero();

  generateNewHistogram(new_histogram, final_cloud_, position_);

  if (send_to_fcu) {
    compressHistogramElevation(to_fcu_histogram_, new_histogram, position_);
    updateObstacleDistanceMsg(to_fcu_histogram_);
  }

}


/**
 * @brief 回调函数，用于处理位置消息
 *
 * 该函数接收一个geometry_msgs::PoseStamped类型的消息，并将其位置和方向转换为Eigen格式，
 * 同时计算偏航角（yaw）并将其存储在yaw_fcu_frame_deg_中。
 *
 * @param msg 包含位置和方向信息的PoseStamped消息
 */
void Distance_to_fcu::positionCallback(const geometry_msgs::PoseStamped& msg) {
  position_ = toEigen(msg.pose.position);
  orientation_ = toEigen(msg.pose.orientation);

  //yaw_fcu_frame_deg_  pitch_fcu_frame_deg_  获取
  yaw_fcu_frame_deg_ = getYawFromQuaternion(orientation_);
  pitch_fcu_frame_deg_ = getPitchFromQuaternion(orientation_);
  //ROS_INFO_STREAM("Yaw: " << yaw_fcu_frame_deg_);
  //std::cout << "yaw_fcu_frame_deg_: " << yaw_fcu_frame_deg_ << std::endl;
  //ROS_INFO_STREAM("Pitch: " << pitch_fcu_frame_deg_);

}



/**
 * @brief 初始化摄像头订阅者
 *
 * 初始化给定摄像头话题的订阅者，并为每个摄像头启动点云转换线程。
 *
 * @param camera_topics 摄像头话题列表
 */
void Distance_to_fcu::initializeCameraSubscribers(std::vector<std::string>& camera_topics) {
  cameras_.resize(camera_topics.size());

  for (size_t i = 0; i < camera_topics.size(); i++) {
    cameras_[i].camera_mutex_.reset(new std::mutex);
    cameras_[i].camera_cv_.reset(new std::condition_variable);

    cameras_[i].received_ = false;
    cameras_[i].transformed_ = false;

    cameras_[i].pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        camera_topics[i], 1, boost::bind(&Distance_to_fcu::pointCloudCallback, this, _1, i));
    cameras_[i].topic_ = camera_topics[i];
    cameras_[i].transform_thread_ = std::thread(&Distance_to_fcu::pointCloudTransformThread, this, i);
  }
}


/**
 * @brief 点云回调处理函数
 *
 * 处理从sensor_msgs::PointCloud2类型的消息中接收到的点云数据，并更新相关状态。
 *
 * @param msg 指向sensor_msgs::PointCloud2消息的智能指针
 * @param index 相机索引
 */
void Distance_to_fcu::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, int index) {
  std::lock_guard<std::mutex> lck(*(cameras_[index].camera_mutex_));

  auto timeSinceLast = [&]() -> ros::Duration {
    ros::Time lastCloudReceived = pcl_conversions::fromPCL(cameras_[index].untransformed_cloud_.header.stamp);
    return msg->header.stamp - lastCloudReceived;
  };

  if (cameras_[index].received_ && timeSinceLast() < ros::Duration(0.3)) {
    return;
  }

  pcl::fromROSMsg(*msg, cameras_[index].untransformed_cloud_);
  cameras_[index].received_ = true;
  cameras_[index].camera_cv_->notify_all();

  // this runs once at the beginning to get the transforms
  if (!cameras_[index].transform_registered_) {
    std::lock_guard<std::mutex> tf_list_guard(buffered_transforms_mutex_);
    std::pair<std::string, std::string> transform_frames;
    transform_frames.first = msg->header.frame_id;
    transform_frames.second = "map";
    buffered_transforms_.push_back(transform_frames);
    transform_frames.second = "base_link";
    buffered_transforms_.push_back(transform_frames);
    cameras_[index].transform_registered_ = true;
  }

  //ROS_INFO("CALLBACK");
}


/**
 * @brief 点云转换线程函数
 *
 * 该函数用于将点云从原始坐标系转换到局部坐标系。
 *
 * @param index 相机索引
 */
void Distance_to_fcu::pointCloudTransformThread(int index) {
  while (!should_exit_) {

    //ROS_INFO("%d,pointCloudTransformThread RUNNING", index);
    bool waiting_on_transform = false;
    bool waiting_on_cloud = false;
    {
      std::lock_guard<std::mutex> camera_lock(*(cameras_[index].camera_mutex_));

      if (cameras_[index].received_) {
        tf::StampedTransform cloud_transform;
        tf::StampedTransform fcu_transform;

        if (tf_buffer_.getTransform(cameras_[index].untransformed_cloud_.header.frame_id, "map",
                                    pcl_conversions::fromPCL(cameras_[index].untransformed_cloud_.header.stamp),
                                    cloud_transform) &&
            tf_buffer_.getTransform(cameras_[index].untransformed_cloud_.header.frame_id,"base_link",
                                    pcl_conversions::fromPCL(cameras_[index].untransformed_cloud_.header.stamp),
                                    fcu_transform)) {
          // remove nan padding and compute fov
          pcl::PointCloud<pcl::PointXYZ> maxima = removeNaNAndGetMaxima(cameras_[index].untransformed_cloud_);

          // update point cloud FOV
          pcl_ros::transformPointCloud(maxima, maxima, fcu_transform);
          updateFOVFromMaxima(cameras_[index].fov_fcu_frame_, maxima);
          //ROS_WARN("FOV: %f",cameras_[index].fov_fcu_frame_.h_fov_deg);

          // transform cloud to local_origin frame
          pcl_ros::transformPointCloud(cameras_[index].untransformed_cloud_, cameras_[index].transformed_cloud_,
                                       cloud_transform);
          cameras_[index].transformed_cloud_.header.frame_id = "map";
          cameras_[index].transformed_cloud_.header.stamp = cameras_[index].untransformed_cloud_.header.stamp;

          cameras_[index].transformed_ = true;
          cameras_[index].received_ = false;
          waiting_on_cloud = true;
          std::lock_guard<std::mutex> lock(transformed_cloud_mutex_);
          transformed_cloud_cv_.notify_all();
          //ROS_INFO("GOT TRANSFORM");
        } else {
          waiting_on_transform = true;
          //ROS_INFO("WAITING FOR TRANSFORM");
        }
      } else {
        waiting_on_cloud = true;
        //ROS_INFO("WAITING FOR CLOUD");
      }
    }

    if (should_exit_) {
      break;
    }

    if (waiting_on_transform) {
      std::unique_lock<std::mutex> lck(buffered_transforms_mutex_);
      tf_buffer_cv_.wait_for(lck, std::chrono::milliseconds(1000));
    } else if (waiting_on_cloud) {
      std::unique_lock<std::mutex> lck(*(cameras_[index].camera_mutex_));
      cameras_[index].camera_cv_->wait_for(lck, std::chrono::milliseconds(1000));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}



/**
 * @brief 处理点云数据，生成最终点云
 *
 * 该函数将输入的点云数据经过一系列处理，生成最终的点云数据。
 *
 * @param final_cloud 输出参数，存储处理后的点云数据
 * @param complete_cloud 输入参数，存储完整的点云数据
 * @param fov 输入参数，存储视野信息
 * @param yaw_fcu_frame_deg 输入参数，表示FCU框架的偏航角（度）
 * @param pitch_fcu_frame_deg 输入参数，表示FCU框架的俯仰角（度）
 * @param position 输入参数，表示位置信息
 * @param min_sensor_range 输入参数，表示传感器最小量程
 * @param max_sensor_range 输入参数，表示传感器最大量程
 * @param max_age 输入参数，表示点的最大年龄
 * @param elapsed_s 输入参数，表示经过的时间（秒）
 * @param min_num_points_per_cell 输入参数，表示每个单元格中的最小点数
 */
void Distance_to_fcu::processPointcloud(pcl::PointCloud<pcl::PointXYZI>& final_cloud,
                       const std::vector<pcl::PointCloud<pcl::PointXYZ>>& complete_cloud, const std::vector<FOV>& fov,
                       float yaw_fcu_frame_deg, float pitch_fcu_frame_deg, const Eigen::Vector3f& position,
                       float min_sensor_range, float max_sensor_range, float max_age, float elapsed_s,
                       int min_num_points_per_cell) {

  //ROS_INFO("PROCESSING CLOUD");
  const int SCALE_FACTOR = 1;
  pcl::PointCloud<pcl::PointXYZI> old_cloud;
  std::swap(final_cloud, old_cloud);
  final_cloud.points.clear();
  final_cloud.width = 0;
  final_cloud.points.reserve((SCALE_FACTOR * GRID_LENGTH_Z) * (SCALE_FACTOR * GRID_LENGTH_E));

  // counter to keep track of how many points lie in a given cell
  Eigen::MatrixXi histogram_points_counter(180 / (ALPHA_RES / SCALE_FACTOR), 360 / (ALPHA_RES / SCALE_FACTOR));
  histogram_points_counter.fill(0);

  auto sqr = [](float f) { return f * f; };

  for (const auto& cloud : complete_cloud) {
    for (const pcl::PointXYZ& xyz : cloud) {
      // Check if the point is invalid
      if (!std::isnan(xyz.x) && !std::isnan(xyz.y) && !std::isnan(xyz.z)) {
        float distanceSq = (position - toEigen(xyz)).squaredNorm();
        if (sqr(min_sensor_range) < distanceSq && distanceSq < sqr(max_sensor_range)) {
          // subsampling the cloud
          PolarPoint p_pol = cartesianToPolarHistogram(toEigen(xyz), position);
          Eigen::Vector2i p_ind = polarToHistogramIndex(p_pol, ALPHA_RES / SCALE_FACTOR);
          histogram_points_counter(p_ind.y(), p_ind.x())++;
          if (histogram_points_counter(p_ind.y(), p_ind.x()) == min_num_points_per_cell) {
            final_cloud.points.push_back(toXYZI(toEigen(xyz), 0.0f));

          }
        }
      }
    }
  }
  //std::cout << "1111:" << final_cloud.points.size() << std::endl;
  // combine with old cloud
  for (const pcl::PointXYZI& xyzi : old_cloud) {
    float distanceSq = (position - toEigen(xyzi)).squaredNorm();
    if (distanceSq < sqr(max_sensor_range)) {
      // adding older points if not expired and space is free according to new cloud
      PolarPoint p_pol = cartesianToPolarHistogram(toEigen(xyzi), position);
      PolarPoint p_pol_fcu = cartesianToPolarFCU(toEigen(xyzi), position);
      p_pol_fcu.e -= pitch_fcu_frame_deg;
      p_pol_fcu.z -= yaw_fcu_frame_deg;
      wrapPolar(p_pol_fcu);
      Eigen::Vector2i p_ind = polarToHistogramIndex(p_pol, ALPHA_RES / SCALE_FACTOR);
      

      // only remember point if it's in a cell not previously populated by complete_cloud, as well as outside FOV and
      // 'young' enough
      if (histogram_points_counter(p_ind.y(), p_ind.x()) < min_num_points_per_cell && xyzi.intensity < max_age &&
          !pointInsideFOV(fov, p_pol_fcu)) {
        final_cloud.points.push_back(toXYZI(toEigen(xyzi), xyzi.intensity + elapsed_s));

        // to indicate that this cell now has a point
        histogram_points_counter(p_ind.y(), p_ind.x()) = min_num_points_per_cell;
      }
    }
  }
  //std::cout << "2222:" << final_cloud.points.size() << std::endl;

  final_cloud.header.stamp = complete_cloud[0].header.stamp;
  final_cloud.header.frame_id = complete_cloud[0].header.frame_id;
  final_cloud.height = 1;  
  final_cloud.width = final_cloud.points.size();
}

/**
 * @brief 转换缓冲区线程函数
 *
 * 该函数从tf中获取转换关系并将其存储在缓冲区中。
 *
 * 在一个循环中，使用互斥锁保护缓冲区，然后遍历缓冲区中的每一对帧。
 * 如果tf监听器能够在这两个帧之间进行转换，则尝试查找转换关系并将其插入到tf缓冲区中。
 * 如果查找转换关系时发生异常，则记录错误信息。
 * 每次循环后，线程将休眠50毫秒。
 */
void Distance_to_fcu::transformBufferThread() {
  // grab transforms from tf and store them into the buffer
  while (!should_exit_) {
    {
      //ROS_INFO("transformBufferThread RUNNING");
      std::lock_guard<std::mutex> guard(buffered_transforms_mutex_);
      for (auto const& frame_pair : buffered_transforms_) {
        tf::StampedTransform transform;

        if (tf_listener_->canTransform(frame_pair.second, frame_pair.first, ros::Time(0))) {
          try {
            tf_listener_->lookupTransform(frame_pair.second, frame_pair.first, ros::Time(0), transform);
            tf_buffer_.insertTransform(frame_pair.first, frame_pair.second, transform);
          } catch (tf::TransformException& ex) {
            ROS_ERROR("Received an exception trying to get transform: %s", ex.what());
          }
        }
      }
      tf_buffer_cv_.notify_all();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}


/**
 * @brief 初始化Distance_to_fcu类
 *
 * 订阅ROS话题，发布ROS话题，初始化相机订阅者，并启动转换缓冲区线程和发布更新线程。
 */
void Distance_to_fcu::init()
{
    pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("mavros/local_position/pose", 1, &Distance_to_fcu::positionCallback, this);
    mavros_obstacle_distance_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/mavros/obstacle/send", 10);
    initializeCameraSubscribers(camera_topics);
    tf_listener_ = new tf::TransformListener(ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), true);
    worker_tf_listener = std::thread(&Distance_to_fcu::transformBufferThread, this);
    pub_update_thread_ = std::thread(&Distance_to_fcu::pubUpdateThread, this);
    
}




/**
 * @brief 获取已转换的点云数量
 *
 * 遍历所有相机，统计已转换的点云数量。
 *
 * @return 已转换的点云数量
 */
size_t Distance_to_fcu::numTransformedClouds() {
  size_t num_transformed_clouds = 0;
  for (size_t i = 0; i < cameras_.size(); i++) {

    std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_[i].camera_mutex_));
    if (cameras_[i].transformed_) num_transformed_clouds++;
  }
  return num_transformed_clouds;
}


/**
 * @brief 更新线程函数
 *
 * 这是一个更新线程函数，用于更新点云数据并发布更新后的点云信息。
 *
 * 在循环中，首先检查是否应该退出线程，如果应该退出则直接跳出循环。
 * 接着，使用一个条件变量等待直到变换后的点云数量与相机数量匹配，或者线程应该退出。
 * 
 * 首先，调整原始点云向量的大小以匹配相机数量。
 * 然后，遍历所有相机，锁定每个相机的互斥锁，并尝试交换原始点云向量和相机的变换点云。
 * 如果交换成功，将相机的变换点云清空，并将相机的变换标志设置为false。
 * 同时，更新视野帧信息。
 *
 * 更新点云数据后，计算自上次处理点云以来经过的时间，并调用processPointcloud函数处理点云。
 * 处理完成后，更新最后处理点云的时间，并调用create2DObstacleRepresentation函数创建二维障碍物表示。
 * 最后，调用publishLaserScan函数发布激光扫描信息，并输出"Publishing"信息。
 *
 * 循环结束后，线程将休眠50毫秒，然后继续下一次循环。
 */
void Distance_to_fcu::pubUpdateThread() {
  while (!should_exit_) {
    //ROS_INFO("Pub_update running");

    while ((cameras_.size() == 0 || cameras_.size() != numTransformedClouds()) && !should_exit_) {
      std::unique_lock<std::mutex> lock(transformed_cloud_mutex_);
      transformed_cloud_cv_.wait_for(lock, std::chrono::milliseconds(1000));
    }

    if (should_exit_) break;

    {
        // update the point cloud
      original_cloud_vector_.resize(cameras_.size());
      for (size_t i = 0; i < cameras_.size(); ++i) {
        std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_[i].camera_mutex_));
        try {
          std::swap(original_cloud_vector_[i], cameras_[i].transformed_cloud_);
          cameras_[i].transformed_cloud_.clear();
          cameras_[i].transformed_ = false;
          if (i < fov_fcu_frame_.size()) {
            fov_fcu_frame_[i] = cameras_[i].fov_fcu_frame_;
          } else {
            fov_fcu_frame_.push_back(cameras_[i].fov_fcu_frame_);
          }
        } catch (tf::TransformException& ex) {
          ROS_ERROR("Received an exception trying to transform a pointcloud: %s", ex.what());
        }
      }

      float elapsed_since_last_processing = static_cast<float>((ros::Time::now() - last_pointcloud_process_time_).toSec());
      processPointcloud(final_cloud_, original_cloud_vector_, fov_fcu_frame_, yaw_fcu_frame_deg_, pitch_fcu_frame_deg_,
                      position_, min_sensor_range_, max_sensor_range_, max_point_age_s_, elapsed_since_last_processing,
                      min_num_points_per_cell_);
      last_pointcloud_process_time_ = ros::Time::now();
      create2DObstacleRepresentation(param_cp_dist_ > 0);
      publishLaserScan();
      ROS_INFO("Publishing");

    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

}

int main(int argc, char** argv) {
    
  ros::init(argc, argv, "dis2fcu");
  avoidance::Distance_to_fcu dis2fcu;
  dis2fcu.init();

  ros::spin();
  

  return 0;
}

