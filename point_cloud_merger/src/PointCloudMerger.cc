#include <point_cloud_merger/PointCloudMerger.h>

namespace pu = parameter_utils;

void ConvertRSToVelo(const pcl::PointCloud<RSPointXYZIRT>& in_cloud,
                     pcl::PointCloud<VeloXYZIRT>& out_cloud,
                     double frame_start_time_sec) {
  out_cloud.clear();
  out_cloud.reserve(in_cloud.size());

  for (const auto& pt : in_cloud.points) {
    VeloXYZIRT out_pt;
    out_pt.x = pt.x;
    out_pt.y = pt.y;
    out_pt.z = pt.z;
    out_pt.intensity = pt.intensity;
    out_pt.ring = pt.ring;
    out_pt.time = static_cast<float>(pt.timestamp - frame_start_time_sec);
    out_cloud.push_back(out_pt);
  }
}

void ConvertVeloToRS(const pcl::PointCloud<VeloXYZIRT>& in_cloud,
                     pcl::PointCloud<RSPointXYZIRT>& out_cloud,
                     double frame_start_time_sec) {
  out_cloud.clear();
  out_cloud.reserve(in_cloud.size());

  for (const auto& pt : in_cloud.points) {
    RSPointXYZIRT out_pt;
    out_pt.x = pt.x;
    out_pt.y = pt.y;
    out_pt.z = pt.z;
    out_pt.intensity = pt.intensity;
    out_pt.ring = pt.ring;
    out_pt.timestamp = static_cast<double>(pt.time) + frame_start_time_sec;
    out_cloud.push_back(out_pt);
  }
}

PointCloudMerger::PointCloudMerger()
  : b_use_random_filter_(false), b_use_radius_filter_(false) {}

PointCloudMerger::~PointCloudMerger() {}

bool PointCloudMerger::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudMerger");
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }
  if (!CreatePublishers(n)) {
    ROS_ERROR("%s: Failed to create publishers.", name_.c_str());
    return false;
  }
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  number_of_active_devices_ = number_of_velodynes_;
  return true;
}

bool PointCloudMerger::LoadParameters(const ros::NodeHandle& n) {
  if (!pu::Get("merging/number_of_velodynes", number_of_velodynes_))
    return false;
  if (!pu::Get("merging/decimate_percentage", decimate_percentage_))
    return false;
  if (!pu::Get("merging/b_use_random_filter", b_use_random_filter_))
    return false;
  if (!pu::Get("merging/b_use_radius_filter", b_use_radius_filter_))
    return false;
  if (!pu::Get("merging/radius", radius_))
    return false;
  if (!pu::Get("merging/radius_knn", radius_knn_))
    return false;

  for (int i = 0; i < number_of_velodynes_; ++i) {
    std::string key = "merging/ext_lidar" + std::to_string(i + 1);
    std::vector<double> ext;
    if (!pu::Get(key, ext) || ext.size() != 16) {
      ROS_ERROR("Failed to load %s or wrong size (need 16 elements)",
                key.c_str());
      return false;
    }
    Eigen::Matrix4f tf;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        tf(r, c) = static_cast<float>(ext[r * 4 + c]);
    extrinsics_.push_back(tf);
  }
  return true;
}

bool PointCloudMerger::RegisterCallbacks(const ros::NodeHandle& n) {
  nl_ = ros::NodeHandle(n);
  failure_detection_sub_ =
      nl_.subscribe("failure_detection",
                    10,
                    &PointCloudMerger::FailureDetectionCallback,
                    this);
  resurrection_detection_sub_ =
      nl_.subscribe("resurrection_detection",
                    10,
                    &PointCloudMerger::ResurrectionDetectionCallback,
                    this);
  pcld0_sub_ = new message_filters::Subscriber<RSCloud>(nl_, "pcld0", 10);
  pcld1_sub_ = new message_filters::Subscriber<RSCloud>(nl_, "pcld1", 10);
  id_to_sub_map_.insert({0, pcld0_sub_});
  id_to_sub_map_.insert({1, pcld1_sub_});
  alive_keys_ = {0, 1};

  if (number_of_velodynes_ == 2) {
    ROS_INFO("PointCloudMerger - 2 VLPs merging requested");
    pcld_synchronizer_2_ = std::make_unique<TwoPcldSynchronizer>(
        TwoPcldSyncPolicy(pcld_queue_size_), *pcld0_sub_, *pcld1_sub_);
    two_sync_connection_ = pcld_synchronizer_2_->registerCallback(
        &PointCloudMerger::TwoPointCloudCallback, this);
  } else if (number_of_velodynes_ == 3) {
    ROS_INFO("PointCloudMerger - 3 VLPs merging requested");
    pcld2_sub_ = new message_filters::Subscriber<RSCloud>(nl_, "pcld2", 10);
    pcld_synchronizer_3_ = std::make_unique<ThreePcldSynchronizer>(
        ThreePcldSyncPolicy(pcld_queue_size_),
        *pcld0_sub_,
        *pcld1_sub_,
        *pcld2_sub_);
    three_sync_connection_ = pcld_synchronizer_3_->registerCallback(
        &PointCloudMerger::ThreePointCloudCallback, this);
    alive_keys_.push_back(2);
    id_to_sub_map_.insert({2, pcld2_sub_});
  } else {
    ROS_WARN("PointCloudMerger - number_of_velodynes_ !=2 and !=3");
    return false;
  }
  return true;
}

bool PointCloudMerger::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  merged_pcld_pub_ =
      nl.advertise<sensor_msgs::PointCloud2>("combined_point_cloud", 10, false);
  return true;
}

void PointCloudMerger::OnePointCloudCallback(const RSCloud::ConstPtr& a) {
  RSCloud::Ptr cloud(new RSCloud(*a));

  double min_time = std::numeric_limits<double>::max();
  double max_time = std::numeric_limits<double>::lowest();
  UpdateMinMaxTimestamp(cloud, min_time, max_time);

  ROS_INFO_STREAM("OneCloud timestamp range: min="
                  << min_time << ", max=" << max_time
                  << ", span=" << (max_time - min_time) << " s");

  if (b_use_random_filter_) {
    const int n_points =
        static_cast<int>((1.0 - decimate_percentage_) * cloud->size());
    pcl::RandomSample<RSPoint> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(cloud);
    random_filter.filter(*cloud);
  }

  if (b_use_radius_filter_) {
    pcl::RadiusOutlierRemoval<RSPoint> rad;
    rad.setInputCloud(cloud);
    rad.setRadiusSearch(radius_);
    rad.setMinNeighborsInRadius(radius_knn_);
    rad.filter(*cloud);
  }

  PublishMergedPointCloud(cloud, min_time);
}

void PointCloudMerger::TwoPointCloudCallback(const RSCloud::ConstPtr& a,
                                             const RSCloud::ConstPtr& b) {
  RSCloud::Ptr a_trans(new RSCloud), b_trans(new RSCloud);
  // 替代 pcl::transformPointCloud，加 ring 偏移
  TransformAndShiftRing(*a, *a_trans, extrinsics_[0], 0); // a: ring 0-4
  TransformAndShiftRing(*b, *b_trans, extrinsics_[1], 5); // b: ring 5-9

  double min_time = std::numeric_limits<double>::max();
  double max_time = std::numeric_limits<double>::lowest();
  UpdateMinMaxTimestamp(a_trans, min_time, max_time);
  UpdateMinMaxTimestamp(b_trans, min_time, max_time);

  RSCloud::Ptr sum(new RSCloud(*a_trans + *b_trans));

  ROS_INFO_STREAM("TwoCloud timestamp range: min="
                  << min_time << ", max=" << max_time
                  << ", span=" << (max_time - min_time) << " s");

  if (b_use_random_filter_) {
    const int n_points =
        static_cast<int>((1.0 - decimate_percentage_) * sum->size());
    pcl::RandomSample<RSPoint> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(sum);
    random_filter.filter(*sum);
  }

  if (b_use_radius_filter_) {
    pcl::RadiusOutlierRemoval<RSPoint> rad;
    rad.setInputCloud(sum);
    rad.setRadiusSearch(radius_);
    rad.setMinNeighborsInRadius(radius_knn_);
    rad.filter(*sum);
  }

  PublishMergedPointCloud(sum, min_time);
}

void PointCloudMerger::ThreePointCloudCallback(const RSCloud::ConstPtr& a,
                                               const RSCloud::ConstPtr& b,
                                               const RSCloud::ConstPtr& c) {
  RSCloud::Ptr a_trans(new RSCloud), b_trans(new RSCloud), c_trans(new RSCloud);

  // 替代 pcl::transformPointCloud，加 ring 偏移
  TransformAndShiftRing(*a, *a_trans, extrinsics_[0], 0);  // a: ring 0-4
  TransformAndShiftRing(*b, *b_trans, extrinsics_[1], 5);  // b: ring 5-9
  TransformAndShiftRing(*c, *c_trans, extrinsics_[2], 10); // c: ring 10-14

  double min_time = std::numeric_limits<double>::max();
  double max_time = std::numeric_limits<double>::lowest();

  UpdateMinMaxTimestamp(a_trans, min_time, max_time);
  UpdateMinMaxTimestamp(b_trans, min_time, max_time);
  UpdateMinMaxTimestamp(c_trans, min_time, max_time);

  RSCloud::Ptr sum(new RSCloud(*a_trans + *b_trans + *c_trans));

  ROS_INFO_STREAM("ThreeCloud timestamp range: min=" << min_time
                                                     << ", max=" << max_time);

  if (b_use_random_filter_) {
    const int n_points =
        static_cast<int>((1.0 - decimate_percentage_) * sum->size());
    pcl::RandomSample<RSPoint> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(sum);
    random_filter.filter(*sum);
  }
  if (b_use_radius_filter_) {
    pcl::RadiusOutlierRemoval<RSPoint> rad;
    rad.setInputCloud(sum);
    rad.setRadiusSearch(radius_);
    rad.setMinNeighborsInRadius(radius_knn_);
    rad.filter(*sum);
  }
  PublishMergedPointCloud(sum, min_time);
}

void PointCloudMerger::PublishMergedPointCloud(const RSCloud::Ptr& rs_cloud,
                                               double frame_start_time) {
  VeloCloud::Ptr velo_cloud(new VeloCloud);
  ConvertRSToVelo(*rs_cloud, *velo_cloud, frame_start_time);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*velo_cloud, msg_out);
  msg_out.header.frame_id = "velodyne";
  msg_out.header.stamp = ros::Time(frame_start_time); // 使用传入时间
  merged_pcld_pub_.publish(msg_out);
}

void PointCloudMerger::FailureDetectionCallback(
    const std_msgs::Int8& sensor_id) {
  ROS_INFO("PointCloudMerger - Received failure detection of sensor %d",
           sensor_id.data);
  alive_keys_.erase(
      std::remove(alive_keys_.begin(), alive_keys_.end(), sensor_id.data),
      alive_keys_.end());
  number_of_active_devices_ = alive_keys_.size();
  if (number_of_active_devices_ == 2) {
    three_sync_connection_.disconnect();
    pcld_synchronizer_2_ = std::make_unique<TwoPcldSynchronizer>(
        TwoPcldSyncPolicy(pcld_queue_size_),
        *id_to_sub_map_[alive_keys_[0]],
        *id_to_sub_map_[alive_keys_[1]]);
    two_sync_connection_ = pcld_synchronizer_2_->registerCallback(
        &PointCloudMerger::TwoPointCloudCallback, this);
  } else if (number_of_active_devices_ == 1) {
    two_sync_connection_.disconnect();
    auto topic = "pcld" + std::to_string(alive_keys_[0]);
    standard_pcld_sub_ =
        nl_.subscribe(topic, 1, &PointCloudMerger::OnePointCloudCallback, this);
  } else if (number_of_active_devices_ == 0) {
    ROS_ERROR("PointCloudMerger - No active lidar sensors");
    standard_pcld_sub_.shutdown();
  }
}

void PointCloudMerger::ResurrectionDetectionCallback(
    const std_msgs::Int8& sensor_id) {
  ROS_INFO("PointCloudMerger - Received resurrection detection of sensor %d",
           sensor_id.data);
  alive_keys_.push_back(sensor_id.data);
  number_of_active_devices_ = alive_keys_.size();
  if (number_of_active_devices_ == 3) {
    two_sync_connection_.disconnect();
    pcld_synchronizer_3_ = std::make_unique<ThreePcldSynchronizer>(
        ThreePcldSyncPolicy(pcld_queue_size_),
        *id_to_sub_map_[alive_keys_[0]],
        *id_to_sub_map_[alive_keys_[1]],
        *id_to_sub_map_[alive_keys_[2]]);
    three_sync_connection_ = pcld_synchronizer_3_->registerCallback(
        &PointCloudMerger::ThreePointCloudCallback, this);
  } else if (number_of_active_devices_ == 2) {
    standard_pcld_sub_.shutdown();
    pcld_synchronizer_2_ = std::make_unique<TwoPcldSynchronizer>(
        TwoPcldSyncPolicy(pcld_queue_size_),
        *id_to_sub_map_[alive_keys_[0]],
        *id_to_sub_map_[alive_keys_[1]]);
    two_sync_connection_ = pcld_synchronizer_2_->registerCallback(
        &PointCloudMerger::TwoPointCloudCallback, this);
  } else if (number_of_active_devices_ == 1) {
    auto topic = "pcld" + std::to_string(alive_keys_[0]);
    standard_pcld_sub_ =
        nl_.subscribe(topic, 1, &PointCloudMerger::OnePointCloudCallback, this);
  }
}

void PointCloudMerger::UpdateMinMaxTimestamp(const RSCloud::Ptr& cloud,
                                             double& min_time,
                                             double& max_time) {
  for (const auto& pt : cloud->points) {
    if (std::isfinite(pt.timestamp)) {
      min_time = std::min(min_time, pt.timestamp);
      max_time = std::max(max_time, pt.timestamp);
    }
  }
}

void PointCloudMerger::TransformAndShiftRing(const RSCloud& input,
                                             RSCloud& output,
                                             const Eigen::Matrix4f& tf,
                                             uint16_t ring_base) {
  output.clear();
  output.reserve(input.size());

  for (const auto& pt : input.points) {
    RSPointXYZIRT pt_out;
    Eigen::Vector4f p_in(pt.x, pt.y, pt.z, 1.0f);
    Eigen::Vector4f p_out = tf * p_in;

    pt_out.x = p_out[0];
    pt_out.y = p_out[1];
    pt_out.z = p_out[2];
    pt_out.intensity = pt.intensity;
    pt_out.timestamp = pt.timestamp;
    pt_out.ring = pt.ring + ring_base;

    output.push_back(pt_out);
  }
}

template class pcl::RandomSample<RSPointXYZIRT>;
template class pcl::RadiusOutlierRemoval<RSPointXYZIRT>;
template class pcl::PCLBase<RSPointXYZIRT>;