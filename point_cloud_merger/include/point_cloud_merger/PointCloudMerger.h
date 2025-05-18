#ifndef POINT_CLOUD_MERGER_H
#define POINT_CLOUD_MERGER_H
#define PCL_NO_PRECOMPILE

#include <frontend_utils/CommonStructs.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>

// 定义点结构
struct RSPointXYZIRT {
  PCL_ADD_POINT4D;                // x, y, z, padding
  float intensity;                // 激光强度
  uint16_t ring;                  // 激光通道号
  double timestamp;               // 时间戳（单位：秒）
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen 内存对齐要求
} EIGEN_ALIGN16;

// 注册该点类型到 PCL 中
POINT_CLOUD_REGISTER_POINT_STRUCT(
    RSPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint16_t, ring, ring)(double, timestamp, timestamp))

struct VeloXYZIRT {
  PCL_ADD_POINT4D; // x, y, z
  float intensity;
  uint16_t ring;
  float time; // 通常是相对时间，单位：秒
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    VeloXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint16_t, ring, ring)(float, time, time))

void ConvertRSToVelo(const pcl::PointCloud<RSPointXYZIRT>& in_cloud,
                     pcl::PointCloud<VeloXYZIRT>& out_cloud,
                     double frame_start_time_sec);

void ConvertVeloToRS(const pcl::PointCloud<VeloXYZIRT>& in_cloud,
                     pcl::PointCloud<RSPointXYZIRT>& out_cloud,
                     double frame_start_time_sec);

// 接收的数据类型（RS雷达格式）
using RSPoint = RSPointXYZIRT;
using RSCloud = pcl::PointCloud<RSPoint>;

// 发布的数据类型（Velodyne格式）
using VeloPoint = VeloXYZIRT;
using VeloCloud = pcl::PointCloud<VeloPoint>;

class PointCloudMerger {
public:
  PointCloudMerger();
  ~PointCloudMerger();

  bool Initialize(const ros::NodeHandle& n);

  // ✅ 接收使用 RS 格式
  typedef message_filters::Subscriber<RSCloud>* MessageFilterSub;

  typedef message_filters::sync_policies::ApproximateTime<RSCloud, RSCloud>
      TwoPcldSyncPolicy;

  typedef message_filters::sync_policies::
      ApproximateTime<RSCloud, RSCloud, RSCloud>
          ThreePcldSyncPolicy;

  typedef message_filters::Synchronizer<TwoPcldSyncPolicy> TwoPcldSynchronizer;
  typedef message_filters::Synchronizer<ThreePcldSyncPolicy>
      ThreePcldSynchronizer;

private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);
  bool CreatePublishers(const ros::NodeHandle& n);

  void OnePointCloudCallback(const RSCloud::ConstPtr& a);
  void TwoPointCloudCallback(const RSCloud::ConstPtr& a,
                             const RSCloud::ConstPtr& b);
  void ThreePointCloudCallback(const RSCloud::ConstPtr& a,
                               const RSCloud::ConstPtr& b,
                               const RSCloud::ConstPtr& c);

  // 发布为 Velo 格式
  void PublishMergedPointCloud(const RSCloud::Ptr& rs_cloud,
                               double frame_start_time);

  void UpdateMinMaxTimestamp(const RSCloud::Ptr& cloud,
                             double& min_time,
                             double& max_time);

  void TransformAndShiftRing(const RSCloud& input,
                             RSCloud& output,
                             const Eigen::Matrix4f& tf,
                             uint16_t ring_base);
  // -----------------------------------------------------------------------
  std::string name_;

  // 添加到 private 成员变量
  std::vector<Eigen::Matrix4f> extrinsics_; // 外参变换列表

  ros::Publisher merged_pcld_pub_;

  int number_of_velodynes_;

  bool b_use_random_filter_;
  double decimate_percentage_;

  bool b_use_radius_filter_;
  double radius_;
  unsigned int radius_knn_;

  int pcld_queue_size_{
      10}; // Approximate time policy queue size to synchronize point clouds

  MessageFilterSub pcld0_sub_;
  MessageFilterSub pcld1_sub_;
  MessageFilterSub pcld2_sub_;

  ros::NodeHandle nl_;
  ros::Subscriber standard_pcld_sub_;

  // TODO: Reduce ----------------------------------------------------------

  std::unique_ptr<TwoPcldSynchronizer> pcld_synchronizer_2_;
  std::unique_ptr<ThreePcldSynchronizer> pcld_synchronizer_3_;

  // -----------------------------------------------------------------------

  /*
  Failure detection --------------------------------------------------------
  Convention:
              - 0:TOP
              - 1:FRONT
              - 2:REAR
  */
  ros::Subscriber failure_detection_sub_;
  ros::Subscriber resurrection_detection_sub_;
  void FailureDetectionCallback(const std_msgs::Int8& sensor_id);
  void ResurrectionDetectionCallback(const std_msgs::Int8& sensor_id);
  int number_of_active_devices_;
  std::map<int, MessageFilterSub> id_to_sub_map_;
  std::vector<int> alive_keys_;
  message_filters::Connection two_sync_connection_;
  message_filters::Connection three_sync_connection_;
  // ------------------------------------------------------------------------
};

#endif
