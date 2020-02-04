#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

using namespace sensor_msgs;
using namespace message_filters;

int maximum_queue_size_;
std::string output_frame_;
bool approximate_sync_;

ros::Publisher pub_output_;

long int cnt = 0;

// Reference Frames
std::string robot_frame_;
std::string ouster_frame_;
std::string up_pico_flexx_frame_;
std::string down_pico_flexx_frame_;

// TF Listener
boost::shared_ptr<tf::TransformListener> ouster_listener;
tf::StampedTransform ouster_transform;
tf::Matrix3x3 ouster_m_euler;
tf::Matrix3x3 ouster_m_inv_euler;
double ouster_roll;
double ouster_pitch;
double ouster_yaw;

boost::shared_ptr<tf::TransformListener> up_pico_flexx_listener;
tf::StampedTransform up_pico_flexx_transform;
tf::Matrix3x3 up_pico_flexx_m_euler;
tf::Matrix3x3 up_pico_flexx_m_inv_euler;
double up_pico_flexx_roll;
double up_pico_flexx_pitch;
double up_pico_flexx_yaw;

boost::shared_ptr<tf::TransformListener> down_pico_flexx_listener;
tf::StampedTransform down_pico_flexx_transform;
tf::Matrix3x3 down_pico_flexx_m_euler;
tf::Matrix3x3 down_pico_flexx_m_inv_euler;
double down_pico_flexx_roll;
double down_pico_flexx_pitch;
double down_pico_flexx_yaw;


void callback(const PointCloud2::ConstPtr &cloud_ouster_ros, const PointCloud2::ConstPtr &cloud_up_pico_flexx_ros, const PointCloud2::ConstPtr &cloud_down_pico_flexx_ros)
{

  std::cout << "Entering callback" << std::endl;


  pcl::PointCloud<pcl::PointXYZI> cloud_ouster;
  pcl::fromROSMsg(*cloud_ouster_ros, cloud_ouster);

  pcl::PointCloud<pcl::PointXYZI> cloud_up_pico_flexx;
  pcl::fromROSMsg(*cloud_up_pico_flexx_ros, cloud_up_pico_flexx);

  pcl::PointCloud<pcl::PointXYZI> cloud_down_pico_flexx;
  pcl::fromROSMsg(*cloud_down_pico_flexx_ros, cloud_down_pico_flexx);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_merge (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI> cloud_ouster_xfrm;
  pcl_ros::transformPointCloud(cloud_ouster, cloud_ouster_xfrm, ouster_transform);

  pcl::PointCloud<pcl::PointXYZI> cloud_up_pico_flexx_xfrm;
  pcl_ros::transformPointCloud(cloud_up_pico_flexx, cloud_up_pico_flexx_xfrm, up_pico_flexx_transform);

  pcl::PointCloud<pcl::PointXYZI> cloud_down_pico_flexx_xfrm;
  pcl_ros::transformPointCloud(cloud_down_pico_flexx, cloud_down_pico_flexx_xfrm, down_pico_flexx_transform);

  for (int i = 0; i < cloud_ouster_xfrm.points.size(); i++)
  {
      cloud_merge->push_back(cloud_ouster_xfrm.points[i]);
  }

  for (int i = 0; i < cloud_up_pico_flexx_xfrm.points.size(); i++)
  {
      cloud_merge->push_back(cloud_up_pico_flexx_xfrm.points[i]);
  }
  
  for (int i = 0; i < cloud_down_pico_flexx_xfrm.points.size(); i++)
  {
      cloud_merge->push_back(cloud_down_pico_flexx_xfrm.points[i]);
  }
          
  sensor_msgs::PointCloud2 cloud_merge_ros;
  pcl::toROSMsg(*cloud_merge, cloud_merge_ros);

  cloud_merge_ros.header.stamp = ros::Time::now();
  cloud_merge_ros.header.frame_id = "base_link";

  pub_output_.publish (boost::make_shared<PointCloud2> (cloud_merge_ros));

  cnt++;
  std::cout << "Published merged point cloud #:" << cnt << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_merge");
  ouster_listener.reset(new tf::TransformListener);
  down_pico_flexx_listener.reset(new tf::TransformListener);
  up_pico_flexx_listener.reset(new tf::TransformListener);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam ("max_queue_size", maximum_queue_size_);
  private_nh.getParam ("approximate_sync", approximate_sync_);
  private_nh.getParam ("output_frame", output_frame_);
  private_nh.getParam ("robot_frame", robot_frame_);
  private_nh.getParam ("ouster_frame", ouster_frame_);
  private_nh.getParam ("up_pico_flexx_frame", up_pico_flexx_frame_);
  private_nh.getParam ("down_pico_flexx_frame", down_pico_flexx_frame_);

  // Method to get the most recent (and most synchronized) messages from the three sensors below
    // http://wiki.ros.org/message_filters/ApproximateTime
    // http://wiki.ros.org/message_filters
  message_filters::Subscriber<PointCloud2> ouster_sub(nh, "os1_cloud_node/points", 5);
  message_filters::Subscriber<PointCloud2> up_pico_flexx_sub(nh, "up/up_camera/stream/1/cloud", 5);
  message_filters::Subscriber<PointCloud2> down_pico_flexx_sub(nh, "down/down_camera/stream/1/cloud", 5);

  typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ouster_sub, up_pico_flexx_sub, down_pico_flexx_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  pub_output_ = private_nh.advertise<PointCloud2> ("output", maximum_queue_size_);
  
  // Get Ouster static transforms     
  try
  {
      ouster_listener->waitForTransform(robot_frame_, ouster_frame_, ros::Time(0), ros::Duration(10.0) );
      ouster_listener->lookupTransform(robot_frame_, ouster_frame_, ros::Time(0), ouster_transform);
      ouster_m_euler.setRotation(ouster_transform.getRotation());
      int ouster_solution_number = 1;
      ouster_m_euler.getEulerYPR(ouster_yaw, ouster_pitch, ouster_roll, ouster_solution_number);
  }
  catch (tf::TransformException ex)
  {
      ROS_WARN("%s",ex.what());
  }

  // Get Up Pico Flexx static transforms     
  try
  {
      up_pico_flexx_listener->waitForTransform(robot_frame_, up_pico_flexx_frame_, ros::Time(0), ros::Duration(10.0) );
      up_pico_flexx_listener->lookupTransform(robot_frame_, up_pico_flexx_frame_, ros::Time(0), up_pico_flexx_transform);
      up_pico_flexx_m_euler.setRotation(up_pico_flexx_transform.getRotation());
      int up_pico_flexx_solution_number = 1;
      up_pico_flexx_m_euler.getEulerYPR(up_pico_flexx_yaw, up_pico_flexx_pitch, up_pico_flexx_roll, up_pico_flexx_solution_number);
  }
  catch (tf::TransformException ex)
  {
      ROS_WARN("%s",ex.what());
  }

  // Get Down Pico Flexx static transforms     
  try
  {
      down_pico_flexx_listener->waitForTransform(robot_frame_, down_pico_flexx_frame_, ros::Time(0), ros::Duration(10.0) );
      down_pico_flexx_listener->lookupTransform(robot_frame_, down_pico_flexx_frame_, ros::Time(0), down_pico_flexx_transform);
      down_pico_flexx_m_euler.setRotation(down_pico_flexx_transform.getRotation());
      int down_pico_flexx_solution_number = 1;
      down_pico_flexx_m_euler.getEulerYPR(down_pico_flexx_yaw, down_pico_flexx_pitch, down_pico_flexx_roll, down_pico_flexx_solution_number);
  }
  catch (tf::TransformException ex)
  {
      ROS_WARN("%s",ex.what());
  }



  ros::spin();

  return 0;
}

// This is a quick and dirty way to do this pcl merging. Less computationally intensive ways may involve:
// (1) Using a nodelet (though I tried and wasn't able to get it working)
// (2) Using the concatenate function: http://pointclouds.org/documentation/tutorials/concatenate_clouds.php
// (3) Using the concatenate class (this did not work for me because the Ouster and Pico Flexx have different point_steps and row_steps, 
  // corrupting the resulting concatenated point cloud, submitted issue): https://github.com/ros-perception/perception_pcl/issues/261#issuecomment-581000604
  // http://docs.ros.org/jade/api/pcl_ros/html/concatenate__data_8cpp_source.html
  // http://docs.ros.org/indigo/api/pcl_ros/html/classpcl__ros_1_1PointCloudConcatenateDataSynchronizer.html#abdb071d343c4db10da51355f04a9b9a4
