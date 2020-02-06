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
bool approximate_sync_;

ros::Publisher pub_output_;

long int cnt = 0;

// Reference Frames
std::string output_frame_;
std::string in2_frame_;
std::string in3_frame_;

boost::shared_ptr<tf::TransformListener> tf_2_listener;
tf::StampedTransform tf_2;
tf::Matrix3x3 m_euler2;
tf::Matrix3x3 up_pico_flexx_m_inv_euler;
double roll2;
double pitch2;
double yaw2;

boost::shared_ptr<tf::TransformListener> tf_3_listener;
tf::StampedTransform tf_3;
tf::Matrix3x3 m_euler3;
tf::Matrix3x3 down_pico_flexx_m_inv_euler;
double roll3;
double pitch3;
double yaw3;


void callback(const PointCloud2::ConstPtr &in1_ros, const PointCloud2::ConstPtr &in2_ros, const PointCloud2::ConstPtr &in3_ros)
{


  pcl::PointCloud<pcl::PointXYZI> in1;
  pcl::fromROSMsg(*in1_ros, in1);

  pcl::PointCloud<pcl::PointXYZI> in2;
  pcl::fromROSMsg(*in2_ros, in2);

  pcl::PointCloud<pcl::PointXYZI> in3;
  pcl::fromROSMsg(*in3_ros, in3);

  pcl::PointCloud<pcl::PointXYZI> in2_t;
  pcl::PointCloud<pcl::PointXYZI> in3_t;

  pcl_ros::transformPointCloud (in2, in2_t, tf_2);
  pcl_ros::transformPointCloud (in3, in3_t, tf_3);

  std::cout << "Entering callback" << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr out (new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = 0; i < in2_t.points.size(); i++)
  {
      in1.push_back(in2_t.points[i]);
  }
  for (int i = 0; i < in3_t.points.size(); i++)
  {
      in1.push_back(in3_t.points[i]);
  }

  sensor_msgs::PointCloud2 cloud_merge_ros;
  pcl::toROSMsg(in1, cloud_merge_ros);

  cloud_merge_ros.header.stamp = ros::Time::now();
  cloud_merge_ros.header.frame_id = output_frame_;

  pub_output_.publish (boost::make_shared<PointCloud2> (cloud_merge_ros));

  cnt++;
  std::cout << "Published merged point cloud #:" << cnt << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_merge");
  tf_3_listener.reset(new tf::TransformListener);
  tf_2_listener.reset(new tf::TransformListener);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam ("max_queue_size", maximum_queue_size_);
  private_nh.getParam ("approximate_sync", approximate_sync_);
  private_nh.getParam ("output_frame", output_frame_);
  private_nh.getParam ("in2_frame", in2_frame_);
  private_nh.getParam ("in3_frame", in3_frame_);

  // Method to get the most recent (and most synchronized) messages from the three sensors below
    // http://wiki.ros.org/message_filters/ApproximateTime
    // http://wiki.ros.org/message_filters
  message_filters::Subscriber<PointCloud2> ouster_sub(nh, "in1", 5);
  message_filters::Subscriber<PointCloud2> up_pico_flexx_sub(nh, "in2", 5);
  message_filters::Subscriber<PointCloud2> down_pico_flexx_sub(nh, "in3", 5);

  typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ouster_sub, up_pico_flexx_sub, down_pico_flexx_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  pub_output_ = private_nh.advertise<PointCloud2> ("cloud", maximum_queue_size_);
  
  // Get Up Pico Flexx static transforms     
  try
  {
      tf_2_listener->waitForTransform(output_frame_, in2_frame_, ros::Time(0), ros::Duration(10.0) );
      tf_2_listener->lookupTransform(output_frame_, in2_frame_, ros::Time(0), tf_2);
      m_euler2.setRotation(tf_2.getRotation());
      int sol_num2 = 1;
      m_euler2.getEulerYPR(yaw2, pitch2, roll2, sol_num2);
  }
  catch (tf::TransformException ex)
  {
      ROS_WARN("%s",ex.what());
  }

  // Get Down Pico Flexx static transforms     
  try
  {
      tf_3_listener->waitForTransform(output_frame_, in3_frame_, ros::Time(0), ros::Duration(10.0) );
      tf_3_listener->lookupTransform(output_frame_, in3_frame_, ros::Time(0), tf_3);
      m_euler3.setRotation(tf_3.getRotation());
      int sol_num3 = 1;
      m_euler3.getEulerYPR(yaw3, pitch3, roll3, sol_num3);
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
