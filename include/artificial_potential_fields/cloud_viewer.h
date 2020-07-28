#include <iostream>
#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#define QUADROTOR_RADIUS    0.30

ros::Subscriber point_cloud_subscriber;                                                                             //                                                                             //
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());                              //
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));    //

void quit(int sig);
