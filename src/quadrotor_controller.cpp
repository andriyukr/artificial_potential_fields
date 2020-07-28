#include <ros/ros.h>
#include <iostream>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

using namespace geometry_msgs;
using namespace Eigen;
using namespace std;

#define KINECT_HORIZONTAL_ANGLE	54 * M_PI / 180
#define KINECT_VERTICAL_ANGLE		54 * M_PI / 180
#define MINIMAL_FLIGHT_HEIGHT		0.2
#define ADMISSIBLE_HORIZONTAL_ERROR	0.2

Vector3f force_attractive;
Vector3f force_repulsive;
float gain_attractive;
float gain_repulsive;
float ro_max;
Vector3 orientation;
Twist velocity;
float height;
pcl::PointCloud<pcl::PointXYZRGB> cloud;
double timer = 0;
int clocks = 0;

float norm(Point p){
  return sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
}

void poseCallback(const PoseStampedConstPtr& msg){
  tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(orientation.x, orientation.y, orientation.z);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& c){  
  pcl::fromROSMsg(*c, cloud);
}

void heightCallback(const sensor_msgs::RangeConstPtr& msg){
  height = msg->range - 0.17; // because the sonar is placed on 17cm from the ground
}

void attractiveCallback(const geometry_msgs::TwistConstPtr& msg){
  force_attractive(0) += gain_attractive * msg->linear.x;
  force_attractive(1) += gain_attractive * msg->linear.y;
  force_attractive(2) += gain_attractive * msg->linear.z;

  velocity.angular.z += gain_attractive * msg->angular.z;
}

void computeVelocity(){
  // update command
  force_attractive(0) *= 0.9;
  force_attractive(1) *= 0.9;
  force_attractive(2) *= 0.9;
  velocity.angular.z *= 0.9;

  // compute the repulsive force
  force_repulsive(0) = 0;
  force_repulsive(1) = 0;
  force_repulsive(2) = 0;
  float ro = ro_max;
  Point obstacle;
  int cloud_size = cloud.width * cloud.height;
  for(int i = 0; i < cloud_size; ++i){
    obstacle.x = cloud.points[i].x;
    obstacle.y = cloud.points[i].y;
    obstacle.z = cloud.points[i].z - 0.18;
    float ro = norm(obstacle);
    if(ro < ro_max){
      float force = gain_repulsive * (1 / ro - 1 / ro_max) * 1  / pow(ro, 2);
      force_repulsive(0) += force * obstacle.x / (cloud_size + 1);
      force_repulsive(1) += force * obstacle.y / (cloud_size + 1);
      force_repulsive(2) += force * obstacle.z / (cloud_size + 1);
    }
  }
  /*if(height < ro_max)
    force_repulsive(2) +=  gain_repulsive * (1 / height - 1 / ro_max) * 1 / pow(height, 2) / (cloud_size + 1); // from sonar

  // turns to the flight direction
  /*if(sqrt(pow(position.x - goal.x, 2) + pow(position.y - goal.y, 2)) >= ADMISSIBLE_HORIZONTAL_ERROR){   
    float angle_difference = orientation.z - atan2(force_attractive(1) + force_repulsive(1), force_attractive(0) + force_repulsive(0));
    if(angle_difference >= -M_PI && angle_difference <= M_PI)
      velocity.angular.z = -gain_attractive * angle_difference;
    else
      if(angle_difference < -M_PI)
        velocity.angular.z = -gain_attractive * (angle_difference + 2 * M_PI);
      else
        velocity.angular.z = -gain_attractive * (angle_difference - 2 * M_PI);
  }*/

  // compute the velocity from attractive and repulsive forces
  float c = cos(orientation.z);
  float s = sin(orientation.z);
  Matrix3f rotation;
  rotation << c, s, 0, -s, c, 0, 0, 0, 1;

  Vector3f total_force = force_attractive + force_repulsive;
  velocity.linear.x = total_force(0);
  velocity.linear.y = total_force(1);
  velocity.linear.z = total_force(2);

  if(total_force(0) >= 0)
    velocity.linear.x = total_force(0);
  else
    velocity.linear.x = 0;
  if(abs(total_force(1)) <= tan(KINECT_HORIZONTAL_ANGLE / 2) * velocity.linear.x)
    velocity.linear.y = total_force(1);
  else{
    velocity.linear.y = 0;
  }
  if(total_force(2) <= 0 || total_force(2) <= tan(KINECT_VERTICAL_ANGLE / 2) * velocity.linear.x || height < MINIMAL_FLIGHT_HEIGHT)
    velocity.linear.z = total_force(2);
  else{
    velocity.linear.z = 0;
  }

  //cout << "height: " << height << endl << endl;
  //cout << "orientation:\n" << orientation << endl;
  cout << "velocity:\n" << velocity << endl;
}

void quit(int sig){
  cout << "*** Esecution time: " << timer / clocks << " ms" << endl;
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "quadrotor_controller");

  ros::NodeHandle node_handle;

  signal(SIGINT, quit);

  gain_attractive = atof(argv[1]);
  gain_repulsive = atof(argv[2]);
  ro_max = atof(argv[3]);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber points_subscriber = node_handle.subscribe("/camera/depth/points", 1, cloudCallback);
  ros::Subscriber pose_subscriber = node_handle.subscribe("/ground_truth_to_tf/pose", 1, poseCallback);
  ros::Subscriber height_subscriber = node_handle.subscribe("/sonar_height", 1, heightCallback);
  ros::Subscriber attractive_subscriber = node_handle.subscribe("/attractive_force", 1, attractiveCallback);
  
  ros::Publisher velocity_publisher = node_handle.advertise<Twist>("cmd_vel", 1);

  ros::Rate rate(100);
  while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
    
    clock_t start = clock();
    computeVelocity();
    timer += 1000 * (double)(clock() - start) / CLOCKS_PER_SEC;
    clocks++;
    
    velocity_publisher.publish(velocity);
  }

  return 0;
};
