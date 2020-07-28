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

#define KINECT_HORIZONTAL_ANGLE		54 * M_PI / 180
#define KINECT_VERTICAL_ANGLE		54 * M_PI / 180
#define MINIMAL_FLIGHT_HEIGHT		0.2
#define ADMISSIBLE_HORIZONTAL_ERROR	0.2

Point goal;
float gain_attractive;
float gain_repulsive;
float ro_max;
Point position;
Vector3 orientation;
Twist velocity;
float height;
pcl::PointCloud<pcl::PointXYZRGB> cloud;
double timer = 0;
int clocks = 0;

float distance1(Point p){
  return sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
}

float distance2(Point p1, Point p2){
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

void poseCallback(const PoseStampedConstPtr& msg){
  tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(orientation.x, orientation.y, orientation.z);

  position = msg->pose.position;
  position.z -= 0.18; // because the Kinect is placed on 18cm from the ground
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& c){  
  pcl::fromROSMsg(*c, cloud);
}

void heightCallback(const sensor_msgs::RangeConstPtr& msg){
  height = msg->range - 0.17; // because the sonar is placed on 17cm from the ground
}

void computeVelocity(){
  Vector3f force_attractive;
  Vector3f force_repulsive;

  // compute the attractive force
  float distance = distance2(position, goal);
  if(distance >= 1){
    force_attractive(0) = -gain_attractive * (position.x - goal.x) / distance;
    force_attractive(1) = -gain_attractive * (position.y - goal.y) / distance;
    force_attractive(2) = -gain_attractive * (position.z - goal.z) / distance;
  }
  else{
    force_attractive(0) = -gain_attractive * (position.x - goal.x);
    force_attractive(1) = -gain_attractive * (position.y - goal.y);
    force_attractive(2) = -gain_attractive * (position.z - goal.z);
  }

  // compute the repulsive force
  force_repulsive(0) = 0;
  force_repulsive(1) = 0;
  force_repulsive(2) = 0;
  // if the quadrotor is far from the goal
  if(sqrt(pow(position.x - goal.x, 2) + pow(position.y - goal.y, 2)) >= ADMISSIBLE_HORIZONTAL_ERROR){
    float ro = ro_max;
    Point obstacle;
    int cloud_size = cloud.width * cloud.height;
    for(int i = 0; i < cloud_size; ++i){
      obstacle.x = cloud.points[i].x;
      obstacle.y = cloud.points[i].y;
      obstacle.z = cloud.points[i].z;
      float ro = distance1(obstacle);
      if(ro < ro_max){
        float force = gain_repulsive * (1 / ro - 1 / ro_max) * 1  / pow(ro, 2);
        force_repulsive(0) += force * obstacle.x;
        force_repulsive(1) += force * obstacle.y;
        force_repulsive(2) += force * obstacle.z / 1000;
      }
    }
    force_repulsive(2) +=  gain_repulsive * (1 / height - 1 / ro_max) * 1  / pow(height, 2) / 100; // from sonar
  }

  // turns to the flight direction
  if(sqrt(pow(position.x - goal.x, 2) + pow(position.y - goal.y, 2)) >= ADMISSIBLE_HORIZONTAL_ERROR){   
    float angle_difference = orientation.z - atan2(force_attractive(1) + force_repulsive(1), force_attractive(0) + force_repulsive(0));
    if(angle_difference >= -M_PI && angle_difference <= M_PI)
      velocity.angular.z = -gain_attractive * angle_difference;
    else
      if(angle_difference < -M_PI)
        velocity.angular.z = -gain_attractive * (angle_difference + 2 * M_PI);
      else
        velocity.angular.z = -gain_attractive * (angle_difference - 2 * M_PI);
  }
  // atan2() is not defined on the goal
  else{
    velocity.angular.z = 0;
  }

  // compute the velocity from attractive and repulsive forces
  float c = cos(orientation.z);
  float s = sin(orientation.z);
  Matrix3f rotation;
  rotation << c, s, 0, -s, c, 0, 0, 0, 1;

  Vector3f total_force = rotation * (force_attractive + force_repulsive);
  if(total_force(0) >= 0)
    velocity.linear.x = total_force(0);
  else
    velocity.linear.x = 0;
  if(abs(total_force(1)) <= tan(KINECT_HORIZONTAL_ANGLE / 2) * velocity.linear.x)
    velocity.linear.y = total_force(1);
  else{
    velocity.linear.x = 0;
    velocity.linear.y = 0;
  }
  if(total_force(2) <= tan(KINECT_VERTICAL_ANGLE / 2) * velocity.linear.x)
    velocity.linear.z = total_force(2);
  else{
    velocity.linear.x = 0;
    velocity.linear.z = total_force(2);
  }

  cout << "position:\n" << position << endl;
  //cout << "height: " << height << endl << endl;
  cout << "orientation:\n" << orientation << endl;
  cout << "force:\n" << total_force << endl;
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

  goal.x = atof(argv[1]);
  goal.y = atof(argv[2]);
  goal.z = atof(argv[3]);
  gain_attractive = atof(argv[4]);
  gain_repulsive = atof(argv[5]);
  ro_max = atof(argv[6]);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber points_subscriber = node_handle.subscribe("/camera/depth/points", 1, cloudCallback);
  ros::Subscriber pose_subscriber = node_handle.subscribe("/ground_truth_to_tf/pose", 1, poseCallback);
  ros::Subscriber height_subscriber = node_handle.subscribe("/sonar_height", 1, heightCallback);
  
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
