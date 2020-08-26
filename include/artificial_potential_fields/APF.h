#include <ros/ros.h>
#include <iostream>
#include <signal.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <artificial_potential_fields/setAPFConfig.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

ros::Subscriber laser_subscriber;                  //
ros::Subscriber odometry_subscriber;
ros::Subscriber attractive_velocity_subscriber;
ros::Publisher force_publisher;                  //
ros::Publisher attractive_publisher;              //
ros::Publisher repulsive_publisher;                  //
ros::Publisher point_cloud_publisher;               //

Vector4f velocity_d;
Matrix3f R;                                         // rotation matrix from world frame to quadrotor frame
Matrix4f transformation;
geometry_msgs::Quaternion orientation;              //
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
laser_geometry::LaserProjection projector;
Vector3f position;
float height = 0;
float speed = 0;
float yaw = 0;

float k_repulsive = 4;
float eta_0 = 4;
float UAV_radius = 0.5;

class APF{
    public:
        APF(int, char**);
        ~APF();
        void run();

    private:
        double distance(Vector3f p);
};
