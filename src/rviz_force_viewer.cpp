#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

using namespace Eigen;

Vector3f force;
Vector3f attractive;
Vector3f repulsive;
Vector3f position;

void forceCallback(const geometry_msgs::TwistStamped& msg){
    force << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
}

void attractiveCallback(const geometry_msgs::Vector3& msg){
    attractive << msg.x, msg.y, msg.z;
}

void repulsiveCallback(const geometry_msgs::Vector3& msg){
    repulsive << msg.x, msg.y, msg.z;
}

void poseCallback(const geometry_msgs::PoseStamped& pose_msg){
    position << pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z;
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "rviz_force_visualization");
  ros::NodeHandle node_handle;
  ros::Rate rate(100);

  ros::Subscriber force_publisher = node_handle.subscribe("/iris_rplidar/command_velocity", 1, forceCallback);
  ros::Subscriber attractive_publisher = node_handle.subscribe("/potential_fields/attractive", 1, attractiveCallback);
  ros::Subscriber repulsive_publisher = node_handle.subscribe("/potential_fields/repulsive", 1, repulsiveCallback);
  ros::Subscriber pose_subscriber = node_handle.subscribe("/mavros/local_position/pose", 1, poseCallback);

  ros::Publisher markers_publisher = node_handle.advertise<visualization_msgs::MarkerArray>("rviz_force_visualization_array", 1);

  while (ros::ok()){
      rate.sleep();
      ros::spinOnce();

    visualization_msgs::MarkerArray markers;
    markers.markers.resize(3);

    /* Total force */

    markers.markers[0].header.frame_id = "map";
    markers.markers[0].header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID. Any marker sent with the same namespace and id will overwrite the old one.
    markers.markers[0].ns = "total_force";
    markers.markers[0].id = 0;

    markers.markers[0].type = visualization_msgs::Marker::ARROW;
    markers.markers[0].action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame specified in the header
    markers.markers[0].pose.orientation.x = 0;
    markers.markers[0].pose.orientation.y = 0;
    markers.markers[0].pose.orientation.z = 0;
    markers.markers[0].pose.orientation.w = 1;

    markers.markers[0].points.resize(2);
    markers.markers[0].points[0].x = position(0);
    markers.markers[0].points[0].y = position(1);
    markers.markers[0].points[0].z = position(2);
    markers.markers[0].points[1].x = position(0) + force(0);
    markers.markers[0].points[1].y = position(1) + force(1);
    markers.markers[0].points[1].z = position(2) + force(2);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    markers.markers[0].scale.x = 0.1;
    markers.markers[0].scale.y = 0.2;
    markers.markers[0].scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    markers.markers[0].color.r = 0.0f;
    markers.markers[0].color.g = 1.0f;
    markers.markers[0].color.b = 0.0f;
    markers.markers[0].color.a = 1.0;

    markers.markers[0].lifetime = ros::Duration();

    /* Attractive force */

    markers.markers[1].header.frame_id = "map";
    markers.markers[1].header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID. Any marker sent with the same namespace and id will overwrite the old one.
    markers.markers[1].ns = "attractive_force";
    markers.markers[1].id = 0;

    markers.markers[1].type = visualization_msgs::Marker::ARROW;
    markers.markers[1].action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame specified in the header
    markers.markers[1].pose.orientation.x = 0;
    markers.markers[1].pose.orientation.y = 0;
    markers.markers[1].pose.orientation.z = 0;
    markers.markers[1].pose.orientation.w = 1;

    markers.markers[1].points.resize(2);
    markers.markers[1].points[0].x = position(0);
    markers.markers[1].points[0].y = position(1);
    markers.markers[1].points[0].z = position(2);
    markers.markers[1].points[1].x = position(0) + attractive(0);
    markers.markers[1].points[1].y = position(1) + attractive(1);
    markers.markers[1].points[1].z = position(2) + attractive(2);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    markers.markers[1].scale.x = 0.1;
    markers.markers[1].scale.y = 0.2;
    markers.markers[1].scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    markers.markers[1].color.r = 1.0f;
    markers.markers[1].color.g = 0.0f;
    markers.markers[1].color.b = 0.0f;
    markers.markers[1].color.a = 1.0;

    markers.markers[1].lifetime = ros::Duration();

    /* Repulsive force */

    markers.markers[2].header.frame_id = "map";
    markers.markers[2].header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID. Any marker sent with the same namespace and id will overwrite the old one.
    markers.markers[2].ns = "repulsive_force";
    markers.markers[2].id = 0;

    markers.markers[2].type = visualization_msgs::Marker::ARROW;
    markers.markers[2].action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame specified in the header
    markers.markers[2].pose.orientation.x = 0;
    markers.markers[2].pose.orientation.y = 0;
    markers.markers[2].pose.orientation.z = 0;
    markers.markers[2].pose.orientation.w = 1;

    markers.markers[2].points.resize(2);
    markers.markers[2].points[0].x = position(0);
    markers.markers[2].points[0].y = position(1);
    markers.markers[2].points[0].z = position(2);
    markers.markers[2].points[1].x = position(0) + repulsive(0);
    markers.markers[2].points[1].y = position(1) + repulsive(1);
    markers.markers[2].points[1].z = position(2) + repulsive(2);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    markers.markers[2].scale.x = 0.1;
    markers.markers[2].scale.y = 0.2;
    markers.markers[2].scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    markers.markers[2].color.r = 0.0f;
    markers.markers[2].color.g = 0.0f;
    markers.markers[2].color.b = 1.0f;
    markers.markers[2].color.a = 1.0;

    markers.markers[2].lifetime = ros::Duration();

    // Publish the marker
    markers_publisher.publish(markers);
  }
}
