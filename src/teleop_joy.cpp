#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>

using namespace std;

ros::Publisher command_publisher;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
    geometry_msgs::Quaternion command;
    command.x = joy->axes[4];
    command.y = joy->axes[3];
    command.z = joy->axes[0];
    command.w = joy->axes[1];
    cout << "Joystick: " << command.x << "; " << command.y << "; " << command.z << "; " << command.w << endl;
    command_publisher.publish(command);
}

class TeleopJoy{
    private:
        ros::NodeHandle node_handle;
        ros::Subscriber joystick_subscriber;
        geometry_msgs::Quaternion command;

    public:
        TeleopJoy(){
            command.x = 0;
            command.y = 0;
            command.z = 0;
            command.w = 0;

            command_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/joystick_command", 1);

            joystick_subscriber = node_handle.subscribe("/joy", 10, joyCallback);
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "joystick_teleop");

    cout << "Joystick teleop running..." << endl;    

    TeleopJoy teleop_joy;

    ros::spin();
}
