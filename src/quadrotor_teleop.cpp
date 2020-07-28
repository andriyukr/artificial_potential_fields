#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace hector_quadrotor{
  class Teleop{
    private:
      ros::NodeHandle node_handle;
      ros::Subscriber joy_subscriber;
      ros::Publisher velocity_publisher;
      geometry_msgs::Twist velocity;

      struct Axis{
        int axis;
        double max;
      };

      struct Button{
        int button;
      };

      struct{
        Axis x;
        Axis y;
        Axis z;
        Axis yaw;
      } axes;

      struct{
        Button slow;
      } buttons;

      double slow_factor;

    public:
      Teleop(){
        ros::NodeHandle params("~");

        axes.x.axis = 0;
        axes.x.max = 2.0;
        axes.y.axis = 0;
        axes.y.max = 2.0;
        axes.z.axis = 0;
        axes.z.max = 2.0;
        axes.yaw.axis = 0;
        axes.yaw.max = 90.0 * M_PI / 180.0;
        buttons.slow.button = 0;
        slow_factor = 0.2;

        params.getParam("x_axis", axes.x.axis);
        params.getParam("y_axis", axes.y.axis);
        params.getParam("z_axis", axes.z.axis);
        params.getParam("yaw_axis", axes.yaw.axis);
        params.getParam("x_velocity_max", axes.x.max);
        params.getParam("y_velocity_max", axes.y.max);
        params.getParam("z_velocity_max", axes.z.max);
        params.getParam("yaw_velocity_max", axes.yaw.max);
        params.getParam("slow_button", buttons.slow.button);
        params.getParam("slow_factor", slow_factor);

        joy_subscriber = node_handle.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&Teleop::joyCallback, this, _1));
        velocity_publisher = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 10);
      }

      ~Teleop(){
        stop();
      }

      void joyCallback(const sensor_msgs::JoyConstPtr& joy){
        velocity.linear.x  = getAxis(joy, axes.x.axis)   * axes.x.max;
        velocity.linear.y  = getAxis(joy, axes.y.axis)   * axes.y.max;
        velocity.linear.z  = getAxis(joy, axes.z.axis)   * axes.z.max;
        velocity.angular.z = getAxis(joy, axes.yaw.axis) * axes.yaw.max;
        if(getButton(joy, buttons.slow.button)){
          velocity.linear.x  *= slow_factor;
          velocity.linear.y  *= slow_factor;
          velocity.linear.z  *= slow_factor;
          velocity.angular.z *= slow_factor;
        }
        velocity_publisher.publish(velocity);
      }

      sensor_msgs::Joy::_axes_type::value_type getAxis(const sensor_msgs::JoyConstPtr& joy, int axis){
        if(axis == 0)
          return 0;
        sensor_msgs::Joy::_axes_type::value_type sign = 1.0;
        if(axis < 0){
          sign = -1.0;
          axis = -axis;
        }
        if((size_t)axis > joy->axes.size())
          return 0;
        return sign * joy->axes[axis - 1];
      }

      sensor_msgs::Joy::_buttons_type::value_type getButton(const sensor_msgs::JoyConstPtr& joy, int button){
        if(button <= 0)
          return 0;
        if((size_t)button > joy->axes.size())
          return 0;
        return joy->buttons[button - 1];
      }

      void stop(){
        velocity = geometry_msgs::Twist();
        velocity_publisher.publish(velocity);
      }
  };
} // namespace hector_quadrotor

int main(int argc, char **argv){
  ros::init(argc, argv, "quadrotor_teleop");

  hector_quadrotor::Teleop teleop;
  ros::spin();

  return 0;
}
