#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_A     0x64
#define KEYCODE_D     0x61
#define KEYCODE_W     0x77
#define KEYCODE_S     0x73
#define KEYCODE_LEFT  0x44
#define KEYCODE_RIGHT 0x43
#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_Q     0x71

using namespace std;

int kfd = 0;
struct termios cooked, raw;

class Teleop{
  private:
  ros::NodeHandle node_handle;
  ros::Publisher velocity_publisher;
  geometry_msgs::Quaternion command;

  public:
  Teleop(){
    command.x = 0;
    command.y = 0;
    command.z = 0;
    command.w = 0;

    velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/keyboard_command", 1);
  }

  ~Teleop(){
  }

  void keyLoop(){
    char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    while(true){
      // get the next event from the keyboard
      read(kfd, &c, 1);

      command.x = 0;
      command.y = 0;
      command.z = 0;
      command.w = 0;
      switch(c){
        case KEYCODE_D:
          command.w = 1;
          break;
        case KEYCODE_A:
          command.w = -1;
          break;
        case KEYCODE_W:
          command.z = 1;
          break;
        case KEYCODE_S:
          command.z = -1;
          break;
        case KEYCODE_LEFT:
          command.y = 1;
          break;
        case KEYCODE_RIGHT:
          command.y = -1;
          break;
        case KEYCODE_UP:
          command.x = 1;
          break;
        case KEYCODE_DOWN:
          command.x = -1;
          break;
      }
      velocity_publisher.publish(command);
    }

    return;
  }
};

void quit(int sig){
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "keyboard_teleop");

  cout << "Press key:\n";
  cout << "UP\t - move forward\n";
  cout << "DOWN\t - move backward\n";
  cout << "LEFT\t - move left\n";
  cout << "RIGHT\t - move right\n";
  cout << "W\t - move up\n";
  cout << "S\t - move down\n";
  cout << "A\t - rotate clockwise\n";
  cout << "D\t - rotate counterclockwise\n";

  Teleop teleop;

  signal(SIGINT, quit);

  teleop.keyLoop();

  ros::spin();

  return 0;
}
