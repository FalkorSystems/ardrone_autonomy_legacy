#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <ctype.h>

#define KEYCODE_YMINUS 'S' // S
#define KEYCODE_YPLUS  'D' // D
#define KEYCODE_XPLUS  'F' // F
#define KEYCODE_XMINUS 'A' // A
#define KEYCODE_ZMINUS 'K' // K
#define KEYCODE_ZPLUS  'J' // J
#define KEYCODE_YAWMINUS 'I' // I
#define KEYCODE_YAWPLUS 'L' // L
#define KEYCODE_HOVER 'Q' // Q

#define KEYCODE_LAND 'W' // W
#define KEYCODE_TAKEOFF 'E' // E
#define KEYCODE_RESET 'R' // R

#define SCALE 100

class TeleopArdrone
{
public:
  TeleopArdrone();
  void keyLoop();

private:
  
  ros::NodeHandle nh_;
  double linear_x_, linear_y_, linear_z_, angular_z_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher land_pub_;
  ros::Publisher reset_pub_;
  ros::Publisher takeoff_pub_;
};

TeleopArdrone::TeleopArdrone():
  linear_x_(0),
  linear_y_(0),
  linear_z_(0),
  angular_z_(0)
{
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  land_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/land", 1);
  takeoff_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  reset_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/reset", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ardrone_teleop_key");

  TeleopArdrone teleop_ardrone;

  signal(SIGINT,quit);

  teleop_ardrone.keyLoop();
  
  return(0);
}


void TeleopArdrone::keyLoop()
{
  char c;
  bool dirty=false;
  std_msgs::Empty empty_msg;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use ASDF to move the ardrone left/right/forward/back, HJKL");
  puts("to move the ardrone up/down, to yaw left/right");
  puts("Q to hover, W to land, and E to takeoff");

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", toupper(c));
  
    switch(toupper(c))
    {
      case KEYCODE_YMINUS:
        ROS_DEBUG("RIGHT\n");
	linear_y_ += -1.0;
        dirty = true;
        break;
      case KEYCODE_YPLUS:
        ROS_DEBUG("LEFT\n");
        linear_y_ += 1.0;
        dirty = true;
        break;
      case KEYCODE_XPLUS:
        ROS_DEBUG("FORWARD\n");
        linear_x_ += 1.0;
        dirty = true;
        break;
      case KEYCODE_XMINUS:
        ROS_DEBUG("BACK\n");
        linear_x_ += -1.0;
        dirty = true;
        break;
      case KEYCODE_ZMINUS:
	ROS_DEBUG("DOWN\n");
	linear_z_ += -1.0;
	dirty = true;
	break;
      case KEYCODE_ZPLUS:
	ROS_DEBUG("UP\n");
	linear_z_ += 1.0;
	dirty = true;
	break;
      case KEYCODE_YAWMINUS:
	ROS_DEBUG("YAW LEFT\n");
	angular_z_ += -1.0;
	dirty = true;
	break;
      case KEYCODE_YAWPLUS:
	ROS_DEBUG("YAW RIGHT\n");
	angular_z_ += 1.0;
	dirty = true;
	break;
      case KEYCODE_HOVER:
	ROS_DEBUG("HOVER\n");
	linear_x_ = linear_y_ = linear_z_ = angular_z_ = 0.0;
	dirty = true;
	break;
      case KEYCODE_TAKEOFF:
	ROS_DEBUG("TAKEOFF\n");
	takeoff_pub_.publish(empty_msg);
	break;
      case KEYCODE_LAND:
	ROS_DEBUG("LAND\n");
	land_pub_.publish(empty_msg);
	break;
      case KEYCODE_RESET:
	ROS_DEBUG("RESET\n");
	reset_pub_.publish(empty_msg);
	break;
    }
   

    geometry_msgs::Twist twist_msg;

    twist_msg.linear.x = linear_x_/SCALE;
    twist_msg.linear.y = linear_y_/SCALE;
    twist_msg.linear.z = linear_z_/SCALE;
    twist_msg.angular.z = angular_z_/SCALE;

    if(dirty ==true)
    {
      cmd_vel_pub_.publish(twist_msg);    
      dirty=false;
    }
  }


  return;
}



