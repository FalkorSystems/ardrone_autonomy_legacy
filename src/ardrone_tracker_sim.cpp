#include<ros/ros.h>
#include<geometry_msgs/Point.h>
#include<signal.h>
#include<termios.h>
#include<ctype.h>
#include<stdio.h>

class ArdroneTrackerSim
{
  ros::NodeHandle nh_;
  ros::Publisher point_pub_;

public:
  ArdroneTrackerSim()
  {
    point_pub_ = nh_.advertise<geometry_msgs::Point>( "/ardrone_tracker/found_point", 1 );
  }

  ~ArdroneTrackerSim()
  {
  }

  void publish( const geometry_msgs::Point &the_point )
  {
    // publish the point
    point_pub_.publish( the_point );
    printf( "published: %lf %lf %lf\n", the_point.x, the_point.y, the_point.z );
  }
};

int kfd = 0;
struct termios cooked, raw;

void quit( int sig )
{
  tcsetattr( kfd, TCSANOW, &cooked );
  ros::shutdown();
  exit( 0 );
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "ardrone_tracker_sim" );
  ArdroneTrackerSim atsim;
  signal( SIGINT, quit );

  tcgetattr( kfd, &cooked );
  memcpy( &raw, &cooked, sizeof( struct termios ) );
  raw.c_lflag &=~ (ICANON | ECHO );
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr( kfd, TCSANOW, &raw );

  geometry_msgs::Point the_point;

  the_point.x = 50;
  the_point.y = 50;
  the_point.z = 0;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use ASDF to move the target left/up/down/right");
  puts("Use X to lose the target, and Z to find it again" );

  char c;
  while(1)
    {
      if( read( kfd, &c, 1 ) < 0 )
	{
	  perror( "read():");
	  exit(-1);
	}

      switch( toupper(c) )
	{
	case 'X':
	  the_point.x = the_point.y = 0;
	  the_point.z = 1.0;
	  break;
        case 'A':
	  the_point.x -= 10;
	  break;
	case 'S':
	  the_point.y -= 10;
	  break;
	case 'D':
	  the_point.y += 10;
	  break;
	case 'F':
	  the_point.x += 10;
	  break;
	case 'Z':
	  the_point.x = the_point.y = 50;
	  the_point.z = 50;
	  break;
	}

      atsim.publish(the_point);
    }
  
  return( 0 );
}
