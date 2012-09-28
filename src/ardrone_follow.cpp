#include<ros/ros.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Twist.h>
#include<ardrone_autonomy/Navdata.h>
#include<ardrone_autonomy/LedAnim.h>
#include<std_msgs/Empty.h>
#include<math.h>

#define SCALE 5

class PidControl
{
  double setPointMax_;
  double setPointMin_;
  double Kp_;
  double Ti_;
  double Td_;

  double outputLimit_;

  double preError_;
  double integral_;

public:
  PidControl( double Kp, double Ti, double Td, double outputLimit ) :
    Kp_( Kp ), Ti_( Ti ), Td_( Td ), outputLimit_( outputLimit ),
    preError_( 0.0 ), integral_( 0.0 )
  {
  }

  void setSetPoint( double setPoint )
  {
    setPointMax_ = setPointMin_ = setPoint;
  }

  void setSetPointMax( double setPointMax )
  {
    setPointMax_ = setPointMax;
  }

  void setSetPointMin( double setPointMin )
  {
    setPointMin_ = setPointMin;
  }

  void setKp( double Kp )
  {
    Kp_ = Kp;
  };

  void setTd( double Td )
  {
    Td_ = Td;
  }

  void setTi( double Ti )
  {
    Ti_ = Ti;
  }

  double getOutput( double PV, ros::Duration Dt )
  {
    double error;

    if( PV < setPointMax_ && PV > setPointMin_ )
      error = 0;
    else
      {
	double errorFromMax = setPointMax_ - PV;
	double errorFromMin = setPointMin_ - PV;

	if( fabs( errorFromMax ) > fabs( errorFromMin ) )
	  error = errorFromMin;
	else
	  error = errorFromMax;
      }

    integral_ += error * Dt.toSec();
    double derivative = ( error - preError_ ) / Dt.toSec();
    preError_ = error;

    double TiRecip = ( Ti_ == 0.0 ) ? 0 : 1/Ti_;
    double output = Kp_ * ( error + TiRecip * integral_ + Td_ * derivative );

    double limitedOutput = fmin( fmax( output, -outputLimit_ ),
				 outputLimit_ );
    return( limitedOutput );
  }
};

class ArdroneFollow
{
  ros::NodeHandle nh_;
  geometry_msgs::Twist current_cmd_;
  ros::Subscriber navdata_;
  ros::Subscriber tracker_;
  ros::Publisher cmd_vel_;

  ros::Timer timer_;
  ros::Timer hoverTimer_;

  ros::Publisher land_pub_;
  ros::Publisher reset_pub_;
  ros::Publisher takeoff_pub_;

  geometry_msgs::Point found_point_;
  geometry_msgs::Point set_point_;

  PidControl xPid, yPid, zPid;

  ros::ServiceClient ledService_;
  int lastAnim_;

public:
  ArdroneFollow()
    : current_cmd_(),
      xPid( 0.020, 0.0, 0.0, 100.0 ), // pos
      yPid( 0.020, 0.0, 0.0, 100.0 ), // pos
      zPid( 0.500, 0.0, 0.0, 50.0 ), // pos
      lastAnim_( -1 )
  {
    navdata_ = nh_.subscribe( "ardrone/navdata", 1000, &ArdroneFollow::navdataCb, this );
    tracker_ = nh_.subscribe( "ardrone_tracker/found_point", 1, &ArdroneFollow::foundpointCb, this );
    cmd_vel_ = nh_.advertise<geometry_msgs::Twist>( "cmd_vel", 1 );
    timer_ = nh_.createTimer( ros::Duration( 0.1 ), &ArdroneFollow::timerCb, this );

    land_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/land", 1);
    takeoff_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
    reset_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/reset", 1);

    xPid.setSetPointMin( 40.0 );
    xPid.setSetPointMax( 60.0 );
    yPid.setSetPointMin( 50.0 );
    yPid.setSetPointMax( 65.0 );
    zPid.setSetPointMin( 40.0 );
    zPid.setSetPointMax( 60.0 );

    ledService_ = nh_.serviceClient<ardrone_autonomy::LedAnim>( "ardrone/setledanimation" );
  }

  ~ArdroneFollow()
  {
  }

  void setLedAnim( int type, int freq = 10 )
  {
    if ( lastAnim_ == type )
      return;

    ardrone_autonomy::LedAnim anim;
    lastAnim_ = anim.request.type = type;
    anim.request.freq = 10;
    anim.request.duration = 3600;

    ledService_.call( anim );
  }

  void takeoff( void )
  {
    takeoff_pub_.publish( std_msgs::Empty() );
    setLedAnim( 9 ); // RED_SNAKE
  }

  void land( void )
  {
    land_pub_.publish( std_msgs::Empty() );
  }

  void reset( void )
  {
    reset_pub_.publish( std_msgs::Empty() );
    setLedAnim( 6 ); // STANDARD
  }

  void navdataCb( const ardrone_autonomy::Navdata &navdata )
  {
    // For now ignore the nav data
  }

  void foundpointCb( const geometry_msgs::Point &found_point )
  {
    found_point_ = found_point;
  }

  void hover( void )
  {
    geometry_msgs::Twist hoverCmd;
    cmd_vel_.publish( hoverCmd );
  }

  void hoverCmdCb( const ros::TimerEvent &event )
  {
    this->hover();
  }

  void timerCb( const ros::TimerEvent &event )
  {
    ros::Duration Dt = event.current_real - event.last_real;
 
    if( found_point_.z == -1.0 )
      {
	// Rotate
	current_cmd_ = geometry_msgs::Twist();
	//	current_cmd_.angular.z = 1.0 / SCALE;
	//current_cmd_.angular.z = 0.0

	setLedAnim( 0, 2 );
      }
    else
      {
	// Run the pid controller
	current_cmd_.angular.z = xPid.getOutput( found_point_.x, Dt );
	current_cmd_.linear.z = yPid.getOutput( found_point_.y, Dt );
	current_cmd_.linear.x = zPid.getOutput( found_point_.z, Dt );

	setLedAnim( 8, 2 );
      }

    cmd_vel_.publish( current_cmd_ );

    // then in 0.5s, publish a hover cmd
    hoverTimer_ = nh_.createTimer( ros::Duration( Dt.toSec() / 2.0 ), &ArdroneFollow::hoverCmdCb, this, /* oneshot = */ true );
  }
};

int main( int argc, char **argv )
{
  ros::init( argc, argv, "ardrone_follow" );
  ArdroneFollow af;

  puts( "reset" );
  af.reset();
  sleep( 2 );
  puts( "takeoff" );
  af.takeoff();

  // clear commands
  af.hover();

  // let it settle after taking off
  sleep( 10 );

  ros::AsyncSpinner spinner(2);

  // Launch
  spinner.start();
  sleep( 300 ); // 5 minutes

  // Land
  puts( "land" );
  af.land();

  ros::shutdown();
  ros::waitForShutdown(); // kind of unnecessary but just in case
  return 0;
}
