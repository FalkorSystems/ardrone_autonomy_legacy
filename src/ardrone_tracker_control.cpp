#include <opencv/highgui.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

ros::Publisher morphSizePub;
ros::Publisher hValuePub;
ros::Publisher hRangePub;
ros::Publisher targetSizePub;
ros::Publisher matchThreshPub;

void morphSizeCb( int val, void *dummy )
{
  geometry_msgs::Point msg;
  msg.x = msg.y = val;
  morphSizePub.publish(msg);
}

void targetSizeCb( int val, void *dummy )
{
  std_msgs::Float32 msg;
  msg.data = val;
  targetSizePub.publish(msg);
}

void hValueCb( int val, void *dummy )
{
  std_msgs::Int32 msg;
  msg.data = val;
  hValuePub.publish( msg );
}

void hRangeCb( int val, void *dummy )
{
  std_msgs::Int32 msg;
  msg.data = val;
  hRangePub.publish( msg );
}

void matchThreshCb( int val, void *dummy )
{
  std_msgs::Float32 msg;
  msg.data = val / 100.0;
  matchThreshPub.publish( msg );
}

int main(int argc, char **argv)
{
  ros::init( argc, argv, "ardrone_tracker_control" );
  ros::NodeHandle n;

  cv::namedWindow( "Tracking Controller", CV_WINDOW_AUTOSIZE );
  cv::Mat image( 1, 400, CV_8UC1, cv::Scalar( 0, 0, 0 ) );
  cv::imshow( "Tracking Controller", image );

  morphSizePub = n.advertise<geometry_msgs::Point>("/ardrone_tracker/control/morph_size", 1 );
  targetSizePub = n.advertise<std_msgs::Float32>("/ardrone_tracker/control/target_size", 1 );
  hValuePub = n.advertise<std_msgs::Int32>("/ardrone_tracker/control/h_value", 1 );
  hRangePub = n.advertise<std_msgs::Int32>("/ardrone_tracker/control/h_range", 1 );
  matchThreshPub = n.advertise<std_msgs::Float32>("/ardrone/tracker/control/match_threshold", 1 );

  int morphSize = 15;
  int hValue = 20;
  int hRange = 50;
  int targetSize = 5;
  int matchThresh = 100;

  cv::createTrackbar( "Morph Size", "Tracking Controller", &morphSize, 50, morphSizeCb );
  cv::createTrackbar( "H Value", "Tracking Controller", &hValue, 180, hValueCb );
  cv::createTrackbar( "H Range", "Tracking Controller", &hRange, 180, hRangeCb );
  cv::createTrackbar( "Target Size", "Tracking Controller", &targetSize, 20, targetSizeCb );
  cv::createTrackbar( "Match Threshold x100", "Tracking Controller", &matchThresh, 500, matchThreshCb );

  while(1)
    cv::waitKey();

  ros::shutdown();
  return(0);
}
