#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
cv::RNG rng(12345);

class ArdroneTracker
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber imageSub_;
  image_transport::Publisher thresholdPub_;
  image_transport::Publisher histPub_;
  image_transport::Publisher morphPub_;
  image_transport::Publisher centersPub_;
  ros::Publisher pointPub_;

  bool useColorTracker_;

  // For all trackers
  cv::Size morphSize_;
  double targetSize_;

  ros::Subscriber morphSizeSub_;
  ros::Subscriber targetSizeSub_;
  
  // For the color-based tracker
  int hValue_;
  int hRange_;

  ros::Subscriber hValueSub_;
  ros::Subscriber hRangeSub_;

  // For the shape-based tracker
  cv::Mat roundelSource_;
  cv::vector < cv::Point > matchContour_;
  double matchThreshold_;

  ros::Subscriber matchThreshSub_;

  geometry_msgs::Point pointPublished_;
  int missesInARow_;
  double dampingFactor_;
  cv::SurfFeatureDetector surf_;
  cv::SurfDescriptorExtractor surfDesc_;
  cv::vector< cv::KeyPoint > keyPoints_;
  cv::Mat descriptors_;

  cv::BruteForceMatcher< cv::L2< float > > matcher_;

  void calcMatchKeyPoints(void)
  {
    surf_.detect( roundelSource_, keyPoints_ );
    surfDesc_.compute( roundelSource_, keyPoints_, descriptors_ );

#ifndef NOIMSHOW
    cv::namedWindow( "Roundel Surf" );
    cv::Mat drawing = roundelSource_.clone();

    cv::drawKeypoints( roundelSource_, keyPoints_, drawing, cv::Scalar( 255, 255, 255 ), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::imshow( "Roundel Surf", drawing );
#endif
  }

  cv::vector< cv::Point > getMatchContour(void)
  {
    cv::Mat roundelThresh;
    cv::inRange( roundelSource_, cv::Scalar( 0, 50, 50 ), cv::Scalar( 75, 255, 255 ), roundelThresh );

    cv::vector< cv::vector<cv::Point> > roundelContours;
    cv::findContours( roundelThresh, roundelContours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );
    if( roundelContours.size() != 1 )
      {
	puts( "invalid roundel file" );
	exit( -1 );
      }

#ifndef NOIMSHOW
    cv::namedWindow( "Roundel Contour" );
    cv::Mat drawing = cv::Mat::zeros( roundelSource_.size(), CV_8UC1 );

    cv::drawContours( drawing, roundelContours, 0, cv::Scalar( 255, 255, 255 ), 2, 8 );
    cv::imshow( "Roundel Contours", drawing );
#endif

    return( roundelContours[0] );
  }

  cv_bridge::CvImagePtr getImage( const sensor_msgs::ImageConstPtr &msg )
  {
    cv_bridge::CvImagePtr cvPtr;

    try
      {
	cvPtr = cv_bridge::toCvCopy( msg, enc::BGR8 );
      }
    catch (cv_bridge::Exception &e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what() );
	return(cvPtr);
      }
  }

  void thresholdImage( cv_bridge::CvImagePtr cvPtr, bool byColor )
  {
    // blur first
    cv::GaussianBlur( cvPtr->image, cvPtr->image, cv::Size( 5, 5 ), 0 );

    if( byColor == true )
      {
	cv::cvtColor( cvPtr->image, cvPtr->image, CV_BGR2HSV );

	cv::Mat destImage;
	cv::Scalar lowerBound( std::max( hValue_ - hRange_/2, 0 ), 50, 50 );
	cv::Scalar upperBound( std::min( hValue_ + hRange_/2, 255 ), 255, 255 );

	cv::inRange( cvPtr->image, lowerBound, upperBound, cvPtr->image );
      }
    else
      {
	cv::cvtColor( cvPtr->image, cvPtr->image, CV_BGR2GRAY );

	cv::Point min_loc;
	cv::Point max_loc;
	double min_val;
	double max_val;

	cv::minMaxLoc( cvPtr->image,
		       &min_val,
		       &max_val,
		       &min_loc,
		       &max_loc );

	cv::threshold(
		      cvPtr->image,
		      cvPtr->image,
		      max_val - 1,
		      //100,
		      255,
		      CV_THRESH_BINARY );
      }

    cvPtr->encoding = enc::MONO8;
    thresholdPub_.publish(cvPtr->toImageMsg());
  }    

  void applyMorphology( cv_bridge::CvImagePtr cvPtr )
  {
    cv::Mat element = cv::getStructuringElement( 0, morphSize_, cv::Point( 1, 1 ) );

    // close then open
    cv::morphologyEx(
		     cvPtr->image, cvPtr->image,
		     cv::MORPH_CLOSE, element );

    cv::morphologyEx(
		     cvPtr->image, cvPtr->image,
    		     cv::MORPH_OPEN, element );


    morphPub_.publish( cvPtr->toImageMsg() );
  }

  cv::vector<cv::vector<cv::Point> > getContours( const cv::Mat &image )
  {
    cv::vector<cv::vector<cv::Point> > contours;

    cv::findContours( image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );
    return( contours );
  }

  cv::vector< double > getAreaBasedProbs( const cv::vector< cv::vector< cv::Point > > &contours )
  {
    int largestContour = -1;
    double maxArea = 0.0;
    cv::vector< double > probs( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    {
      probs[i] = cv::contourArea( contours[i] );
      if( maxArea < probs[i] )
	{
	  maxArea = probs[i];
	  largestContour = i;
	}
    }
    
    for( int i = 0; i < contours.size(); i++ )
      {
	probs[i] /= probs[largestContour];
      }

    return( probs );
  }

  cv::vector< double > getMomentMatchBasedProbs( const cv::vector< cv::vector< cv::Point > > &contours )
  {
    cv::vector< double > probs( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
      {
	probs[i] =  1.0 - fmin( matchShapes( contours[i], matchContour_, CV_CONTOURS_MATCH_I2, 0.0 ) / matchThreshold_, 1.0 );
      }
    return( probs );
  }
  
public:
  ArdroneTracker()
    : it_(nh_), useColorTracker_( true ),
      morphSize_( 15, 15 ),
      targetSize_( 10 ),
      hValue_( 30 ),
      hRange_( 50 ),
      matchThreshold_( 1.0 ),
      missesInARow_( 0 ),
      dampingFactor_( 0.9 ),
      surf_ ( 2500 )
    
  {
    imageSub_ = it_.subscribe("/ardrone/front/image_raw", 1, &ArdroneTracker::imageCb, this);
    thresholdPub_ = it_.advertise("/ardrone_tracker/image_threshold", 1 );
    histPub_ = it_.advertise( "/ardrone_tracker/image_histogram", 1 );
    morphPub_ = it_.advertise( "/ardrone_tracker/image_morphed", 1 );
    centersPub_ = it_.advertise("/ardrone_tracker/image_centers", 1 );
    pointPub_ = nh_.advertise<geometry_msgs::Point>("/ardrone_tracker/found_point", 1);

    morphSizeSub_ = nh_.subscribe( "/ardrone_tracker/control/morph_size", 1, &ArdroneTracker::morphSizeCb, this );
    targetSizeSub_ = nh_.subscribe( "/ardrone_tracker/control/target_size", 1, &ArdroneTracker::targetSizeCb, this );
    hValueSub_ = nh_.subscribe( "/ardrone_tracker/control/h_value", 1, &ArdroneTracker::hValueCb, this );
    hRangeSub_ = nh_.subscribe( "/ardrone_tracker/control/h_range", 1, &ArdroneTracker::hRangeCb, this );
    matchThreshSub_ = nh_.subscribe( "/ardrone_tracker/control/match_threshold", 1, &ArdroneTracker::matchThreshCb, this );
    roundelSource_ = cv::imread( "/home/sameer/Dropbox/Linux XFer/Roundel.png", 1 );

    matchContour_ = getMatchContour();
    calcMatchKeyPoints();
  }

  ~ArdroneTracker()
  {
  }

  void matchThreshCb( const std_msgs::Float32 &msg )
  {
    matchThreshold_ = msg.data;
  }

  void morphSizeCb(const geometry_msgs::Point &msg )
  {
    morphSize_.width = std::max( msg.x, 2.0 );
    morphSize_.height = std::max( msg.y, 2.0 );
  }

  void targetSizeCb( const std_msgs::Float32 &msg )
  {
    targetSize_ = msg.data;
  }

  void hValueCb( const std_msgs::Int32 &msg )
  {
    hValue_ = msg.data;
  }

  void hRangeCb( const std_msgs::Int32 &msg )
  {
    hRange_ = msg.data;
  }

  double getSingleSurfProb( const cv::vector< cv::Point > &contour, cv_bridge::CvImagePtr cvPtr, int contourNum = 0 )
  {
    // Get the ROI for this contour
    cv::Rect boundingRect = cv::boundingRect( contour );
    cv::Mat imageROI = cvPtr->image( boundingRect );

    cv::vector< cv::KeyPoint > keypoints;

    surf_.detect( imageROI, keypoints );

    cv::Mat descriptors;
    surfDesc_.compute( imageROI, keypoints, descriptors );

    std::vector< cv::DMatch > matches;
    matcher_.match( descriptors, descriptors_, matches );

    // only take the 25 best matches
    std::nth_element( matches.begin(), matches.begin() + 24, matches.end() );
    matches.erase( matches.begin() + 25 );

    // get the distance of the 25 best matches
    double sum = 0;
    for( int i = 0; i < matches.size(); i++ )
      sum += 0; // matches[i].distance;

    // draw the matches
#ifdef NOIMSHOW
    cv::Mat drawing = cv::Mat::zeros( cvPtr->image.size(), CV_8UC3 );
    cv::drawMatches( roundelImage_, descriptors_,
		     ctPtr->image, descriptors,
		     matches,
		     drawing,
		     cv::Scalar( 255, 255, 255 ) );

    char windowName[255];
    sprintf( windowName, "Surf Matches %d", contourNum );
    cv::namedWindow( windowName );
    cv::imshow( windowName, drawing );
#endif

    // the probability is how close we are to 0
    return( fmax( 1.0 - sum, 0.0 ) );
  }

  cv::vector< double > getSurfMatchBasedProbs( const cv::vector< cv::vector< cv::Point > > &contours, cv_bridge::CvImagePtr cvPtr )
  {
    cv::vector< double > probs( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
      {
	probs[i] = getSingleSurfProb( contours[i], cvPtr, i );
      }
    return( probs );
  }
	
  void equalizeHistogram( cv_bridge::CvImagePtr cvPtr )
  {
    cv::vector< cv::Mat > splitImages;
    cv::split( cvPtr->image, splitImages );
    for( int i = 0; i < splitImages.size(); i++ )
      {
	cv::equalizeHist( splitImages[i], splitImages[i] );
      }
    cv::merge( splitImages, cvPtr->image );
    histPub_.publish( cvPtr->toImageMsg() );
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //cv_bridge::CvImagePtr cvPtr = getThreshImageColor( msg ); 
    //cv_bridge::CvImagePtr cvPtr = getThreshImageInfrared( msg ); 

    cv_bridge::CvImagePtr cvPtr = getImage( msg );

    equalizeHistogram( cvPtr );

    //    static int counter = 0;
    //    if( counter++ % 128 == 0 )
    //      {
	// every 128 frames do a full match

    cv_bridge::CvImagePtr threshImagePtr(new cv_bridge::CvImage( cvPtr->header, cvPtr->encoding, cvPtr->image.clone() ) );
    thresholdImage( threshImagePtr, /* byColor = */ false );

    cv::vector< cv::vector<cv::Point> > contours;

    //    applyMorphology( threshImagePtr );
    contours = getContours( threshImagePtr->image );

    cv::vector< double > areaBasedProbs = getAreaBasedProbs( contours );
    //    cv::vector< double > momentMatchBasedProbs = getMomentMatchBasedProbs( contours );
    //    cv::vector< double > surfMatchBasedProbs = getSurfMatchBasedProbs( contours, cvPtr );
    

    double maxProb = 0;
    int chosenContour = -1;

    for( int i = 0; i < contours.size(); i++ )
      {
	double prob = areaBasedProbs[i]; // * momentMatchBasedProbs[i] * surfMatchBasedProbs[i];
	if( prob > maxProb )
	  {
	    maxProb = 0;
	    chosenContour = i;
	  }
      }

    // Draw centers & contours
    cv::Mat drawing = cv::Mat::zeros( cvPtr->image.size(), CV_8UC3 );
    for( int i = 0; i < contours.size(); i++ )
      {
	if( i == chosenContour )
	  continue;

	cv::Scalar color =  cv::Scalar( 50, 150, 150 );
	cv::drawContours( drawing, contours, i, color, 2, 8 );
      }

    geometry_msgs::Point point;
    if( chosenContour > -1 )
      {
	// Get the mass center of the chosen contour
	cv::Moments chosenMoments = cv::moments( contours[chosenContour], false );

	cv::Point foundCenter = cv::Point( chosenMoments.m10/chosenMoments.m00,
					   chosenMoments.m01/chosenMoments.m00 );
    

	cv::drawContours( drawing, contours, chosenContour, cv::Scalar( 255, 255, 255 ),
			  2, 8 );
	cv::circle( drawing, foundCenter, 10, cv::Scalar( 255, 255, 255 ), -1, 8, 0 );

	point.x = foundCenter.x * 100.0 / cvPtr->image.size().width;
	point.y = foundCenter.y * 100.0 / cvPtr->image.size().height;

	// distance is the size of the contour relative to targetSize_;
	double contourArea = cv::contourArea( contours[chosenContour] );
	double imageArea = cvPtr->image.size().width * cvPtr->image.size().height;

	// 50 means we are at the target size, lower too close, higher too far
	// if contourArea/imageArea is very close to 1 something is wrong,
	// regard it as a miss
	if( fabs( contourArea/imageArea - 1.0 ) < .05 )
	  {
	    point.x = point.y = 0.0;
	    point.z = -1.0;
	    missesInARow_++;
	  }
	else
	  {
	    point.z = fmax( ( targetSize_ - contourArea/imageArea * 100.0 ) / 2.0 + 50.0, 0.0 );
	    missesInARow_ = 0;
	  }
      }
    else
      {
	point.x = point.y = 0.0;
	point.z = -1.0;
	missesInARow_++;
      }

    if( missesInARow_ > 30 || pointPublished_.z == -1.0 )
      {
	pointPublished_ = point;
	missesInARow_ = 0;
      }
    else if( point.z != -1.0 )
      {
	pointPublished_.x = ( point.x + pointPublished_.x * dampingFactor_ ) / ( 1.0 + dampingFactor_ );
	pointPublished_.y = ( point.y + pointPublished_.y * dampingFactor_ ) / ( 1.0 + dampingFactor_ );
	pointPublished_.z = ( point.z + pointPublished_.z * dampingFactor_ ) / ( 1.0 + dampingFactor_ );
      }

    // draw the pointPublished
    cv::Point drawPointPublished( pointPublished_.x * cvPtr->image.size().width / 100.0,
				  pointPublished_.y * cvPtr->image.size().height / 100.0 );
    cv::circle( drawing, drawPointPublished, 5, cv::Scalar( 255, 0, 255 ), -1, 8, 0 );

    // Publish
    cv_bridge::CvImage drawingImage( cvPtr->header, enc::BGR8, drawing );
    centersPub_.publish( drawingImage.toImageMsg() );

    pointPub_.publish( pointPublished_ ); 
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ardrone_tracker");
  ArdroneTracker at;
  ros::spin();
  return 0;
}
