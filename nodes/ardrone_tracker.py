#!/usr/bin/env python
import roslib
roslib.load_manifest('ardrone_autonomy')

import sys
import rospy
import cv
import cv2
import math

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import tracker

class ardroneTracker:

  def __init__(self, tracker):
    self.point_pub = rospy.Publisher("/ardrone_tracker/found_point", Point )

    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/front/image_raw",Image,self.callback)
    self.tracker = tracker

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e
  
    numpy_image = np.asarray( cv_image )
    trackData = self.tracker.track( numpy_image )
    if trackData:
      areaGoal = 20.0

      x,y,z = trackData[0],trackData[1],trackData[2]/areaGoal*50

      point = Point( x,y,z )
      self.point_pub.publish( point )
    else:
      self.point_pub.publish( Point( 0, 0, -1 ) )

    cv2.waitKey( 10 )

    # try:
    #   self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError, e:
    #   print e

def main():
  rospy.init_node( 'ardrone_tracker' )
#  ardroneTracker(tracker.LkTracker() )
#  ardroneTracker(tracker.DummyTracker() )
#            '/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml'
  ardroneTracker( tracker.CascadeTracker( '/home/sameer/FalkorCascade/cascade/cascade.xml' ) )
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main()
