#!/usr/bin/env python
import roslib
roslib.load_manifest('ardrone_autonomy')

import sys
import rospy
import math
import pid
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy, Image
from ardrone_autonomy.msg import Navdata
from ardrone_autonomy.srv import LedAnim

class ArdroneFollow:
    def __init__( self ):
        self.led_service = rospy.ServiceProxy( "ardrone/setledanimation", LedAnim )
#        print "waiting for driver to startup"
#        rospy.wait_for_service( self.led_service )

        self.tracker_sub = rospy.Subscriber( "ardrone_tracker/found_point",
                                             Point, self.found_point_cb )
        self.goal_vel_pub = rospy.Publisher( "goal_vel", Twist )
        self.found_time = None

        self.tracker_img_sub = rospy.Subscriber( "ardrone_tracker/image",
                                                 Image, self.image_cb )
        self.tracker_image = None

        self.timer = rospy.Timer( rospy.Duration( 0.10 ), self.timer_cb, False )

        self.land_pub = rospy.Publisher( "ardrone/land", Empty )
        self.takeoff_pub = rospy.Publisher( "ardrone/takeoff", Empty )
        self.reset_pub = rospy.Publisher( "ardrone/reset", Empty )

        self.angularZlimit = 3.141592 / 2
        self.linearXlimit = 1.0
        self.linearZlimit = 2.0

        self.xPid = pid.Pid( 0.020, 0.0, 0.0, self.angularZlimit )
        self.yPid = pid.Pid( 0.020, 0.0, 0.0, self.linearZlimit )
        self.zPid = pid.Pid( 0.070, 0.0, 0.0, self.linearXlimit )

        self.xPid.setPointMin = 40
        self.xPid.setPointMax = 60

        self.yPid.setPointMin = 40
        self.yPid.setPointMax = 60

        self.zPid.setPointMin = 17
        self.zPid.setPointMax = 23

        self.lastAnim = -1

        self.found_point = Point( 0, 0, -1 )
        self.old_cmd = self.current_cmd = Twist()

        self.joy_sub = rospy.Subscriber( "joy", Joy, self.callback_joy )
        self.manual_cmd = False
        self.auto_cmd = False

        self.bridge = CvBridge()

        cv2.namedWindow( 'AR.Drone Follow', cv2.cv.CV_WINDOW_NORMAL )

    def image_cb( self, data ):
        try:
            cv_image = self.bridge.imgmsg_to_cv( data, "passthrough" )
        except CvBridgeError, e:
            print e
        
        self.tracker_image = np.asarray( cv_image )

    def increase_z_setpt( self ):
        self.zPid.setPointMin *= 1.01
        self.zPid.setPointMax *= 1.01

    def decrease_z_setpt( self ):
        self.zPid.setPointMin /= 1.01
        self.zPid.setPointMax /= 1.01

    def callback_joy( self, data ):
        empty_msg = Empty()

        if data.buttons[12] == 1 and self.last_buttons[12] == 0:
            self.takeoff()

        if data.buttons[14] == 1 and self.last_buttons[14] == 0:
            self.land()

        if data.buttons[13] == 1 and self.last_buttons[13] == 0:
            self.reset()

        if data.buttons[15] == 1 and self.last_buttons[15] == 0:
            self.auto_cmd = not self.auto_cmd
            self.hover()

        if data.buttons[4] == 1:
            self.increase_z_setpt()

        if data.buttons[6] == 1:
            self.decrease_z_setpt()

        self.last_buttons = data.buttons

        # Do cmd_vel
        self.current_cmd = Twist()

        self.current_cmd.angular.x = self.current_cmd.angular.y = 0
        self.current_cmd.angular.z = data.axes[2] * self.angularZlimit

        self.current_cmd.linear.z = data.axes[3] * self.linearZlimit
        self.current_cmd.linear.x = data.axes[1] * self.linearXlimit
        self.current_cmd.linear.y = data.axes[0] * self.linearXlimit

        if ( self.current_cmd.linear.x == 0 and
             self.current_cmd.linear.y == 0 and
             self.current_cmd.linear.z == 0 and
             self.current_cmd.angular.z == 0 ):
            self.manual_cmd = False
        else:
            self.setLedAnim( 9 )
            self.manual_cmd = True

        self.goal_vel_pub.publish( self.current_cmd )

    def setLedAnim( self, animType, freq = 10 ):
        if self.lastAnim == type:
            return

        msg = LedAnim();
        msg.type = animType;
        msg.freq = freq;
        msg.duration = 3600;
        
        self.led_service( type = animType, freq = freq, duration = 255 )
        self.lastAnim = type

    def takeoff( self ):
        self.takeoff_pub.publish( Empty() )
        self.setLedAnim( 9 )

    def land( self ):
        self.land_pub.publish( Empty() )

    def reset( self ):
        self.reset_pub.publish( Empty() )

    def navdata_cb( self, data ):
        self.vx = data.vx/1e3
        self.vy = data.vy/1e3
        self.vz = data.vz/1e3
        self.ax = (data.ax*9.82)
        self.ay = (data.ay*9.82)
        self.az = (data.az - 1)*9.82

    def found_point_cb( self, data ):
        self.found_point = data
        self.found_time = rospy.Time.now()

    def hover( self ):
        hoverCmd = Twist()
        self.goal_vel_pub.publish( hoverCmd )

    def hover_cmd_cb( self, data ):
        self.hover()

    def draw_picture( self ):
        if self.tracker_image == None:
            return

        vis = self.tracker_image.copy()
        (ySize, xSize, depth) = vis.shape

        cx_min = int(self.xPid.setPointMin * xSize / 100)
        cx_max = int(self.xPid.setPointMax * xSize / 100)
        cx = int( ( cx_min + cx_max ) / 2 )
        
        cy_min = int(self.yPid.setPointMin * ySize / 100)
        cy_max = int(self.yPid.setPointMax * ySize / 100)
        cy = int( ( cy_min + cy_max ) / 2 )

        cv2.rectangle( vis, ( cx_min, cy_min ), ( cx_max, cy_max ),
                       ( 0, 255, 255 ) )

        areasqrt = np.sqrt( 640 * 480 )
        side_min_half = int( self.zPid.setPointMin * areasqrt / 100 / 2 )
        side_max_half = int( self.zPid.setPointMax * areasqrt / 100 / 2 )

        cv2.rectangle( vis, ( cx - side_min_half, cy - side_min_half ),
                       ( cx + side_min_half, cy + side_min_half ),
                       ( 255, 255, 0 ) )

        cv2.rectangle( vis, ( cx - side_max_half, cy - side_max_half ),
                       ( cx + side_max_half, cy + side_max_half ),
                       ( 255, 255, 0 ) )

        if self.current_cmd.linear.x > 0:
            line_color = ( 0, 255, 0 )
        else:
            line_color = ( 0, 0, 255 )

        cv2.line( vis, ( 320, 180 ), ( int( xSize/2 - self.current_cmd.angular.z * 320 ),
                                       int( ySize/2 - self.current_cmd.linear.z * 180  ) ),
                  line_color,
                  min( max( int( abs( self.current_cmd.linear.x ) * 255 ), 0 ), 255 ) )

        cv2.imshow( 'AR.Drone Follow', vis )
        cv2.waitKey( 1 )

    def timer_cb( self, event ):
        self.draw_picture()

        # If we haven't received a found point in a second, let found_point be
        # (0,0,-1)
        if ( self.found_time == None or
             ( rospy.Time.now() - self.found_time ).to_sec() > 1 ):
            self.found_point = Point( 0, 0, -1.0 )
            self.found_time = rospy.Time.now()

        if event.last_real == None:
            dt = 0
        else:
            dt = ( event.current_real - event.last_real ).to_sec()

        if self.found_point.z == -1.0:
            self.current_cmd = Twist()
            self.setLedAnim( 0, 2 )
        else:
            self.current_cmd.angular.z = self.xPid.get_output( self.found_point.x,
                                                               dt )
            self.current_cmd.linear.z = self.yPid.get_output( self.found_point.y,
                                                              dt )
            self.current_cmd.linear.x = self.zPid.get_output( self.found_point.z,
                                                              dt )

            self.setLedAnim( 8, 2 )

        if self.auto_cmd == False or self.manual_cmd == True:
            self.setLedAnim( 9 )
            return

        self.goal_vel_pub.publish( self.current_cmd )


def main():
    rospy.init_node( 'ardrone_follow' )
    af = ArdroneFollow()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Keyboard interrupted"

if __name__ == '__main__':
    main()
