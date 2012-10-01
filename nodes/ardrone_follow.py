#!/usr/bin/env python
import roslib
roslib.load_manifest('ardrone_autonomy')

import sys
import rospy
import math
import pid
import cv2
import numpy as np

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from ardrone_autonomy.msg import Navdata
from ardrone_autonomy.srv import LedAnim

class ArdroneFollow:
    def __init__( self ):
        self.navdata_sub = rospy.Subscriber( "ardrone/navdata", Navdata, 
                                             self.navdata_cb )
        self.tracker_sub = rospy.Subscriber( "ardrone_tracker/found_point",
                                             Point, self.found_point_cb )
        self.cmd_vel_pub = rospy.Publisher( "cmd_vel", Twist )
        self.timer = rospy.Timer( rospy.Duration( 0.25 ), self.timer_cb, False )

        self.land_pub = rospy.Publisher( "ardrone/land", Empty )
        self.takeoff_pub = rospy.Publisher( "ardrone/takeoff", Empty )
        self.reset_pub = rospy.Publisher( "ardrone/reset", Empty )

        self.xPid = pid.Pid( 0.020, 0.0, 0.0, 100.0 )
        self.yPid = pid.Pid( 0.020, 0.0, 0.0, 100.0 )
        self.zPid = pid.Pid( 0.500, 0.0, 0.0,  50.0 )

        self.xPid.setPointMin = 40
        self.xPid.setPointMax = 60

        self.yPid.setPointMin = 50
        self.yPid.setPointMax = 65

        self.zPid.setPointMin = 17
        self.zPid.setPointMax = 23

        self.led_service = rospy.ServiceProxy( "ardrone/setledanimation", LedAnim )
        self.lastAnim = -1

        self.foundpoint = Point( 0, 0, -1 )
        self.old_cmd = self.current_cmd = Twist()

        self.joy_sub = rospy.Subscriber( "joy", Joy, self.callback_joy )
        self.manual_cmd = False
        self.auto_cmd = False

        cv2.namedWindow( 'Follow Clues', cv2.cv.CV_WINDOW_NORMAL )


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
        self.current_cmd.angular.z = data.axes[2]

        self.current_cmd.linear.z = data.axes[3]
        self.current_cmd.linear.x = data.axes[1]
        self.current_cmd.linear.y = data.axes[0]

        if ( self.current_cmd.linear.x == 0 and
             self.current_cmd.linear.y == 0 and
             self.current_cmd.linear.z == 0 and
             self.current_cmd.angular.z == 0 ):
            self.manual_cmd = False
        else:
            self.setLedAnim( 9 )
            self.manual_cmd = True

        self.cmd_vel_pub.publish( self.current_cmd )

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
        pass

    def found_point_cb( self, data ):
        self.found_point = data

    def hover( self ):
        hoverCmd = Twist()
        self.cmd_vel_pub.publish( hoverCmd )

    def hover_cmd_cb( self, data ):
        self.hover()

    def draw_picture( self ):
        vis = np.zeros( ( 360, 640, 3 ), np.uint8);

        cx_min = int(self.xPid.setPointMin * 640 / 100)
        cx_max = int(self.xPid.setPointMax * 640 / 100)
        cx = int( ( cx_min + cx_max ) / 2 )
        
        cy_min = int(self.yPid.setPointMin * 480 / 100)
        cy_max = int(self.yPid.setPointMax * 480 / 100)
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

        cv2.line( vis, ( 320, 180 ), ( int( 320 - self.current_cmd.angular.z * 320 ),
                                       int( 180 - self.current_cmd.linear.z * 180  ) ),
                  ( 0, 255, 0 ),
                  min( max( int( abs( self.current_cmd.linear.x ) * 255 ), 0 ), 255 ) )

        cv2.imshow( 'Follow Clues', vis )
        cv2.waitKey( 1 )

    def timer_cb( self, event ):
        self.draw_picture()

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

        self.cmd_vel_pub.publish( self.current_cmd )
        self.hover_timer = rospy.Timer( rospy.Duration( dt / 2.0 ), self.hover_cmd_cb,
                                        True )



def main():
    rospy.init_node( 'ardrone_follow' )
    af = ArdroneFollow()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Keyboard interrupted"

if __name__ == '__main__':
    main()
