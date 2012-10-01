#!/usr/bin/env python
import roslib
roslib.load_manifest('ardrone_autonomy')

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

class ArdroneTeleopJoy:

    def __init__( self ):
        self.cmd_vel_pub = rospy.Publisher( "cmd_vel", Twist )
        self.land_pub = rospy.Publisher( "ardrone/land", Empty )
        self.takeoff_pub = rospy.Publisher( "ardrone/takeoff", Empty )
        self.reset_pub = rospy.Publisher( "ardrone/reset", Empty )
        self.joy_sub = rospy.Subscriber( "joy", Joy, self.callback_joy )

    def callback_joy( self, data ):
        empty_msg = Empty()

        if data.buttons[12] == 1 and self.last_buttons[12] == 0:
            self.takeoff_pub.publish( empty_msg )

        if data.buttons[14] == 1 and self.last_buttons[14] == 0:
            self.land_pub.publish( empty_msg )

        if data.buttons[13] == 1 and self.last_buttons[13] == 0:
            self.reset_pub.publish( empty_msg )

        self.last_buttons = data.buttons

        # Do cmd_vel
        cmd = Twist()

        cmd.angular.x = cmd.angular.y = 0
        cmd.angular.z = data.axes[2]

        cmd.linear.z = data.axes[3]
        cmd.linear.x = data.axes[1]
        cmd.linear.y = data.axes[0]

        self.cmd_vel_pub.publish( cmd )

def main():
  rospy.init_node( 'ardrone_teleop_joy' )

  ArdroneTeleopJoy()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main()
            
    
