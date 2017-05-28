#!/usr/bin/env python

import numpy
import roslib
import math

import rospy
import time

from provider_keypad.msg import Keypad

from F130_keypad import *

REFRESH_RATE_SECOND = 0.1


class KeypadNode:

    def __init__(self):
        rospy.init_node('provider_keypad_node', anonymous=True)
        keypad_publisher = rospy.Publisher('/provider_keypad/Keypad', Keypad, queue_size=100)
        self.F130_Keypad = F130_Keypad()

        try:
            while not rospy.is_shutdown():
                rospy.sleep(REFRESH_RATE_SECOND)

                # Boutton tracking
                keypad_msg = Keypad()
                keypad_msg.A = bool(self.F130_Keypad.states['A'])
                keypad_msg.Y = bool(self.F130_Keypad.states['Y'])
                keypad_msg.B = bool(self.F130_Keypad.states['B'])
                keypad_msg.X = bool(self.F130_Keypad.states['X'])
                keypad_msg.Start = bool(self.F130_Keypad.states['Start'])
                keypad_msg.Back = bool(self.F130_Keypad.states['Back'])
                keypad_msg.Middle = bool(self.F130_Keypad.states['Middle'])

                keypad_msg.Left = int(self.F130_Keypad.states['Left'])
                keypad_msg.Right = int(self.F130_Keypad.states['Right'])
                keypad_msg.Up = int(self.F130_Keypad.states['Up'])
                keypad_msg.Down = int(self.F130_Keypad.states['Down'])

                keypad_msg.RB = int(self.F130_Keypad.states['RB'])
                keypad_msg.RT = int(self.F130_Keypad.states['RT'])
                keypad_msg.LB = int(self.F130_Keypad.states['LB'])
                keypad_msg.LT = int(self.F130_Keypad.states['LT'])

                keypad_msg.RJ_Left = int(self.F130_Keypad.states['RJ/Left'])
                keypad_msg.RJ_Right = int(self.F130_Keypad.states['RJ/Right'])
                keypad_msg.RJ_Up = int(self.F130_Keypad.states['RJ/Up'])
                keypad_msg.RJ_Down = int(self.F130_Keypad.states['RJ/Down'])
                keypad_msg.RJ_Button = int(self.F130_Keypad.states['RJ/Button'])
                keypad_msg.LJ_Left = int(self.F130_Keypad.states['LJ/Left'])
                keypad_msg.LJ_Right = int(self.F130_Keypad.states['LJ/Right'])
                keypad_msg.LJ_Up = int(self.F130_Keypad.states['LJ/Up'])
                keypad_msg.LJ_Down = int(self.F130_Keypad.states['LJ/Down'])
                keypad_msg.LJ_Button = int(self.F130_Keypad.states['LJ/Button'])

                try:
                    keypad_publisher.publish(keypad_msg)
                except:
                    print "exception catched"

        except KeyboardInterrupt:
            rospy.signal_shutdown('keyboard interrupt')


if __name__ == '__main__':
    try:
        ne = KeypadNode()
    except rospy.ROSInterruptException:
        pass


