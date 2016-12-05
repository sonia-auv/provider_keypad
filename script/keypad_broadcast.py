#!/usr/bin/env python

import numpy
import roslib
import math

import rospy

from provider_keypad.msg import Keypad

from F130_keypad import *

REFRESH_RATE_SECOND = 0.1

if __name__ == "__main__":
    rospy.init_node('provider_keypad_node')
    keypad_publisher = rospy.Publisher('/provider_keypad/Keypad', Keypad, queue_size=100)
    time_since_up = 0
    F130_Keypad = F130_Keypad()

    while not rospy.is_shutdown():
        time.sleep(REFRESH_RATE_SECOND)
        time_since_up += REFRESH_RATE_SECOND

        # Boutton tracking
        keypad_msg = Keypad()
        keypad_msg.A = bool(F130_Keypad.states['A'])
        keypad_msg.Y = bool(F130_Keypad.states['Y'])
        keypad_msg.B = bool(F130_Keypad.states['B'])
        keypad_msg.X = bool(F130_Keypad.states['X'])
        keypad_msg.Start = bool(F130_Keypad.states['Start'])
        keypad_msg.Back = bool(F130_Keypad.states['Back'])
        keypad_msg.Middle = bool(F130_Keypad.states['Middle'])

        keypad_msg.Left = int(F130_Keypad.states['Left'])
        keypad_msg.Right = int(F130_Keypad.states['Right'])
        keypad_msg.Up = int(F130_Keypad.states['Up'])
        keypad_msg.Down = int(F130_Keypad.states['Down'])

        keypad_msg.RB = int(F130_Keypad.states['RB'])
        keypad_msg.RT = int(F130_Keypad.states['RT'])
        keypad_msg.LB = int(F130_Keypad.states['LB'])
        keypad_msg.LT = int(F130_Keypad.states['LT'])

        keypad_msg.RJ_Left = int(F130_Keypad.states['RJ/Left'])
        keypad_msg.RJ_Right = int(F130_Keypad.states['RJ/Right'])
        keypad_msg.RJ_Up = int(F130_Keypad.states['RJ/Up'])
        keypad_msg.RJ_Down = int(F130_Keypad.states['RJ/Down'])
        keypad_msg.RJ_Button = int(F130_Keypad.states['RJ/Button'])
        keypad_msg.LJ_Left = int(F130_Keypad.states['LJ/Left'])
        keypad_msg.LJ_Right = int(F130_Keypad.states['LJ/Right'])
        keypad_msg.LJ_Up = int(F130_Keypad.states['LJ/Up'])
        keypad_msg.LJ_Down = int(F130_Keypad.states['LJ/Down'])
        keypad_msg.LJ_Button = int(F130_Keypad.states['LJ/Button'])

        # Keypad.time = time_since_up * 1000 *1000
        try:
            keypad_publisher.publish(keypad_msg)
        except:
            print "exception catched"
