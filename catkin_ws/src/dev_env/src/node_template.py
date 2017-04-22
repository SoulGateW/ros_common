#!/usr/bin/env python

import rospy
import sys
import signal

TAG = "Node Template:"

#------------------------------- NODE TEMPLATE --------------------------------#

# SIGINT Signal handler
def sigint_handler(signal, frame):
        print ""
        print TAG,"Interrupt!"
        print TAG,"Terminated"
        sys.exit(0)

if __name__ == '__main__':
    print TAG,"Started"
    # Assigning the SIGINT handler.
    signal.signal(signal.SIGINT, sigint_handler)
    # Starting the node
    rospy.init_node('node_template', anonymous=True)
    rospy.spin()

    print TAG,"Terminated"
