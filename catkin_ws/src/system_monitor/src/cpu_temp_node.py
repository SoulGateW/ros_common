#!/usr/bin/env python

import rospy
import sys
import signal
import subprocess
import time
from std_msgs.msg import Header
from sensor_msgs.msg import Temperature

TAG = "CPU Temperature:"

#------------------------------- NODE TEMPLATE --------------------------------#

# SIGINT Signal handler
def sigint_handler(signal, frame):
        print ""
        print TAG,"Interrupt!"
        print TAG,"Terminated"
        sys.exit(0)

if __name__ == '__main__':
    print TAG,"Started"
    isInt = False
    refresh_rate = 1.0
    # Getting the refresh rate
    if len(sys.argv) > 1:
        refresh_rate = float(sys.argv[1])
    # Checking if output is expected to be int or float
    if len(sys.argv) > 2:
        if sys.argv[2] == "int":
            isInt = True
        elif sys.argv[2] == "float":
            isInt = False

    # Assigning the SIGINT handler.
    signal.signal(signal.SIGINT, sigint_handler)
    # Starting the node
    rospy.init_node("cpu_temp")
    pub = rospy.Publisher("~temp",Temperature, queue_size=10)
    # Starting the main cycle
    rate = rospy.Rate(refresh_rate)
    while not rospy.is_shutdown():
        # Getting CPU temperature
        p = subprocess.Popen(["cat", "/sys/devices/virtual/thermal/thermal_zone0/temp"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()
        temp = float(out)
        if isInt == False:
            temp = temp / 1000.0
        print temp

        h = Header()
        h.stamp = rospy.Time.now()

        t = Temperature()
        t.header = h
        t.temperature = temp
        t.variance = 0

        pub.publish(t)

        rate.sleep()

    print TAG,"Terminated"
