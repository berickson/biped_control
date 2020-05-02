#!/usr/bin/env python  
import time
import math

import rospy
from std_msgs.msg import Float64

def main():
    start = time.time()
    left_knee_controller = rospy.Publisher('/biped/left_knee_position_controller/command', Float64, queue_size=1)
    left_ankle_controller = rospy.Publisher('/biped/left_ankle_position_controller/command', Float64, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # hz
    while not rospy.is_shutdown():
        elapsed = time.time()-start
        left_knee_controller.publish(math.sin(elapsed))
        left_ankle_controller.publish(-math.sin(elapsed)/3)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass