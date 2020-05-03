#!/usr/bin/env python  
import time
import math

import rospy
from std_msgs.msg import Float64

from ik import get_joint_angles_for_position

def square_curve(elapsed, cx, rx, cy, ry):
    (f, n) = math.modf(elapsed)
    side = n % 4

    if(side==0):
        # left
        x = cx-rx
        y = cy-ry + 2 * f * ry
    if(side==1):
        # top
        x = cx-rx + 2 * f * rx
        y = cy+ry

    if(side==2):
        # right
        x = cx+rx
        y = cy+ry - 2 * f * ry

    if(side==3):
        # bottom
        x = cx+rx - 2 * f * rx
        y = cy-ry
    return (x,y)

def ellipse_curve(elapsed, cx, rx, cy, ry):
    theta = elapsed
    x = cx + rx * math.sin(theta)
    y = cy + ry * math.cos(theta)
    return (x,y)




def main():
    start = time.time()
    left_knee_controller = rospy.Publisher('/biped/left_knee_position_controller/command', Float64, queue_size=1)
    left_ankle_controller = rospy.Publisher('/biped/left_ankle_position_controller/command', Float64, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # hz

    cx = 0.0
    rx = 0.05
    cy = 0.2
    ry = 0.05

    while not rospy.is_shutdown():
        elapsed = time.time()-start


        #left_knee_controller.publish(math.sin(elapsed))
        #left_ankle_controller.publish(-math.sin(elapsed)/3)

        # draw a circle in the air
        #(x,y) = ellipse_curve(elapsed, cx, rx, cy, ry)

        # draw a square in the air
        (x,y) = square_curve(elapsed, cx, rx, cy, ry)


        (a,b) = get_joint_angles_for_position(x, -y, 0.2, 0.2, trace=False)
        left_ankle_controller.publish(a)
        left_knee_controller.publish(b)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass