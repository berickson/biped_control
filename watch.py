#!/usr/bin/env python  
import rospy
import tf
import geometry_msgs

def main():
    rospy.init_node('transform_listener')
    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    from_frame = 'base_link'
    to_frame = 'left_foot'
    listener.waitForTransform(from_frame, to_frame, rospy.Time(0), rospy.Duration(4))
    while True:
        try:
            geometry_msgs
            (trans,rot) = listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
            print(from_frame+'->'+to_frame)
            print('trans',trans)
            print('rot',rot)
            print('')

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('exception')

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
