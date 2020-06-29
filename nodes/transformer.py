#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import Float64


class transformer:

  angle = 0

  def callback(self,data):
    self.angle = data.data


  def __init__(self):
    rospy.Subscriber("/servo_pkg/servo_angle", Float64, self.callback)

def main():
  rospy.init_node('transformer')
  t = transformer()

  br = tf.TransformBroadcaster()
  rate = rospy.Rate(10.0)

  while not rospy.is_shutdown():
    #t = rospy.Time.now().to_sec() * math.pi
    #rospy.loginfo(t.angle)

    quaternion = tf.transformations.quaternion_from_euler(t.angle, 0, 0, 'sxyz')
    #rospy.loginfo(quaternion)
    br.sendTransform((0.175, 0.0, 0.0485),quaternion,rospy.Time.now(),"velodyne_base","base_link")
    rate.sleep()


if __name__ == '__main__':
  main()