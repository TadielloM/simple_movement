#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped,Pose

class transformer:

  angle = 0

  def callback(self,data):
    self.angle = data.data

  def position_callback(self,data):
    self.position = data.pose

  def __init__(self):
    self.position = Pose()
    rospy.Subscriber("/servo_pkg/servo_angle", Float64, self.callback)
    rospy.Subscriber("/position_drone", PoseStamped, self.position_callback)

def main():
  rospy.init_node('transformer')
  t = transformer()

  br = tf.TransformBroadcaster()
  rate = rospy.Rate(10.0)

  while not rospy.is_shutdown():
    #t = rospy.Time.now().to_sec() * math.pi
    #rospy.loginfo(t.angle)
    # print("Sending transform for velodyne_base")
    quaternion = tf.transformations.quaternion_from_euler(t.angle, 0, 0, 'sxyz')
    #rospy.loginfo(quaternion)
    br.sendTransform((0.175, 0.0, 0.0485),quaternion,rospy.Time.now(),"velodyne_base","base_link")
    rate.sleep()
    pos = (t.position.position.x,t.position.position.y,t.position.position.z) 
    ori = (t.position.orientation.x,t.position.orientation.y,t.position.orientation.z,t.position.orientation.w)
    # print (type(t.position.position) , t.position.position)
    # print (type(t.position.orientation),t.position.orientation )
    br.sendTransform(pos, ori, rospy.Time.now(),"base_link","map")


if __name__ == '__main__':
  main()