#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped,Pose

class transformer:

  angle = 0

  def callback(self,data):
    br = tf.TransformBroadcaster()
    self.angle = data.data
    quaternion = tf.transformations.quaternion_from_euler(self.angle, 0, 0, 'sxyz')

    br.sendTransform((0.175, 0.0, 0.0485),quaternion,rospy.Time.now(),"velodyne_base","base_link")

  def position_callback(self,data):
    br = tf.TransformBroadcaster()
    self.position = data.pose
    pos = (self.position.position.x,self.position.position.y,self.position.position.z) 
    ori = (self.position.orientation.x,self.position.orientation.y,self.position.orientation.z,self.position.orientation.w)
    br.sendTransform(pos, ori, rospy.Time.now(),"base_link","map")

  def __init__(self):
    self.position = Pose()
    rospy.Subscriber("/servo_pkg/servo_angle", Float64, self.callback)
    rospy.Subscriber("/position_drone", PoseStamped, self.position_callback)

def main():
  rospy.init_node('transformer')
  t = transformer()
  rospy.spin()


  # br = tf.TransformBroadcaster()
  # rate = rospy.Rate(10.0)

  # while not rospy.is_shutdown():
    #t = rospy.Time.now().to_sec() * math.pi
    #rospy.loginfo(t.angle)
    # print("Sending transform for velodyne_base")
    #rospy.loginfo(quaternion)
    
    
    
    # rate.sleep()

if __name__ == '__main__':
  main()