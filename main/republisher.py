# ROS node that republishes a topic with a different message type
# the message to be republihsed is a ArmPose message with the following fields:
# float32 q1   # joint 1 target angle 
# float32 q2   # joint 2 target angle 
# float32 q3   # joint 3 target angle
# float32 d4   # elevation
# float32 q5   # gripper angle
# float32 succ # gripper force
# the message shall be republished as individual floats for each joint

import rospy
from arm_pose.msg import ArmPose
from std_msgs.msg import Float32

def callback(data):
    pub1.publish(data.q1)
    pub2.publish(data.q2)
    pub3.publish(data.q3)
    pub4.publish(data.d4)
    pub5.publish(data.q5)
    pub6.publish(data.succ)

def republisher():
    rospy.init_node('republisher', anonymous=True)
    rospy.Subscriber("arm_pose", ArmPose, callback)
    rospy.spin()

if __name__ == '__main__':
    pub1 = rospy.Publisher('arm_pose_target_re/q1', Float32, queue_size=2)
    pub2 = rospy.Publisher('arm_pose_target_re/q2', Float32, queue_size=2)
    pub3 = rospy.Publisher('arm_pose_target_re/q3', Float32, queue_size=2)
    pub4 = rospy.Publisher('arm_pose_target_re/d0', Float32, queue_size=2)
    pub5 = rospy.Publisher('arm_pose_target_re/q5', Float32, queue_size=2)
    pub6 = rospy.Publisher('arm_pose_target_re/succ', Float32, queue_size=2)
    republisher()