
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
pub = 0

def joy_callback(data):
    axes = data.axes

    left_left = axes[0]
    left_up = axes[1]

    right_left = axes[3]
    right_up = axes[4]




if __name__ == "__main__":
    rospy.init_node('teleop_node')    
    pub = rospy.Publisher('/manipulator_controller/command', JointTrajectory, queue_size=10)

    rospy.Subscriber('/joy', Joy, joy_callback, Twist)
    rate = rospy.Rate(50)


    while not rospy.is_shutdown():

        rate.sleep()