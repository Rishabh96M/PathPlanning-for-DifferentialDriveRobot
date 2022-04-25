import rospy
from geometry_msgs.msg import Twist
import time


def send_vel(actions, radius, w_dia, step):
    rospy.init_node('robot_talker', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    msg = Twist()
    for point, move in actions:
        cnt = 0
        msg.angular.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        pub.publish(msg)
        time.sleep(0.1)
        print(move[0], move[1])
        while cnt < 50:
            cnt += 1
            ang_v = (radius / w_dia) * (move[1] - move[0])
            lin_v = 0.5 * radius * (move[0] + move[1])
            msg.angular.z = ang_v / 4.5
            msg.angular.x = 0
            msg.angular.y = 0
            msg.linear.x = lin_v / 455
            msg.linear.y = 0
            msg.linear.z = 0
            pub.publish(msg)
            time.sleep(0.1)
