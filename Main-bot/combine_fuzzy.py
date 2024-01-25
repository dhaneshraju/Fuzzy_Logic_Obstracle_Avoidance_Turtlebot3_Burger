import rclpy
import math
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from pid import pid_obj
from fuzzy import fuzzy
from oa import oa

mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front1': 0,
    'front2': 0,
    'fleft': 0,
    'fback': 0,
    'left': 0,
}
twstmsg_ = None


# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if (twstmsg_ != None):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_

    regions_ = {
        # LIDAR readings are anti-clockwise
        'front1': find_nearest(msg.ranges[0:5]),
        'fro': find_nearest(msg.ranges[0:5]),
        'front2': find_nearest(msg.ranges[355:360]),
        'right': find_nearest(msg.ranges[265:275]),
        'fright': find_nearest(msg.ranges[310:320]),
        'fback': find_nearest(msg.ranges[220:230]),
        'fleft': find_nearest(msg.ranges[40:50]),
        'left': find_nearest(msg.ranges[85:95])

    }
    twstmsg_ = movement()


# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)


# Basic movement method


def movement():
    # print("here")
    global regions_, mynode_
    regions = regions_

    # print("Min distance in front region: ", regions_['fleft'], regions_['fright'])

    # create an object of twist class, used to express the linear and angular velocity of the turtlebot
    msg = Twist()

    # calling the pid function and passing the parameters

    # calling the fuzzy logic and passing the parameters
    pid_linear = 0.8
    pid_angular= pid_obj.pidcontroller(regions_['fright'])

    fuzzy_temp = list(fuzzy.fuzzy_crisp(regions_['fright'], regions_['fback']))
    linear_fuzzy = fuzzy_temp[0]
    angular_fuzzy = fuzzy_temp[1]


    oa_temp = oa.oa_calculation(regions_['fleft'], regions_['fright'], regions_['front1'], regions_['front2'], regions_['fro'])
    linear_oa = oa_temp[0]
    angular_oa = oa_temp[1]

    x1 = [pid_linear, linear_fuzzy, linear_oa]
    x2 = [pid_angular, angular_fuzzy, angular_oa]
    total_linear = np.mean(x1)
    total_angular = np.mean(x2)

    # calling the obstracle avoidance and passing the parameters

    # If an obstacle is found to be within 0.25 of the LiDAR sensors front region the linear velocity is set to 0 (turtlebot stops)

    msg.linear.x = total_linear
    msg.angular.z = total_angular
    return msg


# used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds
    timer = mynode_.create_timer(timer_period, timer_callback)

    global ei, ed, eprevious
    ei, ed, eprevious = 0, 0, 0

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
