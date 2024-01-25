from pid import pid_obj
import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front1': 0,
    'front2': 0,
    'fleft': 0,
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
        'front2': find_nearest(msg.ranges[355:360]),
        'right': find_nearest(msg.ranges[265:275]),
        'fright': find_nearest(msg.ranges[310:320]),
        'fleft': find_nearest(msg.ranges[40:50]),
        'left': find_nearest(msg.ranges[85:95])

    }
    twstmsg_ = movement()


# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)


# def pidcontroller(cd):
#     global ei, ed, eprevious
#     # pid error updating, dd is desired distance and cd is current distance
#     # ei to be a global as 0,
#
#     dd = 1.5
#     error = dd - cd
#     # eprev needs to be updated after
#     print("ei:", ei)
#     print("error:", error)
#     ei = ei + error
#     print("ei:", ei)
#     print("eprevious:", eprevious)
#     # ed also as a global set to 0
#     ed = error - eprevious
#     eprevious = error
#     print("ed:", ed)
#
#     # print("eprevious:", eprevious)
#
#     kp = 0.1
#     ki = 0.00001
#     kd = 0.1
#
#     actual_val = kp * error + ki * ei + kd * ed
#     print("actual_value", actual_val)
#
#     return actual_val


# Basic movement method
def movement():
    # print("here")
    global regions_, mynode_
    regions = regions_

    print("Min distance in front region: ", regions_['front1'], regions_['front2'])

    # create an object of twist class, used to express the linear and angular velocity of the turtlebot
    msg = Twist()

    # If an obstacle is found to be within 0.25 of the LiDAR sensors front region the linear velocity is set to 0 (turtlebot stops)
    msg.linear.x = 0.8
    msg.angular.z = pid_obj.pidcontroller(regions_['fright'])
    # msg.angular.z = pidcontroller(regions_['fright'])
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
