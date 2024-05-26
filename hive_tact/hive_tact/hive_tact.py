import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Int16MultiArray, Int64MultiArray, Int8
from nav_msgs.msg import Odometry
import RPi.GPIO as GPIO
import numpy as np
import sys

def quaternion_to_euler_angle(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return yaw_z, pitch_y, roll_x

class ButtonPublisher(Node):

    def __init__(self):
        super().__init__('button_publisher')

        # 파라미터로부터 로봇 번호 설정
        self.robot_num = self.declare_parameter('robot_num', 1).get_parameter_value().integer_value

        # 로봇 번호를 사용하여 토픽명 설정
        self.grid_odom_topic = f'/robot{self.robot_num}/grid_odom'
        self.odom_topic = f'/robot{self.robot_num}/odom'

        self.red_btn = 23
        self.blue_btn = 24

        self.red_temp = False
        self.blue_temp = False
        self.curr_pose = (0, 0)
        self.curr_pose_control = (0.0, 0.0)
        self.curr_yaw = 0.0

        self.parking_goal_topic = f'/robot{self.robot_num}/parkingGoal'
        self.serving_goal_topic = f'/robot{self.robot_num}/servingGoal'
        self.serve_btn_topic = f'/robot{self.robot_num}/serve_btn'

        self.parking_publisher_ = self.create_publisher(Int16, self.parking_goal_topic, 10)
        self.serving_publisher_ = self.create_publisher(Int64MultiArray, self.serving_goal_topic, 10)
        self.serve_btn_publisher_ = self.create_publisher(Int8, self.serve_btn_topic, 10)

        self.gridOdom_subscription = self.create_subscription(Int16MultiArray, self.grid_odom_topic, self.gridOdom_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.red_btn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.blue_btn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def timer_callback(self):
        if GPIO.input(self.red_btn) == GPIO.HIGH:
            self.red_temp = True
        else:
            if self.red_temp:
                self.red_temp = False
                self.publish_parking_goal()

        if GPIO.input(self.blue_btn) == GPIO.HIGH:
            self.blue_temp = True
        else:
            if self.blue_temp:
                self.blue_temp = False
                self.publish_serving_goal()
                self.publish_serve_btn()

    def gridOdom_callback(self, msgs):
        self.curr_pose = (int(msgs.data[0] / 4), int(msgs.data[1] / 4))
        self.get_logger().info(f'Received grid odom: {self.curr_pose}')

    def odom_callback(self, msgs):
        """
        '/robot{self.robot_num}/odom' 토픽의 콜백 함수입니다. 현재 위치와 방향을 업데이트합니다.
        """
        self.curr_pose_control = tuple(np.array([msgs.pose.pose.position.x, msgs.pose.pose.position.y]).astype(float))
        self.curr_ori = msgs.pose.pose.orientation
        self.curr_yaw, _, _ = quaternion_to_euler_angle(self.curr_ori)
        self.get_logger().info(f'Received odom: position={self.curr_pose_control}, yaw={self.curr_yaw}')

    def publish_parking_goal(self):
        msg = Int16()
        msg.data = self.robot_num
        self.parking_publisher_.publish(msg)
        self.get_logger().info(f'Published parking goal: {msg.data}')

    def publish_serving_goal(self):
        msg = Int64MultiArray()
        msg.data = [int(self.robot_num), int(self.curr_pose[0]), int(self.curr_pose[1]), int(self.curr_yaw)]
        self.serving_publisher_.publish(msg)
        self.get_logger().info(f'Published serving goal: {msg.data}')

    def publish_serve_btn(self):
        msg = Int8()
        msg.data = 1
        self.serve_btn_publisher_.publish(msg)
        self.get_logger().info(f'Published serve button press: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ButtonPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()

