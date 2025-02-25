import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class LeapGestureRecognition(Node):

    def __init__(self):
        super().__init__('leap_gesture_recognition')
        self.subscription = self.create_subscription(JointState, '/leapmotion/joints', self.listener_callback, 1)
        self.subscription  # prevent unused variable warning
        self.joint_names = []
        self.joint_positions = []

    def listener_callback(self, msg):
        self.joint_names = msg.name
        self.joint_positions = msg.position
        self.get_logger().info('Joint names and positions updated.')
        print(self.joint_names)


def main(args=None):
    rclpy.init(args=args)

    leap_gesture_recognition = LeapGestureRecognition()

    rclpy.spin(leap_gesture_recognition)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    leap_gesture_recognition.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
