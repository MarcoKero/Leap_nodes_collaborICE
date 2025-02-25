import leap
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
from leap.events import TrackingEvent
from timeit import default_timer as timer

fingers_name = ['thumb', 'index', 'middle', 'ring', 'pinky']
joints_name = ['metacarpal', 'proximal', 'intermediate', 'distal']

class TrackingEventListener(leap.Listener):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
        self.latest_hands = {'left': None, 'right': None}
        self.latest_timestamp = None

    def publish_fused_data(self):
        if self.latest_hands['left'] is None or self.latest_hands['right'] is None:
            return  # Wait until both hands are available

        joint_msg = JointState()
        joint_msg.header.stamp = rclpy.time.Time().to_msg()

        for hand in [self.latest_hands['left'], self.latest_hands['right']]:
            append_joints(hand, joint_msg)

        if joint_msg.name:
            self.publisher.publish(joint_msg)
            self.latest_hands = {'left': None, 'right': None}  # Reset after publishing

    def on_tracking_event(self, event: TrackingEvent):
        for hand in event.hands:
            hand_type = 'left' if str(hand.type) == 'HandType.Left' else 'right'
            self.latest_hands[hand_type] = hand
        
        if self.latest_hands['left'] and self.latest_hands['right']:
            self.publish_fused_data()


def append_joints(hand, joint_msg):
    hand_type = 'left' if str(hand.type) == 'HandType.Left' else 'right'
    joint_msg.name.append(hand_type + '_palm_x')
    joint_msg.position.append(hand.palm.position.x)
    joint_msg.name.append(hand_type + '_palm_y')
    joint_msg.position.append(hand.palm.position.y)
    joint_msg.name.append(hand_type + '_palm_z')
    joint_msg.position.append(hand.palm.position.z)
    
    for finger, finger_name in zip(hand.digits, fingers_name):
        for bone, joint_name in zip(finger.bones, joints_name):
            joint_msg.name.append(f"{hand_type}_{finger_name}_{joint_name}_x")
            joint_msg.position.append(bone.next_joint.x)
            joint_msg.name.append(f"{hand_type}_{finger_name}_{joint_name}_y")
            joint_msg.position.append(bone.next_joint.y)
            joint_msg.name.append(f"{hand_type}_{finger_name}_{joint_name}_z")
            joint_msg.position.append(bone.next_joint.z)


def main():
    rclpy.init()
    node = Node('leapmotion_fusion_publisher')
    publisher = node.create_publisher(JointState, '/leapmotion_fusion/joints', 1)
    tracking_listener = TrackingEventListener(publisher)
    
    connection = leap.Connection(multi_device_aware=True)
    connection.add_listener(tracking_listener)
    
    with connection.open():
        rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
