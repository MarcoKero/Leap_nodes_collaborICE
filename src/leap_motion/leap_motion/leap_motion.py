import leap
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time



fingers_name = ['thumb', 'index', 'middle', 'ring', 'pinky']
joints_name = ['metacarpal', 'proximal', 'intermediate', 'distal']

class LeapMotionPublisher(Node):
    def __init__(self):
        super().__init__('leapmotion_publisher')
        self.publisher_ = self.create_publisher(JointState, '/leapmotion/joints', 1)
        self.timer = self.create_timer(0.1, self.publish_joints)
        self.connection = leap.Connection()
        self.listener = MyListener(self.publisher_)
        self.connection.add_listener(self.listener)

        with self.connection.open():
            #    print("Connected")
            self.connection.set_tracking_mode(leap.TrackingMode.Desktop)
            while True:
                time.sleep(1)

    def publish_joints(self):
        pass  # The listener handles publishing


class MyListener(leap.Listener):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher

    def on_connection_event(self, event):
        print("Connected")

    def on_tracking_event(self, event):
        print("Event triggered")
        joint_msg = JointState()
        joint_msg.header.stamp = rclpy.time.Time().to_msg()
        if event.hands != []:
            for hand in event.hands:
                append_joints(hand, joint_msg)
            self.publisher.publish(joint_msg)
        else:
            print("Hand not detected")


def append_joints(hand, joint_msg):
    hand_type = 'left' if str(hand.type) == 'HandType.Left' else 'right'
    joint_msg.name.append(hand_type + '_palm_x')
    joint_msg.position.append(hand.palm.position.x)
    joint_msg.name.append(hand_type + '_palm_y')
    joint_msg.position.append(hand.palm.position.y)
    joint_msg.name.append(hand_type + '_palm_z')
    joint_msg.position.append(hand.palm.position.z)
    fing_count = 0
    for finger in hand.digits:
        this_fing_name = fingers_name[fing_count]
        fing_count += 1
        # print(this_fing_name)
        joint_count = 0
        for bone in finger.bones:
            this_joint_name = joints_name[joint_count]
            joint_count += 1

            # print(this_joint_name)
            joint_msg.name.append(hand_type + '_' + this_fing_name + '_' + this_joint_name + '_x')
            joint_msg.position.append(bone.next_joint.x)
            joint_msg.name.append(hand_type + '_' + this_fing_name + '_' + this_joint_name + '_y')
            joint_msg.position.append(bone.next_joint.y)
            joint_msg.name.append(hand_type + '_' + this_fing_name + '_' + this_joint_name + '_z')
            joint_msg.position.append(bone.next_joint.z)


def main():
    rclpy.init()
    leapmotion_publisher = LeapMotionPublisher()
    rclpy.spin(leapmotion_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
