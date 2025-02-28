import leap
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
from leap.events import TrackingEvent
from typing import Callable
from timeit import default_timer as timer

# List of finger and joint names for reference
fingers_name = ['thumb', 'index', 'middle', 'ring', 'pinky']
joints_name = ['metacarpal', 'proximal', 'intermediate', 'distal']

# Listener class to monitor Leap Motion device events
class MultiDeviceListener(leap.Listener):
    def __init__(self, event_type):
        super().__init__()
        self._event_type = event_type
        self.n_events = 0  # Counter for the number of events received

    def on_event(self, event):
        # Increment event count if the event is of the specified type
        if isinstance(event, self._event_type):
            self.n_events += 1

# Utility function to wait for a condition to become true within a timeout period
def wait_until(condition: Callable[[], bool], timeout: float = 5, poll_delay: float = 0.01):
    start_time = timer()
    while timer() - start_time < timeout:
        if condition():
            return True
        time.sleep(poll_delay)
    return condition()

# Function to retrieve and subscribe to active Leap Motion devices
def get_updated_devices(connection):
    devices = connection.get_devices()
    for device in devices:
        with device.open():
            connection.subscribe_events(device)

# ROS2 Node that publishes Leap Motion joint state data
class LeapMotionPublisher(Node):
    def __init__(self):
        super().__init__('leapmotion_publisher')
        self.publisher1_ = self.create_publisher(JointState, '/leapmotion1/joints', 1)
        self.publisher2_ = self.create_publisher(JointState, '/leapmotion2/joints', 1)
        self.timer = self.create_timer(0.1, self.publish_joints)  # Timer for publishing data

        # Create event listeners for tracking and device events
        self.tracking_listener = TrackingEventListener([self.publisher1_, self.publisher2_])
        self.device_listener = MultiDeviceListener(leap.events.DeviceEvent)

        self.connection = leap.Connection(multi_device_aware=True)
        self.connection.add_listener(self.tracking_listener)
        self.connection.add_listener(self.device_listener)

        # Open connection and ensure multiple devices are detected
        with self.connection.open():
            wait_until(lambda: self.device_listener.n_events > 1)

            self.current_device_events = self.device_listener.n_events
            get_updated_devices(self.connection)
            while True:
                if self.device_listener.n_events != self.current_device_events:
                    self.current_device_events = self.device_listener.n_events
                    get_updated_devices(self.connection)

                time.sleep(0.5)  # Delay to prevent excessive polling

    def publish_joints(self):
        pass  # The listener handles publishing data

# Listener for tracking Leap Motion hand movement events
class TrackingEventListener(leap.Listener):
    def __init__(self, publishers):
        super().__init__()
        self.publishers = publishers  # List of ROS publishers
        self.last_framepublished = [0, 0]  # Track last published frame for each device
        self.hands = []  # Store detected hands

    def number_of_devices_tracking(self):
        return len(self.device_latest_tracking_event)

    def on_tracking_event(self, event: TrackingEvent):
        if event.hands:  # Check if hands are detected
            source_device = event.metadata.device_id - 1  # Identify source device
            this_timeframe = event.tracking_frame_id
            if this_timeframe > int(self.last_framepublished[source_device]):  # Ensure new frame
                self.last_framepublished[source_device] = this_timeframe  # Update last frame
                self.hands = event.hands
                self.publish_new_data(source_device)

    def publish_new_data(self, source_device):
        joint_msg = JointState()
        joint_msg.header.stamp = rclpy.time.Time().to_msg()
        for hand in self.hands:
            append_joints(hand, joint_msg)  # Append detected joint positions
        if joint_msg.name:
            self.publishers[source_device].publish(joint_msg)

# Helper function to append specific joint value to a message
def append_specific_joint(hand_type, name, obj_to_append, joint_msg):
    joint_msg.name.append(hand_type + name + '_x')
    joint_msg.position.append(obj_to_append.x)
    joint_msg.name.append(hand_type + name + '_y')
    joint_msg.position.append(obj_to_append.y)
    joint_msg.name.append(hand_type + name + '_z')
    joint_msg.position.append(obj_to_append.z)

# Function to append all hand joints to the message
def append_joints(hand, joint_msg):
    hand_type = 'left' if str(hand.type) == 'HandType.Left' else 'right'

    append_specific_joint(hand_type, '_palm', hand.palm.position, joint_msg)
    append_specific_joint(hand_type, '_palm_rot', hand.palm.orientation, joint_msg)
    joint_msg.name.append(hand_type + '_palm_rot_w')
    joint_msg.position.append(hand.palm.orientation.w)
    append_specific_joint(hand_type, '_wrist', hand.arm.next_joint, joint_msg)

    for fing_count, finger in enumerate(hand.digits):
        this_fing_name = fingers_name[fing_count]
        for joint_count, bone in enumerate(finger.bones):
            this_joint_name = joints_name[joint_count]
            bone_name = f'_{this_fing_name}_{this_joint_name}'
            append_specific_joint(hand_type, bone_name, bone.next_joint, joint_msg)

# Main function to initialize the ROS2 node
def main():
    rclpy.init()
    leapmotion_publisher = LeapMotionPublisher()
    rclpy.spin(leapmotion_publisher)  # Keep the node running
    rclpy.shutdown()

if __name__ == '__main__':
    main()
