import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


class LeapGestureRecognition(Node):

    def __init__(self):
        super().__init__('leap_fusion')
        self.subscription1 = self.create_subscription(JointState, '/leapmotion1/joints', self.listener_callback_1, 1)
        self.subscription2 = self.create_subscription(JointState, '/leapmotion2/joints', self.listener_callback_2, 1)

        self.hand_left_leap1 = []
        self.hand_right_leap1 = []
        self.last_update_leap1=self.get_clock().now().seconds_nanoseconds()[0]

        self.hand_left_leap2 = []
        self.hand_right_leap2 = []
        self.last_update_leap2=self.get_clock().now().seconds_nanoseconds()[0]

        self.publisher_ = self.create_publisher(JointState, '/leap_fusion/joints', 1)
        self.timer = self.create_timer(0.1, self.publish_joints)  # Timer for publishing data

        self.time_passed = self.get_clock().now().seconds_nanoseconds()[0]#rclpy.duration.Duration(seconds=1)

    def listener_callback_1(self, msg):
        print('call1')

        this_time = rclpy.time.Time().seconds_nanoseconds()[0]
        if True :# this_time > self.last_update_leap1:
            self.last_update_leap1 = this_time

            left_hand, right_hand = separate_hands(msg.position, msg.name)
            self.hand_left_leap1 = self.transform_coordinates(left_hand)
            self.hand_right_leap1 = self.transform_coordinates(right_hand)

            self.fuse_data()

    def listener_callback_2(self, msg):
        print('call2')
        this_time = rclpy.time.Time().seconds_nanoseconds()[0]
        if this_time > self.last_update_leap2:
            self.last_update_leap2 = this_time

            left_hand, right_hand = separate_hands(msg.position, msg.name)
            self.hand_left_leap2 = left_hand
            self.hand_right_leap2 = right_hand
            #self.fuse_data()

    def publish_joints(self):
        pass  # The listener handles publishing data

    def fuse_hand(self, hand1, hand2):
        if hand1 is not None and hand2 is not None:
            return (np.array(hand1) + np.array(hand2)) / 2
        return np.array(hand1 or hand2) if (hand1 or hand2) else None

    def fuse_data(self):
        """ Fuse the hand data from both Leap Motion devices """
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        if (current_time - self.last_update_leap1 >= self.time_passed and
                current_time - self.last_update_leap2 >= self.time_passed):
            return

        fused_left_hand = self.fuse_hand(self.hand_left_leap1, self.hand_left_leap2)
        fused_right_hand = self.fuse_hand(self.hand_right_leap1, self.hand_right_leap2)

        self.send_fused_data(fused_left_hand, fused_right_hand)

    def transform_coordinates(self, joint_positions):
        """ Apply a rotation and translation to the second Leap Motion data """
        # Define the rotation matrix and translation vector (to be set according to Leap positioning)
        palm_rot = joint_positions[3:7]  # Elements from index 3 to 6 (Python slicing is exclusive on the end index)
        joint_positions = joint_positions[:3] + joint_positions[7:]  # Everything before index 3 and after index 6
        R = np.eye(3)  # Rotation matrix (to be determined)
        T = np.array([0.0, 0.0, 0.0])  # Translation vector (to be determined)

        joint_positions = np.array(joint_positions).reshape(-1, 3)  # Convert to Nx3 array
        transformed_positions = np.dot(joint_positions, R.T) + T  # Apply rotation and translation
        transformed_positions=transformed_positions.flatten()
        newlist = list(transformed_positions[:3])
        newlist.extend(palm_rot)
        newlist.extend(list(transformed_positions[7:]))

        return  newlist # Return as a 1D array

    def publish_new_data(self, fused_left_hand, fused_right_hand):
        joint_msg = JointState()
        joint_msg.header.stamp = rclpy.time.Time().to_msg()
        if fused_left_hand:
            hand_type = 'left'
            append_joints(fused_left_hand, hand_type, joint_msg)  # Append detected joint positions
        if fused_right_hand:
            hand_type = 'right'
            append_joints(fused_right_hand, hand_type, joint_msg)  # Append detected joint positions

        if joint_msg.name:
            self.publisher_.publish(joint_msg)
            print('Published fused data')

    '''def send_fused_data(self, ):
        fused_joint_state = JointState()
        fused_joint_state.name = self.joint_names_1  #+ self.joint_names_2  # Combine names
        fused_joint_state.position = self.joint_positions_1  #+ self.joint_positions_2  # Combine positions
        fused_joint_state.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(fused_joint_state)
        print('Published fused data')'''


# Helper function to append specific joint value to a message
def append_specific_joint(hand_type, name, obj_to_append, joint_msg):
    joint_msg.name.append(hand_type + name + '_x')
    joint_msg.position.append(obj_to_append.x)
    joint_msg.name.append(hand_type + name + '_y')
    joint_msg.position.append(obj_to_append.y)
    joint_msg.name.append(hand_type + name + '_z')
    joint_msg.position.append(obj_to_append.z)


# Function to append all hand joints to the message
def append_joints(hand, hand_type, joint_msg):
    names = ['left_palm_x', 'left_palm_y', 'left_palm_z', 'left_palm_rot_x', 'left_palm_rot_y', 'left_palm_rot_z',
             'left_palm_rot_w', 'left_wrist_x', 'left_wrist_y', 'left_wrist_z', 'left_thumb_metacarpal_x',
             'left_thumb_metacarpal_y', 'left_thumb_metacarpal_z', 'left_thumb_proximal_x', 'left_thumb_proximal_y',
             'left_thumb_proximal_z', 'left_thumb_intermediate_x', 'left_thumb_intermediate_y',
             'left_thumb_intermediate_z',
             'left_thumb_distal_x', 'left_thumb_distal_y', 'left_thumb_distal_z', 'left_index_metacarpal_x',
             'left_index_metacarpal_y', 'left_index_metacarpal_z', 'left_index_proximal_x', 'left_index_proximal_y',
             'left_index_proximal_z', 'left_index_intermediate_x', 'left_index_intermediate_y',
             'left_index_intermediate_z',
             'left_index_distal_x', 'left_index_distal_y', 'left_index_distal_z', 'left_middle_metacarpal_x',
             'left_middle_metacarpal_y', 'left_middle_metacarpal_z', 'left_middle_proximal_x', 'left_middle_proximal_y',
             'left_middle_proximal_z', 'left_middle_intermediate_x', 'left_middle_intermediate_y',
             'left_middle_intermediate_z',
             'left_middle_distal_x', 'left_middle_distal_y', 'left_middle_distal_z', 'left_ring_metacarpal_x',
             'left_ring_metacarpal_y', 'left_ring_metacarpal_z', 'left_ring_proximal_x', 'left_ring_proximal_y',
             'left_ring_proximal_z', 'left_ring_intermediate_x', 'left_ring_intermediate_y', 'left_ring_intermediate_z',
             'left_ring_distal_x', 'left_ring_distal_y', 'left_ring_distal_z', 'left_pinky_metacarpal_x',
             'left_pinky_metacarpal_y', 'left_pinky_metacarpal_z', 'left_pinky_proximal_x', 'left_pinky_proximal_y',
             'left_pinky_proximal_z', 'left_pinky_intermediate_x', 'left_pinky_intermediate_y',
             'left_pinky_intermediate_z',
             'left_pinky_distal_x', 'left_pinky_distal_y', 'left_pinky_distal_z']
    if hand_type == 'right':
        names = ['right_palm_x', 'right_palm_y', 'right_palm_z', 'right_palm_rot_x', 'right_palm_rot_y',
                 'right_palm_rot_z', 'right_palm_rot_w', 'right_wrist_x', 'right_wrist_y', 'right_wrist_z',
                 'right_thumb_metacarpal_x', 'right_thumb_metacarpal_y', 'right_thumb_metacarpal_z',
                 'right_thumb_proximal_x', 'right_thumb_proximal_y', 'right_thumb_proximal_z',
                 'right_thumb_intermediate_x', 'right_thumb_intermediate_y', 'right_thumb_intermediate_z',
                 'right_thumb_distal_x', 'right_thumb_distal_y', 'right_thumb_distal_z', 'right_index_metacarpal_x',
                 'right_index_metacarpal_y', 'right_index_metacarpal_z', 'right_index_proximal_x',
                 'right_index_proximal_y', 'right_index_proximal_z', 'right_index_intermediate_x',
                 'right_index_intermediate_y', 'right_index_intermediate_z', 'right_index_distal_x',
                 'right_index_distal_y', 'right_index_distal_z', 'right_middle_metacarpal_x',
                 'right_middle_metacarpal_y', 'right_middle_metacarpal_z', 'right_middle_proximal_x',
                 'right_middle_proximal_y', 'right_middle_proximal_z', 'right_middle_intermediate_x',
                 'right_middle_intermediate_y', 'right_middle_intermediate_z', 'right_middle_distal_x',
                 'right_middle_distal_y', 'right_middle_distal_z', 'right_ring_metacarpal_x', 'right_ring_metacarpal_y',
                 'right_ring_metacarpal_z', 'right_ring_proximal_x', 'right_ring_proximal_y', 'right_ring_proximal_z',
                 'right_ring_intermediate_x', 'right_ring_intermediate_y', 'right_ring_intermediate_z',
                 'right_ring_distal_x', 'right_ring_distal_y', 'right_ring_distal_z', 'right_pinky_metacarpal_x',
                 'right_pinky_metacarpal_y', 'right_pinky_metacarpal_z', 'right_pinky_proximal_x',
                 'right_pinky_proximal_y', 'right_pinky_proximal_z', 'right_pinky_intermediate_x',
                 'right_pinky_intermediate_y', 'right_pinky_intermediate_z', 'right_pinky_distal_x',
                 'right_pinky_distal_y', 'right_pinky_distal_z']
    for elem in hand:
        joint_msg.position.append(elem)
    for elem in names:
        joint_msg.name.append(elem)

def separate_hands(joint_list, names):
    total_joints = len(joint_list)

    if total_joints == 0:
        return [], []  # No data received

    # If it's a full dataset, split it in half
    if total_joints > 100:
        half = total_joints // 2
        first_hand = "left" if names[0].startswith("left_") else "right"

        if first_hand == "left":
            return joint_list[:half], joint_list[half:]
        else:
            return joint_list[half:], joint_list[:half]

    # If only one hand is present
    first_hand = "left" if names[0].startswith("left_") else "right"
    if first_hand == "left":
        return joint_list, []
    else:
        return [], joint_list


def specific_joints_return(joint_names, joint_positions, target_joints):
    vector_joints = []
    for name in joint_names:
        if name in target_joints:
            position = joint_names.index(name)
            vector_joints.append(joint_positions[position])
            #print(f"Tracked Joint: {name}, Position: {position}")
    return vector_joints


def create_xyz_target_joint(name):
    return [str(name) + '_x', str(name) + '_y', str(name) + '_z']


def create_xyzw_target_joint(name):
    return [str(name) + '_x', str(name) + '_y', str(name) + '_z', str(name) + '_w']


def handle_joints(joint_names, joint_positions):
    target_joints = ["left_palm", "thumb_tip"]
    for t_j in target_joints:
        target = create_xyz_target_joint(t_j)
        specific_joints_return(joint_names, joint_positions, target)


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
