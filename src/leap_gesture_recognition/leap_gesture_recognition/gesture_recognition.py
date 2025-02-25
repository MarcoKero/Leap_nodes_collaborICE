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
        #self.get_logger().info('Joint names and positions updated.')
        #print(self.joint_names)
        handle_joints(self.joint_names, self.joint_positions)


def specific_joints_return(joint_names, joint_positions, target_joints):
    vector_joints=[]
    #print(joint_positions)
    for name, position in zip(joint_names, joint_positions):
        if name in target_joints:
            vector_joints.append(position)
            #print(f"Tracked Joint: {name}, Position: {position}")
    return vector_joints

def create_xyz_target_joint(name):
    return [str(name)+'_x',str(name)+'_y',str(name)+'_z']

def create_xyzw_target_joint(name):
    return [str(name)+'_x',str(name)+'_y',str(name)+'_z',str(name)+'_w']

def handle_joints(joint_names,joint_positions):
    target_joints = ["left_palm", "thumb_tip"]
    for t_j in target_joints:
        target=create_xyz_target_joint(t_j)
        #print(target)
        this_target_joint_positions=specific_joints_return(joint_names, joint_positions, target)

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
