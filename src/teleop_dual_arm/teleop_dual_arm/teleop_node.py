import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import cv2
import mediapipe as mp
import numpy as np
import math
from tf_transformations import quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError

class TeleoperationNode(Node):
    def __init__(self):
        super().__init__('teleop_dual_arm_node')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "image_topic", 10)

        # Publishers for left and right arm end-effector poses
        self.left_arm_pose_pub = self.create_publisher(PoseStamped, 'left/arm_servo_node/left_arm_pose_cmds', 10)
        self.right_arm_pose_pub = self.create_publisher(PoseStamped, '/right/arm_servo_node/right_arm_pose_cmds', 10)

        # Action clients for gripper commands
        self.left_gripper_client = ActionClient(self, GripperCommand, '/left_hand_controller/gripper_cmd')
        self.right_gripper_client = ActionClient(self, GripperCommand, '/right_hand_controller/gripper_cmd')

        # MediaPipe hand tracking setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils

        # OpenCV camera capture
        self.cap = cv2.VideoCapture(0)

        # Timer for processing frames at 10Hz
        self.timer = self.create_timer(0.1, self.process_frame)
        self.h, self.w = 0, 0
        self.right_gripper_state = 0.05
        self.left_gripper_state = 0.05

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to read from camera.")
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        # get width and height of the frame
        self.h, self.w, _ = frame.shape

        if results.multi_hand_landmarks:
            for hand_landmarks, hand_info in zip(results.multi_hand_landmarks, results.multi_handedness):
                handedness = hand_info.classification[0].label
                frame_id = 'left_panda_link0' if handedness == 'Left' else 'right_panda_link0'  # Dynamic frame_id

                pose_msg = self.create_pose_msg(hand_landmarks, frame_id)
                gripper_command = self.create_gripper_command(hand_landmarks)

                if handedness == 'Left':
                    self.left_arm_pose_pub.publish(pose_msg)
                    if self.left_gripper_state != gripper_command.command.position:
                        self.send_gripper_command(self.left_gripper_client, gripper_command)
                        self.left_gripper_state = gripper_command.command.position
                else:
                    self.right_arm_pose_pub.publish(pose_msg)
                    if self.right_gripper_state != gripper_command.command.position:
                        self.right_gripper_state = gripper_command.command.position
                        self.send_gripper_command(self.right_gripper_client, gripper_command)

                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
        frame = cv2.line(frame, (0, int(self.h / 2)), (self.w, int(self.h / 2)), (0, 255, 0), 2)
        frame = cv2.line(frame, (int(self.w / 2), 0), (int(self.w / 2), self.h), (0, 255, 0), 2)  

        frame = cv2.line(frame, (int(self.w / 4), 0), (int(self.w / 4), self.h), (0, 255, 0), 2)
        frame = cv2.line(frame, (int(self.w * 3 / 4), 0), (int(self.w * 3 / 4), self.h), (0, 255, 0), 2)



        self.image_message = self.bridge.cv2_to_imgmsg(frame, "passthrough")
        self.image_pub.publish(self.image_message)
        # cv2.imshow('Teleoperation - Camera Feed', frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     self.destroy_node()
        #     cv2.destroyAllWindows()

    def create_pose_msg(self, hand_landmarks, frame_id):
        # Adjust scaling factors and offsets based on actual workspace
        z_scale, y_scale = 1.5, 1.5 # Adjusted scaling factors
        rotation_z_scale = 2.0  # Adjusted scaling factors
        x_offset, y_offset = 0,0  # Adjusted offsets

        # Correct the axes mapping based on observed behavior
        # Current: x (camera) -> y (robot), y (camera) -> -x (robot)
        # Inversion (-) added where necessary to correct directions
        robot_z = 1 - hand_landmarks.landmark[0].y  # Corrected for forward/backward
        robot_y = 0
        if frame_id == 'left_panda_link0':
            robot_y = 0.25 - hand_landmarks.landmark[0].x
        else:
            robot_y = 0.75 - hand_landmarks.landmark[0].x     # Corrected for left/right

        a = np.array([hand_landmarks.landmark[0].x, hand_landmarks.landmark[0].y]) # First coord
        b = np.array([hand_landmarks.landmark[5].x, hand_landmarks.landmark[5].y]) # Second coord
        c = np.array([hand_landmarks.landmark[17].x, hand_landmarks.landmark[17].y]) # Third coord
        
        radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
        angle = np.abs(radians*180.0/np.pi)
        
        if angle > 180.0:
            angle = 360-angle

        staright_hande_angle = 65

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = 0.6
        pose.pose.position.y = -round((y_scale * robot_y ), 1)
        pose.pose.position.z = round(z_scale * robot_z, 1) if robot_z > 0 else 0.0  # Slightly above base for safety

        # Simple rotation along z-axis
        # convert rads to quaternion
        # print(math.radians(rotation_z))
        q = quaternion_from_euler(math.radians(180 + rotation_z_scale * math.ceil((staright_hande_angle - angle)/5) * 5), 0, 0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def create_gripper_command(self, hand_landmarks):
        # Simple threshold on distance between thumb and index finger for open/close
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

        distance = ((thumb_tip.x - index_tip.x) ** 2 + (thumb_tip.y - index_tip.y) ** 2) ** 0.5
        gripper_closed = distance < 0.05  # Threshold value

        command = GripperCommand.Goal()
        command.command.position = 0.0 if gripper_closed else 0.05  # Adjust based on gripper range
        command.command.max_effort = 10.0  # Adjust based on gripper specifications

        return command

    def send_gripper_command(self, client, command):
        client.wait_for_server(timeout_sec=0.2)
        future = client.send_goal_async(command)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # self.get_logger().info('Goal rejected')
            return

        # self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # self.get_logger().info(f'Gripper command result: {result}')

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleoperationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()