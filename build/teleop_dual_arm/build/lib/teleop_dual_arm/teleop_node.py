import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import cv2
import mediapipe as mp


class TeleoperationNode(Node):
    def __init__(self):
        super().__init__('teleop_dual_arm_node')

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

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to read from camera.")
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks, hand_info in zip(results.multi_hand_landmarks, results.multi_handedness):
                handedness = hand_info.classification[0].label
                frame_id = 'left_panda_link8' if handedness == 'Left' else 'right_panda_link8'  # Dynamic frame_id

                pose_msg = self.create_pose_msg(hand_landmarks, frame_id)
                gripper_command = self.create_gripper_command(hand_landmarks)

                if handedness == 'Left':
                    self.left_arm_pose_pub.publish(pose_msg)
                    self.send_gripper_command(self.left_gripper_client, gripper_command)
                else:
                    self.right_arm_pose_pub.publish(pose_msg)
                    self.send_gripper_command(self.right_gripper_client, gripper_command)

                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        cv2.imshow('Teleoperation - Camera Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            cv2.destroyAllWindows()

    def create_pose_msg(self, hand_landmarks, frame_id):
        # Adjust scaling factors and offsets based on actual workspace
        x_scale, y_scale = 2, 2  # Adjusted scaling factors
        x_offset, y_offset = 0.2, -0.1  # Adjusted offsets

        # Correct the axes mapping based on observed behavior
        # Current: x (camera) -> y (robot), y (camera) -> -x (robot)
        # Inversion (-) added where necessary to correct directions
        robot_x = -(hand_landmarks.landmark[0].y * y_scale) + x_offset  # Corrected for forward/backward
        robot_y = hand_landmarks.landmark[0].x * x_scale + y_offset     # Corrected for left/right

        # Example rotation measure along z-axis
        rotation_z = hand_landmarks.landmark[5].x - hand_landmarks.landmark[17].x

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = robot_x
        pose.pose.position.y = robot_y
        pose.pose.position.z = 0.1  # Slightly above base for safety

        # Simple rotation along z-axis
        pose.pose.orientation.z = rotation_z
        pose.pose.orientation.w = 1.0  # Unit quaternion

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
        client.wait_for_server()
        future = client.send_goal_async(command)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Gripper command result: {result}')

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