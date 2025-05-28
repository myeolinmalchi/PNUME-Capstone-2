# face_focusing_node.py
import rclpy
from rclpy.node import Node
from control_system_interfaces.msg import TrackedFaces, FaceFocus, SystemState
from control_system.utils.yolo.yolo11 import YOLO11
from control_system.utils.yolo.wrappers import RKNNWrapper


class FaceFocusingNode(Node):

    def __init__(self):
        super().__init__('face_focusing_node')
        self.state = SystemState()

        self.create_subscription(
            SystemState,
            'system_state',
            self.cb_state,
            10,
        )
        self.create_subscription(
            TrackedFaces,
            'identified_faces',
            self.cb_faces,
            10,
        )
        self.pub = self.create_publisher(
            FaceFocus,
            'current_face',
            10,
        )

    def cb_state(self, msg):
        self.state = msg

    def cb_faces(self, faces_msg):
        if self.state.mode != SystemState.TRACKING:
            return
        focus = FaceFocus()
        # TODO: tracking_id 와 일치하는 얼굴 추출
        self.pub.publish(focus)


def main():
    rclpy.init()
    rclpy.spin(FaceFocusingNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
