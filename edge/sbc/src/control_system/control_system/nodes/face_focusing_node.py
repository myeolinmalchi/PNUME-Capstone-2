# face_focusing_node.py
import rclpy
from rclpy.node import Node
from control_system_interfaces.msg import TrackedFaces, TrackedFace, SystemState


class FaceFocusingNode(Node):

    def __init__(self):
        super().__init__('face_focusing_node')
        self.state = SystemState()

        self.create_subscription(
            SystemState,
            'system_state',
            self.system_listener,
            10,
        )
        self.create_subscription(
            TrackedFaces,
            'identified_faces',
            self.faces_listener,
            10,
        )
        self.pub = self.create_publisher(
            TrackedFace,
            'current_face',
            10,
        )

        self.faces = []
        self.id = None

    def focus(self):
        for face in self.faces:
            if face.id == self.tracking_id:
                return face

        return None

    def system_listener(self, msg):
        self.tracking_id = msg.tracking_id
        self.pub.publish(self.focus())

    def faces_listener(self, msg):
        self.faces = msg.faces
        self.pub.publish(self.focus())


def main():
    rclpy.init()
    rclpy.spin(FaceFocusingNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
