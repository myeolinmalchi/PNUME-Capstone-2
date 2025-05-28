# face_tracking_node.py
import rclpy
from rclpy.node import Node
from control_system.msg import BoundingBox, BoundingBoxes, TrackedFace, TrackedFaces
from edge.sbc.src.control_system.control_system.utils.sort import Sort

import numpy as np


class FaceTrackingNode(Node):

    def __init__(self):
        super().__init__('face_tracking_node')
        self.sub = self.create_subscription(
            BoundingBoxes,
            'bounding_boxes',
            self.identify_faces,
            10,
        )
        self.pub = self.create_publisher(
            TrackedFaces,
            'identified_faces',
            10,
        )

        self.tracker = Sort()

    def identify_faces(self, msg):
        """SORT 알고리즘을 통해, 탐지된 bbox에 ID 부여"""

        bboxes = np.array(
            [[bbox.x1, bbox.y1, bbox.x2, bbox.y2] for bbox in msg.bbs],
            dtype=np.float32,
        )

        tracked_bboxes = self.tracker.update(bboxes)

        tracked_bboxes_msg = TrackedFaces(faces=[
            TrackedFace(id=id, bbox=BoundingBox(x1=x1, y1=y1, x2=x2, y2=y2))
            for x1, y1, x2, y2, id in tracked_bboxes
        ])

        self.pub.publish(tracked_bboxes_msg)


def main():
    rclpy.init()
    rclpy.spin(FaceTrackingNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
