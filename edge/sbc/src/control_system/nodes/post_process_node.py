import rclpy
from rclpy.node import Node

from control_system.msg import (
    YoloRawDetections,
    BoundingBox,
    BoundingBoxes,
)

import numpy as np

from control_system.utils.yolo import yolo11


class PostProcessNode(Node):

    def __init__(self):

        super().__init__('post_process_node')
        self.pub = self.create_publisher(BoundingBoxes, 'bounding_boxes', 10)
        self.sub = self.create_subscription(YoloRawDetections,
                                            'yolo_raw_detections', 10)

    def tensor_msg_to_numpy(self, tensor):
        shape = tuple(tensor.dims)
        data = np.array(tensor.data, dtype=np.float32)
        return data.reshape(shape)

    def box_to_msg(self, box, score):
        x1, y1, w, h = box.astype(int)
        x2, y2 = x1 + w, y1 + h

        return BoundingBox(x1=x1, y1=y1, x2=x2, y2=y2, score=score)

    def postprocess_callback(self, msg):

        input_data = [self.tensor_msg_to_numpy(t) for t in msg.tensors]
        boxes, _, scores = yolo11.post_process(input_data)

        bbox_msgs: list[BoundingBox] = [
            self.box_to_msg(box, score) for box, score in zip(boxes, scores)
        ]

        bboxes_msg = BoundingBoxes(bboxes=bbox_msgs)
        self.pub.publish(bboxes_msg)


def main():
    rclpy.init()
    rclpy.spin(PostProcessNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
