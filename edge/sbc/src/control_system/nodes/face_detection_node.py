# face_detection_node.py
import rclpy
from rclpy.node import Node
from control_system.msg import (
    BoundingBoxes,
    YoloTensor,
    YoloRawDetections,
    SystemState,
)

from control_system.utils.yolo.wrappers import RKNNWrapper
from control_system.utils.camera import setup_camera

import cv2
import numpy as np

from time import time


class FaceDetectionNode(Node):
    """YOLO11을 통해 얼굴 위치를 추론하는 노드
    
    Raw Detection 결과만 반환하며, 후처리는 `PostProcessNode`에서 전담
    """

    def __init__(
        self,
        model_path: str,
        cam_width: int,
        cam_height: int,
    ):
        super().__init__('face_detection_node')
        self.pub = self.create_publisher(YoloRawDetections,
                                         'yolo_raw_detections', 10)

        self.sub = self.create_subscription(
            SystemState,
            'system_state',
            self.system_state_callback,
            10,
        )

        self.system_state = None
        self.rknn = RKNNWrapper(model_path=model_path)
        self.img_size = (cam_width, cam_height)
        self.capture = setup_camera(cam_width, cam_height)

    def system_state_callback(self, msg):
        self.system_state = msg

    def publish_raw(self, outputs):
        msg = YoloRawDetections()
        for out in outputs:
            yt = YoloTensor()
            yt.dims = list(out.shape)
            yt.data = out.astype(np.float32).flatten().tolist()
            msg.tensors.append(yt)

        self.pub.publish(msg)

    def run(self):
        """추론을 위한 메인 루프 실행"""

        while self.capture.isOpened() and rclpy.ok():

            # TRACKING 모드가 아닌 경우 Detection 중지
            if self.state.mode in {SystemState.IDLE, SystemState.MANUAL}:
                time.sleep(0.05)
                continue

            ret, frame = self.capture.read()
            if not ret:
                break

            img = cv2.resize(
                frame,
                self.img_size,
                interpolation=cv2.INTER_AREA,
            )

            outputs = self.rknn.run(img)
            self.publish_raw(outputs)


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionNode()
    node.run()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
