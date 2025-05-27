# desktop_bridge_node.py
import rclpy
from rclpy.node import Node
from control_system.msg import SystemState, TrackedFaces

import serial
import json

DESKTOP_SERIAL_PORT = '/dev/ttyGS0'
DESKTOP_BUAD_RATE = 115200


class DesktopBridgeNode(Node):
    """데스크탑과 통신하며 System State 설정하는 노드"""

    def __init__(self):
        super().__init__('desktop_bridge_node')

        self.state = None

        self.desktop_serial = serial.Serial(DESKTOP_SERIAL_PORT,
                                            DESKTOP_BUAD_RATE,
                                            timeout=1)
        self.pub = self.create_publisher(SystemState, 'system_state', 10)
        self.create_subscription(TrackedFaces, 'identified_faces',
                                 self.faces_listener, 10)

    def faces_listener(self, msg):

        faces = []
        for face in msg.faces:
            faces.append({
                "bbox": {
                    "x1": face.bbox.x1,
                    "y1": face.bbox.y1,
                    "x2": face.bbox.x2,
                    "y2": face.bbox.y2
                }
            })

        data = {
            "mode": self.state["mode"],
            "trackingState": {
                **self.state["trackingState"],
                "faces": faces,
            },
            "manualState": {
                **self.state["manualState"]
            }
        }

        json_str = json.dumps(data, ensure_ascii=False)
        json_bytes = json_str.encode('utf-8')
        self.desktop_serial.write(json_bytes)

    def run(self):
        """데스크탑 명령 수신을 위한 메인 루프 실행"""

        while True:
            if self.desktop_serial.in_waiting:
                line: str = self.desktop_serial.readline().decode().strip()
                self.get_logger().info(f'[RX] Received from desktop: {line}')

                stateDict = json.loads(line)
                self.state = stateDict

                stateMsg = SystemState(
                    mode=stateDict["mode"],
                    command=stateDict["manualState"]["currentCommand"],
                    tracking_id=stateDict["trackingState"]["trackingId"],
                )
                self.pub.publish(stateMsg)

            rclpy.spin_once(self, timeout_sec=0.01)


def main():
    rclpy.init()
    rclpy.spin(DesktopBridgeNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
