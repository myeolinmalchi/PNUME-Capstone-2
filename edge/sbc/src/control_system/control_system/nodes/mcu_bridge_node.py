# mcu_bridge_node.py
import rclpy
from rclpy.node import Node
from control_system_interfaces.msg import TrackedFace, SystemState

import serial
import json

MCU_SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUDRATE = 115200


class MCUBridgeNode(Node):

    def __init__(self):
        super().__init__('mcu_bridge_node')
        self.mcu_serial = serial.Serial(MCU_SERIAL_PORT,
                                        SERIAL_BAUDRATE,
                                        timeout=1)
        self.create_subscription(TrackedFace, 'current_face',
                                 self.face_listener, 10)
        self.create_subscription(SystemState, 'system_state', self.cb_state,
                                 10)

        self.mode = SystemState.IDLE

    def face_listener(self, msg):
        if self.mode != SystemState.TRACKING:
            return

        data = {
            "mode": SystemState.TRACKING,
            "bbox": {
                "x1": msg.bbox.x1,
                "y1": msg.bbox.y1,
                "x2": msg.bbox.x2,
                "y2": msg.bbox.y2,
            },
        }

    def state_listener(self, msg):
        """SystemState 변경시 mcu에 상태값 갱신"""

        data = None

        if self.state.mode == msg.mode:

            # Tracking 모드에서 얼굴 id만 변경된 경우 early return
            # `self.face_listenter`에서 처리하면 됨
            if (msg.mode == SystemState.TRACKING) and (self.state.tracking_id
                                                       != msg.tracking_id):
                return

            # Manual 모드에서 command 변경시 mcu에 시리얼로 전달
            if (msg.mode == SystemState.MANUAL) and (self.state.command
                                                     != msg.command):
                data = {
                    "mode": SystemState.MANUAL,
                    "command": msg.command,
                }

        if self.state.mode != msg.mode:
            # Tracking 모드로 변경시 early retrun
            if msg.mode == SystemState.TRACKING:
                return

            data = {
                "mode": msg.mode,
                "command": SystemState.NONE,
            }

        if data != None:
            json_str = json.dumps(data, ensure_ascii=False)
            json_bytes = json_str.encode('utf-8')
            self.mcu_serial.write(json_bytes)


def main():
    rclpy.init()
    rclpy.spin(MCUBridgeNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
