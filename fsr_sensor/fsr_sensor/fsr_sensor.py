#!/usr/bin/env python3

import can
import time
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CanReceiver(Node):
    def __init__(self):
        super().__init__('can_receiver')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/heroehs/alice/force_data', 10)
        self.bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000)
        self.ids = [0x000, 0x001, 0x002]  # 하트비트 및 force 센서 ID

    def receive_can_data(self):
        while rclpy.ok():
            message = self.bus.recv()
            if message.arbitration_id in self.ids:
                if message.arbitration_id == 0x000:
                    # 하트비트 처리 (필요 시 추가 처리 가능)
                    self.get_logger().info(f"Heartbeat received: {message.data.hex()}")
                elif message.arbitration_id in [0x001, 0x002]:
                    # force 센서 데이터 처리
                    sensor_index = message.arbitration_id - 0x001
                    force_value = struct.unpack('d', message.data[:8])[0]
                    self.get_logger().info(f"Force sensor {sensor_index + 1} value: {force_value}")
                    self.publish_force_data(sensor_index, force_value)

    def publish_force_data(self, sensor_index, force_value):
        msg = Float64MultiArray()
        msg.data = [0.0] * 8  # 초기화된 배열 생성
        msg.data[sensor_index] = force_value
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    can_receiver = CanReceiver()

    try:
        can_receiver.receive_can_data()
    except KeyboardInterrupt:
        pass
    finally:
        can_receiver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

