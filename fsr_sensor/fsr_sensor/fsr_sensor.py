#!/usr/bin/env python3

import can
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CanReceiver(Node):
    def __init__(self):
        super().__init__('can_receiver')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/heroehs/alice/force_data', 10)
        self.bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000)
        self.high_byte = [0] * 8  # 상위 8비트 저장 배열
        self.low_byte = [0] * 8  # 하위 8비트 저장 배열
        self.force_data = [0.0] * 8  # 초기화된 force 센서 데이터 배열

    def receive_can_data(self):
        while rclpy.ok():
            message = self.bus.recv()
            if 0x001 <= message.arbitration_id <= 0x010:
                self.process_sensor_data(message)

    def process_sensor_data(self, message):
        sensor_index = (message.arbitration_id - 0x001) // 2
        if message.arbitration_id % 2 == 1:
            # 상위 8비트
            self.high_byte[sensor_index] = message.data[0]
        else:
            # 하위 8비트
            self.low_byte[sensor_index] = message.data[0]
            # 상위 8비트와 하위 8비트를 결합하여 16비트 값 생성
            combined_value = (self.high_byte[sensor_index] << 8) | self.low_byte[sensor_index]
            self.force_data[sensor_index] = float(combined_value)
            self.publish_force_data()

    def publish_force_data(self):
        msg = Float64MultiArray()
        msg.data = self.force_data
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
