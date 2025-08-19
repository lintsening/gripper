#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

class ServoMonitorNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # PWM 參數
        self.PWM_FREQUENCY = 50
        self.declare_parameter('min_pulse_us', 500)
        self.declare_parameter('max_pulse_us', 2500)
        self.declare_parameter('gpio_pin', 0)  # 監測腳位（讀取）
        self.declare_parameter('topic_name', '')  # 發布實際角度的 topic（使用 {原topic}/actual）
        self.MIN_PULSE_US = self.get_parameter('min_pulse_us').value
        self.MAX_PULSE_US = self.get_parameter('max_pulse_us').value
        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.topic_name = self.get_parameter('topic_name').value

        # 驗證參數
        if self.gpio_pin == 0 or not self.topic_name:
            raise ValueError('必須提供有效的 gpio_pin 和 topic_name')

        # 初始化 GPIO
        try:
            GPIO.setmode(GPIO.BCM)
            self.pulse_data = {'pulse_us': 0.0, 'angle': 0.0, 'duty_cycle': 0.0}
            self.start_time = 0.0
            GPIO.setup(self.gpio_pin, GPIO.IN)

            # 創建發布者（發布到 topic_name，通常為 {原topic}/actual）
            self.publisher = self.create_publisher(Float32, self.topic_name, 10)

            # 設置邊沿偵測
            GPIO.add_event_detect(self.gpio_pin, GPIO.BOTH, callback=self.pulse_callback)
            self.get_logger().info(f'監測腳位 {self.gpio_pin} 已設置，對應發布 topic {self.topic_name}，脈衝範圍 {self.MIN_PULSE_US}-{self.MAX_PULSE_US} us')

            # 定時發布
            self.timer = self.create_timer(0.1, self.publish_servo_data)
            self.get_logger().info(f'{node_name} 已啟動')
        except Exception as e:
            self.get_logger().error(f'GPIO 初始化失敗: {str(e)}')
            raise

    @abstractmethod
    def pulse_callback(self, channel):
        """抽象方法：處理脈衝偵測（子類別可自訂）"""
        pass

    def publish_servo_data(self):
        msg = Float32()
        msg.data = self.pulse_data['angle']
        self.publisher.publish(msg)
        self.get_logger().debug(f'腳位 {self.gpio_pin} 發布角度: {msg.data:.2f} 度')

    def cleanup(self):
        try:
            GPIO.cleanup()
            self.get_logger().info('GPIO 清理完成')
        except Exception as e:
            self.get_logger().error(f'GPIO 清理失敗: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoMonitorNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'節點啟動失敗: {str(e)}')
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()