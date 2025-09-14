#!/usr/bin/env python3
'''
這份程式碼是設計用於控制伺服馬達，發送topic做出角度控制的指令，ServoControllerNode作為抽象父類別給同資料夾中的press 和 rotate繼承
init 參數、GPIO初始化，以及清理資訊cleanup
另有抽象方法angle_callback
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time
from abc import ABC, abstractmethod

# 抽象 ServoControllerNode 類別（繼承自 rclpy.node.Node）
@abstractclass
class ServoControllerNode(Node, ABC):
    def __init__(self, node_name):
        super().__init__(node_name)

        # PWM 參數
        self.PWM_FREQUENCY = 50
        self.declare_parameter('min_pulse_us', 500)
        self.declare_parameter('max_pulse_us', 2500)
        self.declare_parameter('gpio_pin', 0)  # 輸出腳位
        self.declare_parameter('topic_name', '')  # 目標角度 topic
        self.MIN_PULSE_US = self.get_parameter('min_pulse_us').value
        self.MAX_PULSE_US = self.get_parameter('max_pulse_us').value
        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.topic_name = self.get_parameter('topic_name').value

        # 平滑參數
        self.SMOOTHING_FACTOR = 0.2
        self.MAX_ANGLE_STEP = 10.0

        # 儲存當前角度與實際角度
        self.current_angle = 0.0
        self.actual_angle = 0.0

        # 驗證參數
        if self.gpio_pin == 0 or not self.topic_name:
            raise ValueError('必須提供有效的 gpio_pin 和 topic_name')

        # 初始化 GPIO
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_pin, GPIO.OUT)
            self.pwm = GPIO.PWM(self.gpio_pin, self.PWM_FREQUENCY)
            self.pwm.start(0)
            self.get_logger().info(f'腳位 {self.gpio_pin} 已 attach，對應 topic {self.topic_name}，脈衝範圍 {self.MIN_PULSE_US}-{self.MAX_PULSE_US} us')

            # 訂閱目標角度
            self.subscription = self.create_subscription(
                Float32,
                self.topic_name,
                self.angle_callback,
                10
            )

            # 訂閱監測角度（使用 {topic_name}/actual）
            monitor_topic = f"{self.topic_name}/actual"
            self.monitor_subscription = self.create_subscription(
                Float32,
                monitor_topic,
                self.monitor_callback,
                10
            )
            self.get_logger().info(f'訂閱監測 topic {monitor_topic} 用於腳位 {self.gpio_pin}')

            self.get_logger().info(f'{node_name} 已啟動')
        except Exception as e:
            self.get_logger().error(f'GPIO 初始化失敗: {str(e)}')
            raise

    @abstractmethod
    def angle_callback(self, msg):
        """抽象方法：處理目標角度（子類別可自訂）"""
        pass

    def monitor_callback(self, msg):
        """接收監測節點回傳的實際角度，並與當前目標比較"""
        self.actual_angle = msg.data
        error = abs(self.actual_angle - self.current_angle)
        self.get_logger().info(f'腳位 {self.gpio_pin} 實際角度: {self.actual_angle:.2f} 度，目標: {self.current_angle:.2f} 度，誤差: {error:.2f} 度')
        if error > 5.0:
            self.get_logger().warn(f'腳位 {self.gpio_pin} 角度誤差過大，考慮調整')

    def cleanup(self):
        try:
            self.pwm.stop()
            GPIO.cleanup()
            self.get_logger().info('GPIO 清理完成')
        except Exception as e:
            self.get_logger().error(f'GPIO 清理失敗: {str(e)}')