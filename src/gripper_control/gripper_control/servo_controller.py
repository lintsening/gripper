#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

class ServoControlNode(Node):
    def __init__(self):
        super().__init__(node_name='servo_control_node')

        # PWM 參數（參考 Arduino Servo 庫）
        self.PWM_FREQUENCY = 50  # 50Hz，週期 20ms (20000us)
        self.declare_parameter('min_pulse_us', 500)  # 預設 MG995 的 0 度 (500us)
        self.declare_parameter('max_pulse_us', 2500) # 預設 MG995 的 180 度 (2500us)
        self.MIN_PULSE_US = self.get_parameter('min_pulse_us').value
        self.MAX_PULSE_US = self.get_parameter('max_pulse_us').value

        # 平滑參數（解決振動）
        self.SMOOTHING_FACTOR = 0.2  # 平滑因子（0 到 1，越小越平滑）
        self.MAX_ANGLE_STEP = 10.0  # 每次角度變化的最大步進（度）

        # 儲存當前角度
        self.current_angles = {}

        # 取得 servo 配置參數
        self.declare_parameter('servo_configs', [])
        servo_configs = self.get_parameter('servo_configs').get_parameter_value().string_array_value

        # 初始化 GPIO
        try:
            GPIO.setmode(GPIO.BCM)
            self.pwms = {}
            self.subscriptions = []

            # 為每個伺服馬達設置（模擬 Arduino attach）
            for config in servo_configs:
                try:
                    pin, topic = config.split(':')
                    pin = int(pin)
                    GPIO.setup(pin, GPIO.OUT)
                    pwm = GPIO.PWM(pin, self.PWM_FREQUENCY)
                    pwm.start(0)  # 啟動 PWM，但 duty=0（類似初始 attach）
                    self.pwms[pin] = pwm
                    self.current_angles[pin] = 0.0  # 初始化角度
                    self.get_logger().info(f'腳位 {pin} 已 attach，對應 topic {topic}，脈衝範圍 {self.MIN_PULSE_US}-{self.MAX_PULSE_US} us')

                    # 建立訂閱者
                    subscription = self.create_subscription(
                        Float32,
                        topic,
                        lambda msg, p=pin: self.angle_callback(msg, p),
                        10
                    )
                    self.subscriptions.append(subscription)
                except ValueError as e:
                    self.get_logger().error(f'無效的 servo 配置 {config}: {str(e)}')
                    continue

            if not self.pwms:
                raise ValueError('沒有有效的 servo 配置')
        except Exception as e:
            self.get_logger().error(f'GPIO 初始化失敗: {str(e)}')
            raise

        self.get_logger().info('伺服馬達控制節點已啟動')

    def angle_callback(self, msg, pin):
        target_angle = max(0.0, min(180.0, msg.data))  # 限制在 0~180 度

        # 平滑處理：限制角度變化速率（解決振動）
        current_angle = self.current_angles.get(pin, 0.0)
        angle_diff = target_angle - current_angle
        if abs(angle_diff) > self.MAX_ANGLE_STEP:
            target_angle = current_angle + self.MAX_ANGLE_STEP * (1 if angle_diff > 0 else -1)

        # 應用低通濾波
        smoothed_angle = current_angle + self.SMOOTHING_FACTOR * (target_angle - current_angle)
        self.current_angles[pin] = smoothed_angle

        # 計算脈衝寬度（us），參考 Arduino write()
        pulse_us = self.MIN_PULSE_US + (smoothed_angle / 180.0) * (self.MAX_PULSE_US - self.MIN_PULSE_US)

        # 計算 duty cycle %（RPi.GPIO 需要 %）
        duty_cycle = (pulse_us / (1000000.0 / self.PWM_FREQUENCY)) * 100.0
        self.get_logger().info(f'腳位 {pin} 接收到目標角度: {msg.data:.2f} 度，平滑後角度: {smoothed_angle:.2f} 度，脈衝: {pulse_us:.0f} us，duty cycle: {duty_cycle:.2f}%')

        try:
            self.pwms[pin].ChangeDutyCycle(duty_cycle)
            time.sleep(0.02)  # 20ms 延遲，匹配 PWM 週期以穩定訊號
        except Exception as e:
            self.get_logger().error(f'腳位 {pin} 設定 PWM 失敗: {str(e)}')

    def cleanup(self):
        try:
            for pin, pwm in self.pwms.items():
                pwm.stop()  # 停止 PWM（類似 Arduino detach）
            GPIO.cleanup()
            self.get_logger().info('GPIO 清理完成（所有伺服已 detach）')
        except Exception as e:
            self.get_logger().error(f'GPIO 清理失敗: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoControlNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'節點啟動失敗: {str(e)}')
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()
        GPIO.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()