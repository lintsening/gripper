#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

# MG995 伺服馬達參數
SERVO_PIN = 18  # GPIO 18 (BCM 編號)
PWM_FREQUENCY = 50  # 50Hz
MIN_DUTY_CYCLE = 2.5   # 0 度（500μs）
MAX_DUTY_CYCLE = 12.5  # 180 度（2500μs）

class ServoControlNode(Node):
    def __init__(self):
        super().__init__(node_name='servo_control_node')

        # 初始化 GPIO
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(SERVO_PIN, GPIO.OUT)
            self.pwm = GPIO.PWM(SERVO_PIN, PWM_FREQUENCY)
            self.pwm.start(0)
            self.get_logger().info('PWM 初始化成功')
        except Exception as e:
            self.get_logger().error(f'GPIO 初始化失敗: {str(e)}')
            raise

        # 建立 ROS 2 訂閱者
        self.subscription = self.create_subscription(
            Float32,
            '/servo_angle',
            self.angle_callback,
            10
        )

        self.get_logger().info('伺服馬達控制節點已啟動，等待 /servo_angle 指令...')

    def angle_callback(self, msg):
        angle = max(0.0, min(180.0, msg.data))  # 限制在 0~180 度
        duty_cycle = MIN_DUTY_CYCLE + (angle / 180.0) * (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE)
        self.get_logger().info(f'接收到角度: {angle:.2f} 度 -> PWM duty cycle: {duty_cycle:.2f}%')

        try:
            self.pwm.ChangeDutyCycle(duty_cycle)
        except Exception as e:
            self.get_logger().error(f'設定 PWM 失敗: {str(e)}')

    def cleanup(self):
        try:
            if hasattr(self, 'pwm'):
                self.pwm.stop()
            GPIO.cleanup()
            self.get_logger().info('GPIO 清理完成')
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