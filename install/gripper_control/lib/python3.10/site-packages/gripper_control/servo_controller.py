import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import re

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # 取得參數
        self.declare_parameter('gpio_pin', 0)
        self.declare_parameter('topic_name', None)

        try:
            self.gpio_pin = self.get_parameter('gpio_pin').get_parameter_value().integer_value
            self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        except Exception as e:
            self.get_logger().error(f'參數取得失敗: {str(e)}')
            rclpy.shutdown()
            return

        # 驗證 GPIO pin
        if not (2 <= self.gpio_pin <= 27):
            self.get_logger().error(f'GPIO pin {self.gpio_pin} 不在有效範圍 2~27 之內。')
            rclpy.shutdown()
            return

        # 驗證 topic_name 格式
        if not re.match(r'^\/[a-zA-Z0-9_\/]*$', self.topic_name):
            self.get_logger().error(f'Topic 名稱 "{self.topic_name}" 格式錯誤，請使用以 "/" 開頭、不含空白的名稱。')
            rclpy.shutdown()
            return

        # 初始化 GPIO
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_pin, GPIO.OUT)
            self.pwm = GPIO.PWM(self.gpio_pin, 50)  # 50Hz
            self.pwm.start(7.5)

            self.current_angle = 90.0
            self.current_duty = 7.5
            self.get_logger().info(f'GPIO {self.gpio_pin} 初始化成功，初始 PWM: 7.5%')

        except Exception as e:
            self.get_logger().error(f'GPIO 初始化失敗: {str(e)}')
            rclpy.shutdown()
            return

        # 訂閱角度指令
        self.subscription = self.create_subscription(
            Float32,
            self.topic_name,
            self.angle_callback,
            10
        )

        # 狀態回報 Timer（1 Hz）
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f'伺服馬達控制啟動於 GPIO {self.gpio_pin}, 訂閱 topic {self.topic_name}')

    def angle_callback(self, msg):
        angle = max(0.0, min(180.0, msg.data))
        duty = 2.0 + (angle / 180.0) * 12.0  # 0° -> 2.5%，180° -> 12.5%
        
        try:
            self.pwm.ChangeDutyCycle(duty)
            self.current_angle = angle
            self.current_duty = duty
            self.get_logger().info(f'收到角度: {angle:.2f}°, PWM: {duty:.2f}%')

        except Exception as e:
            self.get_logger().error(f'PWM 應用失敗: {str(e)}')

    def publish_status(self):
        self.get_logger().info(f'[狀態回報] 目前角度: {self.current_angle:.2f}°, PWM: {self.current_duty:.2f}%')

    def destroy_node(self):
        self.get_logger().info('關閉節點，釋放 GPIO 資源')
        try: 
            if hasattr(self, 'pwm'):
                self.pwm.ChangeDutyCycle(0)
                self.pwm.stop()
                GPIO.cleanup(self.gpio_pin)
        except Exception as e:
            self.get_logger().error(f'GPIO 清理失敗: {str(e)}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ServoController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()