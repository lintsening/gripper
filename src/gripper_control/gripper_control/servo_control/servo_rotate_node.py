# ServoRotateNode（繼承自 ServoControllerNode）
class ServoRotateNode(ServoControllerNode):
    def __init__(self):
        super().__init__('servo_rotate_controller')

    def angle_callback(self, msg):
        target_angle = max(0.0, min(180.0, msg.data))
        angle_diff = target_angle - self.current_angle
        if abs(angle_diff) > self.MAX_ANGLE_STEP:
            target_angle = self.current_angle + self.MAX_ANGLE_STEP * (1 if angle_diff > 0 else -1)
        smoothed_angle = self.current_angle + self.SMOOTHING_FACTOR * (target_angle - self.current_angle)
        self.current_angle = smoothed_angle
        pulse_us = self.MIN_PULSE_US + (smoothed_angle / 180.0) * (self.MAX_PULSE_US - self.MIN_PULSE_US)
        duty_cycle = (pulse_us / (1000000.0 / self.PWM_FREQUENCY)) * 100.0
        self.get_logger().info(f'腳位 {self.gpio_pin} 接收到目標角度: {msg.data:.2f} 度，平滑後角度: {smoothed_angle:.2f} 度，脈衝: {pulse_us:.0f} us，duty cycle: {duty_cycle:.2f}%')
        try:
            self.pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.02)
        except Exception as e:
            self.get_logger().error(f'腳位 {self.gpio_pin} 設定 PWM 失敗: {str(e)}')

def servo_rotate_main(args=None):
    rclpy.init(args=args)
    node = ServoRotateNode()
    try:
        rclpy.spin(node)
    finally:
        node.cleanup()
        rclpy.shutdown()