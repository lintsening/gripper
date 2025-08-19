# MonitorPressNode（繼承自 ServoMonitorNode）
class MonitorPressNode(ServoMonitorNode):
    def __init__(self):
        super().__init__('servo_press_monitor')

    def pulse_callback(self, channel):
        current_time = time.time() * 1000000
        level = GPIO.input(self.gpio_pin)
        if level == 1:
            self.start_time = current_time
        elif level == 0 and self.start_time != 0.0:
            pulse_us = current_time - self.start_time
            self.pulse_data['pulse_us'] = pulse_us
            if self.MIN_PULSE_US <= pulse_us <= self.MAX_PULSE_US:
                angle = (pulse_us - self.MIN_PULSE_US) / (self.MAX_PULSE_US - self.MIN_PULSE_US) * 180.0
                self.pulse_data['angle'] = max(0.0, min(180.0, angle))
            else:
                self.pulse_data['angle'] = 0.0
                self.get_logger().warn(f'腳位 {self.gpio_pin} 脈衝無效: {pulse_us:.0f} us')
            duty_cycle = (pulse_us / (1000000.0 / self.PWM_FREQUENCY)) * 100.0
            self.pulse_data['duty_cycle'] = duty_cycle
            self.get_logger().info(f'腳位 {self.gpio_pin} 偵測到脈衝: {pulse_us:.0f} us，角度: {self.pulse_data["angle"]:.2f} 度，佔空比: {duty_cycle:.2f}%')


def monitor_press_main(args=None):
    rclpy.init(args=args)
    node = MonitorPressNode()
    try:
        rclpy.spin(node)
    finally:
        node.cleanup()
        rclpy.shutdown()