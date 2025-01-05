import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import lgpio
import time

# GPIO pins
SERVO_Y = 6
SERVO_Z = 7

# Maximum angle
MAX_Y_ANGLE = 80
MIN_Y_ANGLE = -80
MIN_Z_ANGLE = -80
MAX_Z_ANGLE = 80

# PWM settings
PWM_FREQUENCY = 50  # 50 Hz

# Camera servo drive class
class CameraServoDrive(Node):
    def __init__(self):
        super().__init__('camera_servo_drive')
        self.get_logger().info('Node initialized: camera_servo_drive')
        self.subscriber = self.create_subscription(
            Twist,
            'camera_twist',
            self.twist_callback,
            10
        )
        self.get_logger().info('Subscribed to topic: camera_twist')
        
        self.twist = Twist()

        # Initialize GPIO
        self.chip_handle = lgpio.gpiochip_open(4)
        self.get_logger().info('GPIO chip opened')

        # Initialize pins
        lgpio.gpio_claim_output(self.chip_handle, SERVO_Y)
        lgpio.gpio_write(self.chip_handle, SERVO_Y, 0)
        lgpio.gpio_claim_output(self.chip_handle, SERVO_Z)
        lgpio.gpio_write(self.chip_handle, SERVO_Z, 0)
        self.get_logger().info('GPIO pins initialized for servos')

        self.current_angles = [0, 0] # y, z
            
    def twist_callback(self, msg):
        self.twist = msg

    def set_servo_angle(self, servo, angle):
        # Calculate duty cycle: Map angle to pulse width (500-2480 microseconds)
        pulsewidth = ((int(angle) + 90) * 11) + 500  # Scale angle to microseconds
        duty_cycle = pulsewidth / 20000  # Convert to fraction of 20ms (duty cycle)

        # Set PWM using lgpio
        lgpio.tx_pwm(self.chip_handle, servo, PWM_FREQUENCY, duty_cycle * 100, pulse_cycles=1)  # Duty cycle as a percentage
        time.sleep(0.02)

def main(args=None):
    rclpy.init(args=args)
    camera_servo_drive = CameraServoDrive()
    while rclpy.ok():
        rclpy.spin_once(camera_servo_drive, timeout_sec=0.0)
        if camera_servo_drive.twist.angular.y != 0.0:
            camera_servo_drive.current_angles[0] += int(camera_servo_drive.twist.angular.y)
            camera_servo_drive.current_angles[0] = min(MAX_Y_ANGLE, max(MIN_Y_ANGLE, camera_servo_drive.current_angles[0]))
            camera_servo_drive.set_servo_angle(SERVO_Y, camera_servo_drive.current_angles[0])
        elif camera_servo_drive.twist.angular.z != 0.0:
            camera_servo_drive.current_angles[1] += int(camera_servo_drive.twist.angular.z)
            camera_servo_drive.current_angles[1] = min(MAX_Z_ANGLE, max(MIN_Z_ANGLE, camera_servo_drive.current_angles[1]))
            camera_servo_drive.set_servo_angle(SERVO_Z, camera_servo_drive.current_angles[1])
    camera_servo_drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
