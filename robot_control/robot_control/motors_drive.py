from tkinter import TOP
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import lgpio

L_TOP_FORWARD = 'L_TOP_FORWARD'
L_TOP_BACKWARD = 'L_TOP_BACKWARD'
L_TOP_PWM = 'L_TOP_PWM'

L_BOTTOM_FORWARD = 'L_BOTTOM_FORWARD'
L_BOTTOM_BACKWARD = 'L_BOTTOM_BACKWARD'
L_BOTTOM_PWM = 'L_BOTTOM_PWM'

R_TOP_FORWARD = 'R_TOP_FORWARD'
R_TOP_BACKWARD = 'R_TOP_BACKWARD'
R_TOP_PWM = 'R_TOP_PWM'

R_BOTTOM_FORWARD = 'R_BOTTOM_FORWARD'
R_BOTTOM_BACKWARD = 'R_BOTTOM_BACKWARD'
R_BOTTOM_PWM = 'R_BOTTOM_PWM'

# Motor pins
MOTOR_PINS = {
L_TOP_FORWARD : 21,
L_TOP_BACKWARD : 20,
L_TOP_PWM : 0,

L_BOTTOM_FORWARD : 22,
L_BOTTOM_BACKWARD : 23,
L_BOTTOM_PWM : 1,

R_TOP_FORWARD : 24,
R_TOP_BACKWARD : 25,
R_TOP_PWM : 12,

R_BOTTOM_FORWARD : 27,
R_BOTTOM_BACKWARD : 26,
R_BOTTOM_PWM : 13
}

# PWM frequency
PWM_FREQUENCY = 1000 # 1 kHz

# Power
POWER = 30 # %

# Motor drive class
class MotorsDrive(Node):
    def __init__(self):
        super().__init__('motors_drive')
        self.get_logger().info('Node initialized: motors_drive')
        self.subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        self.get_logger().info('Subscribed to topic: cmd_vel')
        self.twist = Twist()
        
        # Initialize GPIO
        self.chip_handle = lgpio.gpiochip_open(4)
        
        # Initialize motor pins
        for pin in MOTOR_PINS.values():
            lgpio.gpio_claim_output(self.chip_handle, pin)
            lgpio.gpio_write(self.chip_handle, pin, 0)
            
        # Timer for sending drive signals
        self.timer = self.create_timer(0.01, self.drive_callback)
        
        self.left_speed = 0.0
        self.right_speed = 0.0
            

    def twist_callback(self, msg):
        self.get_logger().info(f'Received Twist message: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
        
        # Calculate motor speeds
        self.left_speed = msg.linear.x - msg.angular.z
        self.right_speed = msg.linear.x + msg.angular.z
        
        # Limit motor speeds to [-1, 1]
        self.left_speed = max(-1, min(1, self.left_speed))
        self.right_speed = max(-1, min(1, self.right_speed))
        
                
    def drive_callback(self):
        # Drive motors
        self.get_logger().info(f'Driving motors: left={self.left_speed}, right={self.right_speed}')
        
        # Left motors
        if self.left_speed > 0:
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[L_TOP_FORWARD], 1)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[L_BOTTOM_FORWARD], 1)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[L_TOP_BACKWARD], 0)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[L_BOTTOM_BACKWARD], 0)
            
        else:
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[L_TOP_FORWARD], 0)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[L_BOTTOM_FORWARD], 0)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[L_TOP_BACKWARD], 1)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[L_BOTTOM_BACKWARD], 1)
        lgpio.tx_pwm(self.chip_handle, MOTOR_PINS[L_TOP_PWM], PWM_FREQUENCY, abs(self.left_speed) * POWER)
        lgpio.tx_pwm(self.chip_handle, MOTOR_PINS[L_BOTTOM_PWM], PWM_FREQUENCY, abs(self.left_speed) * POWER)
        
        # Right motors
        if self.right_speed > 0:
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[R_TOP_FORWARD], 1)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[R_BOTTOM_FORWARD], 1)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[R_TOP_BACKWARD], 0)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[R_BOTTOM_BACKWARD], 0)
        else:
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[R_TOP_FORWARD], 0)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[R_BOTTOM_FORWARD], 0)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[R_TOP_BACKWARD], 1)
            lgpio.gpio_write(self.chip_handle, MOTOR_PINS[R_BOTTOM_BACKWARD], 1)
        lgpio.tx_pwm(self.chip_handle, MOTOR_PINS[R_TOP_PWM], PWM_FREQUENCY, abs(self.right_speed) * POWER)
        lgpio.tx_pwm(self.chip_handle, MOTOR_PINS[R_BOTTOM_PWM], PWM_FREQUENCY, abs(self.right_speed) * POWER)
        
def main(args=None):
    rclpy.init(args=args)
    motors_drive = MotorsDrive()
    rclpy.spin(motors_drive)
    motors_drive.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()