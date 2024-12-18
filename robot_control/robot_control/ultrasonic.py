import time
import lgpio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math

# Set the GPIO pins
TRIGGER = 14
ECHO = 4

class Ultrasonic(Node):
    def __init__(self):
        super().__init__('ultrasonic')
        self.publisher = self.create_publisher(Range, 'ultrasonic', 10)
        
        # Initialize the GPIO pins
        self.gpio_handle = lgpio.gpiochip_open(4)
        # Set the GPIO pins as output/input
        lgpio.gpio_claim_output(self.gpio_handle, TRIGGER)
        lgpio.gpio_claim_input(self.gpio_handle, ECHO)
        lgpio.gpio_write(self.gpio_handle, TRIGGER, 0)
        time.sleep(0.5)
        
        # Message to publish the distance
        self.msg = Range()
        self.msg.header.frame_id = 'ultrasonic'
        self.msg.radiation_type = Range.ULTRASOUND
        self.msg.field_of_view = math.radians(30)
        self.msg.min_range = 0.02
        self.msg.max_range = 4.0
        
        self.get_logger().info("Ultrasonic node initialized.")
        
        # Create a timer to read the ultrasonic sensor
        self.timer = self.create_timer(0.05, self.timer_callback)

    # Function to read the ultrasonic sensor
    def timer_callback(self):
        self.get_logger().info("Triggering ultrasonic sensor.")
        # Send a trigger signal
        lgpio.gpio_write(self.gpio_handle, TRIGGER, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.gpio_handle, TRIGGER, 0)
        
        try:
            # Read the echo signal
            while lgpio.gpio_read(self.gpio_handle, ECHO) == 0:
                pulse_start = time.time()
            while lgpio.gpio_read(self.gpio_handle, ECHO) == 1:
                pulse_end = time.time()
            
            # Calculate the distance in meters
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 170
            
            # Log the measured distance
            self.get_logger().info(f"Measured distance: {distance:.2f} meters.")
            
            # Publish the distance
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.msg.range = distance
            self.publisher.publish(self.msg)
        except Exception as e:
            self.get_logger().error(f"Error in ultrasonic reading: {e}")
        
def main(args=None):
    rclpy.init(args=args)
    ultrasonic = Ultrasonic()
    rclpy.spin(ultrasonic)
    ultrasonic.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
