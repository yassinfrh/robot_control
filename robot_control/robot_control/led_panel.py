import time
import lgpio
import rclpy
from rclpy.node import Node
from robot_control_interfaces.msg import LedPanel
import threading

# Define font data for each character with its corresponding width
FONT_DATA = {
    'A': ([0x3f, 0x48, 0x48, 0x48, 0x3f], 5),  # 5x8 matrix
    'B': ([0x7F, 0x49, 0x49, 0x49, 0x36], 5),  # 5x8 matrix
    'C': ([0x7E, 0x41, 0x41, 0x41, 0x22], 5),  # 5x8 matrix
    'D': ([0x7F, 0x41, 0x41, 0x41, 0x3E], 5),  # 5x8 matrix
    'E': ([0x7F, 0x49, 0x49, 0x49, 0x41], 5),  # 5x8 matrix
    'F': ([0x7f, 0x48, 0x48, 0x48, 0x40], 5),  # 5x8 matrix
    'G': ([0x7E, 0x41, 0x49, 0x49, 0x2E], 5),  # 5x8 matrix
    'H': ([0x7F, 0x08, 0x08, 0x08, 0x7F], 5),  # 5x8 matrix
    'I': ([0x41, 0x7F, 0x41], 3),  # 3x8 matrix
    'J': ([0x02, 0x01, 0x41, 0x41, 0x7e], 5),  # 5x8 matrix
    'K': ([0x7F, 0x08, 0x14, 0x22, 0x41], 5),  # 5x8 matrix
    'L': ([0x7f, 0x01, 0x01, 0x01, 0x01], 5),  # 5x8 matrix
    'M': ([0x7F, 0x20, 0x10, 0x20, 0x7F], 5),  # 5x8 matrix
    'N': ([0x7f, 0x10, 0x08, 0x04, 0x7f], 5),  # 5x8 matrix
    'O': ([0x3e, 0x41, 0x41, 0x41, 0x3e], 5),  # 5x8 matrix
    'P': ([0x7f, 0x48, 0x48, 0x48, 0x30], 5),  # 5x8 matrix
    'Q': ([0x3e, 0x41, 0x41, 0x42, 0x3d], 5),  # 5x8 matrix
    'R': ([0x7f, 0x48, 0x4c, 0x4a, 0x31], 5),  # 5x8 matrix
    'S': ([0x79, 0x49, 0x49, 0x49, 0x4f], 5),  # 5x8 matrix
    'T': ([0x40, 0x40, 0x7f, 0x40, 0x40], 5),  # 5x8 matrix
    'U': ([0x7e, 0x01, 0x01, 0x01, 0x7e], 5),  # 5x8 matrix
    'V': ([0x7c, 0x02, 0x01, 0x02, 0x7c], 5),  # 5x8 matrix
    'W': ([0x7f, 0x02, 0x1c, 0x02, 0x7f], 5),  # 5x8 matrix
    'X': ([0x63, 0x14, 0x08, 0x14, 0x63], 5),  # 5x8 matrix
    'Y': ([0x60, 0x10, 0x0f, 0x10, 0x60], 5),  # 5x8 matrix
    'Z': ([0x43, 0x45, 0x49, 0x51, 0x61], 5),  # 5x8 matrix
    
    'a': ([0x02, 0x15, 0x15, 0x15, 0x0f], 5),  # 5x8 matrix
    'b': ([0x7f, 0x11, 0x11, 0x11, 0x0e], 5),  # 5x8 matrix
    'c': ([0x0e, 0x11, 0x11, 0x11, 0x11], 5),  # 5x8 matrix
    'd': ([0x0e, 0x11, 0x11, 0x11, 0x7f], 5),  # 5x8 matrix
    'e': ([0x0e, 0x15, 0x15, 0x15, 0x0c], 5),  # 5x8 matrix
    'f': ([0x3f, 0x50, 0x40], 3),  # 3x8 matrix
    'g': ([0x18, 0x25, 0x25, 0x25, 0x1e], 5),  # 5x8 matrix
    'h': ([0x7f, 0x10, 0x10, 0x10, 0x0f], 5),  # 5x8 matrix
    'i': ([0x5f], 1),  # 1x8 matrix
    'j': ([0x01, 0x01, 0x5e], 3),  # 3x8 matrix
    'k': ([0x7f, 0x04, 0x04, 0x0a, 0x11], 5),  # 5x8 matrix
    'l': ([0x7e, 0x01], 2),  # 2x8 matrix
    'm': ([0x0f, 0x10, 0x0f, 0x10, 0x0f], 5),  # 5x8 matrix
    'n': ([0x0f, 0x10, 0x10, 0x10, 0x0f], 5),  # 5x8 matrix
    'o': ([0x0e, 0x11, 0x11, 0x11, 0x0e], 5),  # 5x8 matrix
    'p': ([0x3f, 0x24, 0x24, 0x24, 0x18], 5),  # 5x8 matrix
    'q': ([0x18, 0x24, 0x24, 0x24, 0x3f], 5),  # 5x8 matrix
    'r': ([0x0f, 0x10, 0x10, 0x10, 0x10], 5),  # 5x8 matrix
    's': ([0x09, 0x15, 0x15, 0x15, 0x12], 5),  # 5x8 matrix
    't': ([0x10, 0x7e, 0x11, 0x01], 4),  # 4x8 matrix
    'u': ([0x1e, 0x01, 0x01, 0x01, 0x1e], 5),  # 5x8 matrix
    'v': ([0x18, 0x06, 0x01, 0x06, 0x18], 5),  # 5x8 matrix
    'w': ([0x1e, 0x01, 0x1e, 0x01, 0x1e], 5),  # 5x8 matrix
    'x': ([0x11, 0x0a, 0x04, 0x0a, 0x11], 5),  # 5x8 matrix
    'y': ([0x38, 0x05, 0x05, 0x05, 0x3e], 5),  # 5x8 matrix
    'z': ([0x11, 0x13, 0x15, 0x19, 0x11], 5),  # 5x8 matrix
    
    '0': ([0x3e, 0x45, 0x49, 0x51, 0x3e], 5),  # 5x8 matrix
    '1': ([0x20, 0x7f], 2),  # 2x8 matrix
    '2': ([0x41, 0x43, 0x45, 0x49, 0x31], 5),  # 5x8 matrix
    '3': ([0x22, 0x41, 0x49, 0x49, 0x36], 5),  # 5x8 matrix
    '4': ([0x78, 0x08, 0x08, 0x08, 0x7f], 5),  # 5x8 matrix
    '5': ([0x79, 0x49, 0x49, 0x49, 0x46], 5),  # 5x8 matrix
    '6': ([0x3e, 0x49, 0x49, 0x49, 0x06], 5),  # 5x8 matrix
    '7': ([0x40, 0x40, 0x43, 0x4c, 0x70], 5),  # 5x8 matrix
    '8': ([0x36, 0x49, 0x49, 0x49, 0x36], 5),  # 5x8 matrix
    '9': ([0x30, 0x49, 0x49, 0x49, 0x3e], 5),  # 5x8 matrix
    
    ' ': ([0x00, 0x00, 0x00], 3),  # Space (width: 3)
    '.': ([0x01], 1),  # Period (width: 1)
    ',': ([0x01, 0x02], 2),  # Comma (width: 2)
    '!': ([0x7D], 1),  # Exclamation mark (width: 1)
    '?': ([0x20, 0x40, 0x45, 0x48, 0x30], 5),  # Question mark (width: 5)
}

# GPIO pins
SCLK = 4
DIO = 14

class LedPanelNode(Node):
    def __init__(self):
        super().__init__('led_panel_node')
        
        self.get_logger().info("Initializing LED Panel Node")
        
        # Initialize the GPIO chip
        self.gpio_handle = lgpio.gpiochip_open(4)
        self.get_logger().info("GPIO chip opened successfully")
        
        # Initialize the GPIO pins
        lgpio.gpio_claim_output(self.gpio_handle, SCLK)
        lgpio.gpio_claim_output(self.gpio_handle, DIO)
        self.get_logger().info("GPIO pins initialized as outputs")
        
        self.subscriber = self.create_subscription(LedPanel, 'led_panel', self.led_panel_callback, 10)
        self.get_logger().info("Subscriber created for 'led_panel' topic")

        # Shared variables for display
        self.text = ''
        self.scrolling_delay = 0.0
        self.start_delay = 0.0
        self.end_delay = 0.0

        # Threading setup
        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.daemon = True
        self.lock = threading.Lock()
        self.interrupt_display = False
        self.running = True

        self.display_thread.start()
        self.get_logger().info("Display thread started")

    def led_panel_callback(self, msg):
        self.get_logger().info(f"Received new message: text='{msg.text}', scrolling_delay={msg.scrolling_delay}, start_delay={msg.start_delay}, end_delay={msg.end_delay}")
        with self.lock:
            self.text = msg.text
            self.scrolling_delay = msg.scrolling_delay
            self.start_delay = msg.start_delay
            self.end_delay = msg.end_delay
            self.interrupt_display = True  # Interrupt current display

    def nop(self):
        time.sleep(0.00003)

    def start(self):
        lgpio.gpio_write(self.gpio_handle, SCLK, 0)
        self.nop()
        lgpio.gpio_write(self.gpio_handle, SCLK, 1)
        self.nop()
        lgpio.gpio_write(self.gpio_handle, DIO, 1)
        self.nop()
        lgpio.gpio_write(self.gpio_handle, DIO, 0)
        self.nop()

    def matrix_clear(self):
        lgpio.gpio_write(self.gpio_handle, SCLK, 0)
        self.nop()
        lgpio.gpio_write(self.gpio_handle, DIO, 0)
        self.nop()
        lgpio.gpio_write(self.gpio_handle, DIO, 0)
        self.nop()

    def send_date(self, date):
        for i in range(0, 8):
            lgpio.gpio_write(self.gpio_handle, SCLK, 0)
            self.nop()
            if date & 0x01:
                lgpio.gpio_write(self.gpio_handle, DIO, 1)
            else:
                lgpio.gpio_write(self.gpio_handle, DIO, 0)
            self.nop()
            lgpio.gpio_write(self.gpio_handle, SCLK, 1)
            self.nop()
            date >>= 1
        lgpio.gpio_write(self.gpio_handle, SCLK, 0)

    def end(self):
        lgpio.gpio_write(self.gpio_handle, SCLK, 0)
        self.nop()
        lgpio.gpio_write(self.gpio_handle, DIO, 0)
        self.nop()
        lgpio.gpio_write(self.gpio_handle, SCLK, 1)
        self.nop()
        lgpio.gpio_write(self.gpio_handle, DIO, 1)
        self.nop()

    def matrix_display(self, matrix_value):
        self.start()
        self.send_date(0xc0)
        for i in range(0, 16):
            self.send_date(matrix_value[15 - i])
        self.end()
        self.start()
        self.send_date(0x8A)
        self.end()

    def text_to_matrix(self, text):
        matrices = []
        total_width = 0
        for char in text:
            if char in FONT_DATA:
                matrix, width = FONT_DATA[char]
                matrices.append(matrix)
                total_width += width
            else:
                matrix, width = FONT_DATA[' ']  # Default to space if character is not found
                matrices.append(matrix)
                total_width += width
        return matrices, total_width

    def scroll_text(self, text):
        matrices, total_width = self.text_to_matrix(text)
        combined_matrix = []

        for matrix in matrices:
            combined_matrix.extend(matrix)
            combined_matrix.append(0x00)

        while len(combined_matrix) < 16:
            combined_matrix.append(0x00)

        for start_col in range(len(combined_matrix) - 16 + 1):
            with self.lock:
                if self.interrupt_display:
                    self.interrupt_display = False
                    return
            display = combined_matrix[start_col:start_col + 16]
            self.matrix_display(display)
            if start_col == 0:
                time.sleep(self.start_delay)
            time.sleep(self.scrolling_delay)

        time.sleep(self.end_delay)

    def center_text(self, text):
        matrices, total_width = self.text_to_matrix(text)
        combined_matrix = []

        for matrix in matrices:
            combined_matrix.extend(matrix)
            combined_matrix.append(0x00)

        text_length = total_width + (len(matrices) - 1)
        if text_length < 16:
            left_padding = (16 - text_length) // 2
            combined_matrix = [0x00] * left_padding + combined_matrix + [0x00] * (16 - len(combined_matrix) - left_padding)

        self.matrix_display(combined_matrix)

    def display_loop(self):
        while self.running:
            with self.lock:
                text = self.text
                matrices, total_width = self.text_to_matrix(text)
            if total_width + (len(matrices) - 1) <= 16:
                self.center_text(text)
            else:
                self.scroll_text(text)

    def shutdown(self):
        self.running = False
        self.display_thread.join()
        lgpio.gpiochip_close(self.gpio_handle)
        self.destroy_node()
        self.get_logger().info("Node shutdown complete")

def main(args=None):
    rclpy.init(args=args)
    led_panel = LedPanelNode()
    try:
        rclpy.spin(led_panel)
    except KeyboardInterrupt:
        led_panel.get_logger().info("Shutting down due to keyboard interrupt")
    finally:
        led_panel.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()