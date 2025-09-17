import rclpy
from rclpy.node import Node
import threading
import sys
import termios
import tty

from std_msgs.msg import Float64MultiArray


class ThrusterTeleOp(Node):

    def __init__(self):
        super().__init__('thruster_teleop')
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 'hydrus_thruster', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.thruster_speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Vertical movement settings
        self.vertical_thrust = 0.0
        self.thrust_increment = 0.1
        self.max_thrust = 1.0
        self.min_thrust = -1.0

        # Start keyboard listener thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.print_instructions()

    def print_instructions(self):
        print("\n=== Hydrus Thruster Teleop Control ===")
        print("Controls:")
        print("  w/W : Move up (increase thrust)")
        print("  s/S : Move down (decrease thrust)")
        print("  x/X : Stop all thrusters")
        print("  q/Q : Quit")
        print("=====================================\n")

    def keyboard_listener(self):
        """Listen for keyboard input in a separate thread"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                key = sys.stdin.read(1)
                if key:
                    self.process_key(key)
        except Exception as e:
            self.get_logger().error(f'Keyboard listener error: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def process_key(self, key):
        """Process keyboard input"""
        key_lower = key.lower()

        if key_lower == 'q':
            # Move up - increase thrust
            self.vertical_thrust = min(self.max_thrust,
                                       self.vertical_thrust + self.thrust_increment)
            self.get_logger().info(f'Moving up - Thrust: {self.vertical_thrust:.1f}')

        elif key_lower == 'e':
            # Move down - decrease thrust (negative values)
            self.vertical_thrust = max(self.min_thrust,
                                       self.vertical_thrust - self.thrust_increment)
            self.get_logger().info(f'Moving down - Thrust: {self.vertical_thrust:.1f}')

        elif key_lower == 'x':
            # Stop all thrusters
            self.vertical_thrust = 0.0
            self.get_logger().info('Stopping all thrusters')

        elif key_lower == 'z':
            # Quit
            self.get_logger().info('Quitting...')
            self.running = False
            rclpy.shutdown()

        # Update thruster speeds (last 4 thrusters for vertical movement)
        for i in range(4, 8):  # Thrusters 4, 5, 6, 7
            self.thruster_speeds[i] = self.vertical_thrust

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.thruster_speeds
        self.publisher_.publish(msg)
        # Only log when thrust values change to reduce spam
        if any(speed != 0.0 for speed in self.thruster_speeds):
            self.get_logger().info(
                f'Thruster speeds: {[f"{speed:.1f}" for speed in self.thruster_speeds]}')


def main(args=None):
    rclpy.init(args=args)

    thruster_teleop = None
    try:
        thruster_teleop = ThrusterTeleOp()
        rclpy.spin(thruster_teleop)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if thruster_teleop is not None:
            thruster_teleop.running = False
            thruster_teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
