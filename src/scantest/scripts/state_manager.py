#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose2D
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from std_msgs.msg import String

# Define the publishing rate
PUBLISH_FREQUENCY_HZ = 1.0

# Define possible states
STATE_EXPLORE = "EXPLORE/0"
STATE_STOP = "STOP/1"
STATE_SOLVE = "SOLVE/2"
STATE_RETURN = "RETURN/3"


class StateManager(Node):

    def __init__(self):
        super().__init__('state_manager_node')

        # Start directly in EXPLORE
        self.current_state = STATE_EXPLORE

        # --- Parameter Declarations ---
        self.declare_parameter(
            'stop_control',
            False,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)
        )
        self.declare_parameter(
            'solve_control',
            False,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)
        )
        self.declare_parameter(
            'target_x',
            0,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Target X grid coordinate.')
        )
        self.declare_parameter(
            'target_y',
            0,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Target Y grid coordinate.')
        )

        # Register parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Load initial parameters
        self._load_target_grid()

        # --- Publishers, Subscribers, and Timers ---
        self.subscription = self.create_subscription(
            Pose2D,
            '/grid_location',
            self.listener_callback,
            10
        )

        timer_period = 1.0 / PUBLISH_FREQUENCY_HZ
        self.publisher_state = self.create_publisher(String, '/current_state', 10)
        self.timer = self.create_timer(timer_period, self.state_publisher_callback)

        self.get_logger().info(f'State Manager Node started in state: {self.current_state}.')

    def _load_target_grid(self):
        """Helper to load/check the target grid parameters and update members."""
        self.target_x = int(self.get_parameter('target_x').value)
        self.target_y = int(self.get_parameter('target_y').value)
        self.get_logger().info(f"Target Grid Loaded: ({self.target_x}, {self.target_y})")

    def parameter_callback(self, params):
        """Handle runtime parameter updates."""
        success = True

        for param in params:
            if param.name == 'target_x':
                if param.type_ == Parameter.Type.INTEGER:
                    self.target_x = int(param.value)
                    self.get_logger().warn(f"Target X UPDATED at Runtime to: {self.target_x}")
                else:
                    self.get_logger().error(f"Invalid type for target_x: {param.type_}")
                    success = False

            elif param.name == 'target_y':
                if param.type_ == Parameter.Type.INTEGER:
                    self.target_y = int(param.value)
                    self.get_logger().warn(f"Target Y UPDATED at Runtime to: {self.target_y}")
                else:
                    self.get_logger().error(f"Invalid type for target_y: {param.type_}")
                    success = False

            elif param.name in ['stop_control', 'solve_control']:
                if param.type_ != Parameter.Type.BOOL:
                    self.get_logger().error(f"Invalid type for control parameter {param.name}: {param.type_}")
                    success = False

            else:
                self.get_logger().error(f"Attempted to set unknown parameter: {param.name}")
                success = False

        if success:
            self.get_logger().info(f"Final Target Coordinates: ({self.target_x}, {self.target_y})")

        return SetParametersResult(successful=success)

    def state_publisher_callback(self):
        """Publishes current state and checks for control-triggered transitions."""
        state_msg = String()
        state_msg.data = self.current_state
        self.publisher_state.publish(state_msg)

        self._check_stop_param_transition()
        self._check_solve_param_transition()

    def _check_stop_param_transition(self):
        """If stop_control becomes True during EXPLORE, transition to STOP."""
        if self.current_state == STATE_EXPLORE:
            if self.get_parameter('stop_control').value:
                self.transition_state(STATE_STOP, "Parameter 'stop_control' is True.")

    def _check_solve_param_transition(self):
        """If solve_control becomes True during STOP, transition to SOLVE."""
        if self.current_state == STATE_STOP:
            if self.get_parameter('solve_control').value:
                self.transition_state(STATE_SOLVE, "Parameter 'solve_control' is True.")

    def listener_callback(self, msg):
        """Handles state transitions based on location and current state."""
        try:
            grid_x = int(msg.x)
            grid_y = int(msg.y)
        except (ValueError, TypeError):
            self.get_logger().error("Received non-numeric grid location.")
            return

        if self.current_state == STATE_EXPLORE:
            if self.get_parameter('stop_control').value:
                self.transition_state(STATE_STOP, "Parameter 'stop_control' is True. Exiting EXPLORE.")

        elif self.current_state == STATE_SOLVE:
            if grid_x == self.target_x and grid_y == self.target_y:
                self.transition_state(STATE_RETURN, f"Reached target ({self.target_x}, {self.target_y}).")

    def transition_state(self, new_state, reason=""):
        self.get_logger().warn(
            f"STATE TRANSITION: {self.current_state} -> {new_state} (Reason: {reason})"
        )
        self.current_state = new_state


def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()
    try:
        rclpy.spin(state_manager)
    except KeyboardInterrupt:
        pass
    finally:
        state_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
