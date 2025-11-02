import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String

# Define the publishing rate
PUBLISH_FREQUENCY_HZ = 1.0

# Define the possible states
STATE_IDLE = "IDLE"      
STATE_EXPLORE = "EXPLORE"
STATE_STOP = "STOP"      
STATE_SOLVE = "SOLVE"    
STATE_RETURN = "RETURN"  

class StateManager(Node):

    def __init__(self):
        super().__init__('state_manager_node')
        
        # 1. State Initialization
        self.current_state = STATE_IDLE
        
        # --- Parameter Declarations ---
        
        # New: 1. Explore Control Parameter (Default value is False)
        explore_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='If true, moves the node into the EXPLORE state from IDLE.'
        )
        self.declare_parameter('explore_control', False, explore_descriptor) 

        # Existing: 2. Stop Control Parameter
        stop_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='If true, forces the node into the STOP state from EXPLORE.'
        )
        self.declare_parameter('stop_control', False, stop_descriptor) 

        # Existing: 3. Target Grid Parameter
        target_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER_ARRAY,
            description='Target grid [x, y] for the RETURN state transition.'
        )
        self.declare_parameter('target_grid', [0, 0], target_descriptor)
        
        target_param = self.get_parameter('target_grid').value
        if len(target_param) == 2:
             self.target_x, self.target_y = target_param[0], target_param[1]
        else:
             self.get_logger().error(f"Invalid 'target_grid' parameter: {target_param}. Defaulting to [0, 0].")
             self.target_x, self.target_y = 0, 0

        self.get_logger().info(f"Target Grid: ({self.target_x}, {self.target_y})")

        # --- Publishers, Subscribers, and Timers ---
        
        # Subscribe to /grid_location (Pose2D)
        self.subscription = self.create_subscription(
            Pose2D,
            '/grid_location',
            self.listener_callback,
            10)
        
        # State Publisher and Timer for 1Hz updates
        timer_period = 1.0 / PUBLISH_FREQUENCY_HZ
        self.publisher_state = self.create_publisher(String, '/current_state', 10)
        self.timer = self.create_timer(timer_period, self.state_publisher_callback)
        
        self.get_logger().info(f'State Manager Node started in state: **{self.current_state}**.')

    # ----------------------------------------------------------------
    # --- Timer Callback and Parameter Check ---
    # ----------------------------------------------------------------
    def state_publisher_callback(self):
        # Publish current state
        state_msg = String()
        state_msg.data = self.current_state
        self.publisher_state.publish(state_msg)
        
        # Check and handle the 'explore_control' parameter for IDLE -> EXPLORE transition
        self._check_explore_param_transition()


    def _check_explore_param_transition(self):
        """Checks the value of the 'explore_control' parameter and handles the state change."""
        explore_control_value = self.get_parameter('explore_control').value

        # Transition IDLE -> EXPLORE if parameter is True
        if explore_control_value and self.current_state == STATE_IDLE:
             self.transition_state(STATE_EXPLORE, "Parameter 'explore_control' is True.")


    # ----------------------------------------------------------------
    # --- Subscriber Callback (Location Based Transitions) ---
    # ----------------------------------------------------------------
    def listener_callback(self, msg):
        """Handles state transitions based on location and current state."""
        grid_x = int(msg.x)
        grid_y = int(msg.y)
        
        # State Machine Logic
        
        if self.current_state == STATE_IDLE:
            # Transition to EXPLORE is handled by the timer/parameter check.
            pass
            
        # New Logic: EXPLORE -> STOP Transition
        elif self.current_state == STATE_EXPLORE:
            stop_control_value = self.get_parameter('stop_control').value
            
            # Transition EXPLORE -> STOP
            # Condition: stop_control parameter is True
            if stop_control_value:
                self.transition_state(STATE_STOP, "Parameter 'stop_control' is True. Exiting EXPLORE.")

        elif self.current_state == STATE_STOP:
            # Reset the explore control when transitioning past STOP
            self.get_logger().info("Resetting 'explore_control' and 'stop_control' parameters to False.")
            self.set_parameters([
                rclpy.parameter.Parameter('explore_control', rclpy.Parameter.Type.BOOL, False),
                rclpy.parameter.Parameter('stop_control', rclpy.Parameter.Type.BOOL, False)
            ])
            
            # 3. Exit STOP state and enter SOLVE state
            # Condition: Location is (0, 0)
            if grid_x == 0 and grid_y == 0:
                self.transition_state(STATE_SOLVE, "Reached (0, 0). Exiting STOP.")
                
        elif self.current_state == STATE_SOLVE:
            # 4. Exit SOLVE state and enter RETURN state
            # Condition: Location is equal to target_x and target_y
            if grid_x == self.target_x and grid_y == self.target_y:
                self.transition_state(STATE_RETURN, f"Reached target ({self.target_x}, {self.target_y}).")
            
    # --- Helper Function for State Transitions ---
    def transition_state(self, new_state, reason=""):
        """Logs the state transition."""
        self.get_logger().warn(
            f"STATE TRANSITION: **{self.current_state}** -> **{new_state}** (Reason: {reason})"
        )
        self.current_state = new_state


def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()
    rclpy.spin(state_manager)
    state_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()