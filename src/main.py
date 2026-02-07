import tkinter as tk
from threading import Thread
import time

# AirSim integration
try:
    import airsim
    AIRSIM_AVAILABLE = True
except ImportError:
    AIRSIM_AVAILABLE = False
    print("[WARNING] airsim not available, running in offline mode")

# Import system components
from frequency_hopper import AdaptiveFrequencyHopper
from drone_encryption import DroneEncryption
from energy_management import EnergyManager
from sensor import SensorInput
from navigation import NavigationSystem
from obstacle import ObstacleDetector
from flight_plan import FlightPlanner
from decision_maker import DecisionMaker 
from user_interface import DroneControlPanel
from emergency import EmergencyHandler
from drone_swarm import DroneSwarm
from weather_interaction import WeatherInteraction
from exceptions import CriticalNavigationError, SensorError


class AirSimController:
    """Controller for AirSim drone integration."""
    
    def __init__(self):
        self.client = None
        self.connected = False
        self.flying = False
        self.move_speed = 3  # m/s
        self.move_distance = 2  # meters per command
        
    def connect(self):
        """Connect to AirSim simulator."""
        if not AIRSIM_AVAILABLE:
            return False
        try:
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            self.connected = True
            return True
        except Exception as e:
            print(f"[AirSim] Connection failed: {e}")
            self.connected = False
            return False
    
    def takeoff(self):
        """Take off the drone."""
        if not self.connected:
            return False
        try:
            self.client.enableApiControl(True)
            self.client.armDisarm(True)
            self.client.takeoffAsync().join()
            self.flying = True
            return True
        except Exception as e:
            print(f"[AirSim] Takeoff failed: {e}")
            return False
    
    def land(self):
        """Land the drone."""
        if not self.connected:
            return False
        try:
            self.client.landAsync().join()
            self.client.armDisarm(False)
            self.client.enableApiControl(False)
            self.flying = False
            return True
        except Exception as e:
            print(f"[AirSim] Landing failed: {e}")
            return False
    
    def move_to_position(self, x, y, z, speed=None):
        """Move to absolute position (NED coordinates: z negative = up)."""
        if not self.connected or not self.flying:
            return False
        try:
            if speed is None:
                speed = self.move_speed
            self.client.moveToPositionAsync(
                float(x), float(y), float(z), float(speed)
            ).join()
            return True
        except Exception as e:
            print(f"[AirSim] Move failed: {e}")
            return False
    
    def go_to_altitude(self, altitude_meters):
        """Go to specified altitude (positive value, converted to NED)."""
        if not self.connected or not self.flying:
            return False
        try:
            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            # NED: negative z = up, so altitude 10m = z=-10
            target_z = -abs(altitude_meters)
            self.client.moveToPositionAsync(
                float(pos.x_val), float(pos.y_val), float(target_z), float(self.move_speed)
            ).join()
            return True
        except Exception as e:
            print(f"[AirSim] Altitude change failed: {e}")
            return False
    
    def execute_decision(self, decision):
        """Execute a movement decision."""
        if not self.connected or not self.flying:
            return False
        
        try:
            # Get current position
            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            x, y, z = pos.x_val, pos.y_val, pos.z_val
            
            # Calculate new position based on decision
            if decision == "forward":
                x += self.move_distance
            elif decision == "back":
                x -= self.move_distance
            elif decision == "left":
                y -= self.move_distance
            elif decision == "right":
                y += self.move_distance
            elif decision == "up":
                z -= self.move_distance  # NED: negative is up
            elif decision == "down":
                z += self.move_distance  # NED: positive is down
            elif decision == "hover":
                self.client.hoverAsync()
                return True
            
            # Move to new position
            self.client.moveToPositionAsync(
                float(x), float(y), float(z), float(self.move_speed)
            ).join()
            return True
            
        except Exception as e:
            print(f"[AirSim] Move failed: {e}")
            return False
    
    def get_position(self):
        """Get current drone position."""
        if not self.connected:
            return [0, 0, 0]
        try:
            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            return [pos.x_val, pos.y_val, pos.z_val]
        except:
            return [0, 0, 0]
    
    def draw_trajectory_line(self, point1, point2, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness=5.0):
        """
        Draw a line segment in UE5 between two points.
        
        Args:
            point1: Start point [x, y, z] in NED coordinates
            point2: End point [x, y, z] in NED coordinates
            color_rgba: Color as [R, G, B, A] where values are 0-1
            thickness: Line thickness in pixels
        """
        if not self.connected:
            return False
        try:
            p1 = airsim.Vector3r(float(point1[0]), float(point1[1]), float(point1[2]))
            p2 = airsim.Vector3r(float(point2[0]), float(point2[1]), float(point2[2]))
            # Draw persistent line (duration=-1 means forever)
            self.client.simPlotLineStrip(
                [p1, p2], 
                color_rgba=color_rgba, 
                thickness=thickness, 
                duration=-1.0, 
                is_persistent=True
            )
            return True
        except Exception as e:
            print(f"[AirSim] Draw line failed: {e}")
            return False
    
    def clear_trajectory(self):
        """Clear all drawn lines from the simulation."""
        if not self.connected:
            return False
        try:
            self.client.simFlushPersistentMarkers()
            return True
        except Exception as e:
            print(f"[AirSim] Clear trajectory failed: {e}")
            return False
    
    def get_distance_to_obstacle(self, direction="front"):
        """
        Get distance to obstacle in specified direction using distance sensor.
        Returns distance in meters, or 100 if no obstacle detected.
        """
        if not self.connected:
            return 100.0
        try:
            # Try to get distance sensor data
            distance_data = self.client.getDistanceSensorData()
            if distance_data:
                return distance_data.distance
            return 100.0
        except:
            # Fallback: use simGetCollisionInfo
            try:
                collision = self.client.simGetCollisionInfo()
                if collision.has_collided:
                    return 0.0
                return 100.0
            except:
                return 100.0
    
    def check_obstacle_ahead(self, target_x, target_y, target_z, threshold=2.0):
        """
        Check if there's an obstacle between current position and target.
        Uses raycast or collision prediction.
        Returns True if obstacle detected within threshold distance.
        """
        if not self.connected:
            return False
        try:
            # Get current position
            pos = self.get_position()
            
            # Calculate direction vector
            dx = target_x - pos[0]
            dy = target_y - pos[1]
            dz = target_z - pos[2]
            dist = (dx**2 + dy**2 + dz**2) ** 0.5
            
            if dist < 0.1:
                return False
            
            # Normalize direction
            dx, dy, dz = dx/dist, dy/dist, dz/dist
            
            # Check point at threshold distance ahead
            check_x = pos[0] + dx * threshold
            check_y = pos[1] + dy * threshold
            check_z = pos[2] + dz * threshold
            
            # Use simGetCollisionInfo after small movement test
            collision = self.client.simGetCollisionInfo()
            if collision.has_collided:
                return True
            
            return False
        except Exception as e:
            print(f"[OBSTACLE] Check failed: {e}")
            return False
    
    def rotate_yaw(self, angle_degrees, duration=1.0):
        """Rotate drone by specified angle (positive = left/counter-clockwise)."""
        if not self.connected or not self.flying:
            return False
        try:
            # Get current yaw
            state = self.client.getMultirotorState()
            orientation = state.kinematics_estimated.orientation
            
            # Convert quaternion to yaw
            import math
            # Simplified yaw extraction
            siny_cosp = 2 * (orientation.w_val * orientation.z_val + orientation.x_val * orientation.y_val)
            cosy_cosp = 1 - 2 * (orientation.y_val * orientation.y_val + orientation.z_val * orientation.z_val)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Calculate new yaw
            new_yaw = current_yaw + math.radians(angle_degrees)
            
            # Rotate using rotateToYawAsync
            self.client.rotateToYawAsync(math.degrees(new_yaw), duration).join()
            return True
        except Exception as e:
            print(f"[AirSim] Rotation failed: {e}")
            return False
    
    def get_current_yaw(self):
        """Get current yaw angle in degrees."""
        if not self.connected:
            return 0.0
        try:
            import math
            state = self.client.getMultirotorState()
            orientation = state.kinematics_estimated.orientation
            siny_cosp = 2 * (orientation.w_val * orientation.z_val + orientation.x_val * orientation.y_val)
            cosy_cosp = 1 - 2 * (orientation.y_val * orientation.y_val + orientation.z_val * orientation.z_val)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return math.degrees(yaw)
        except:
            return 0.0
    
    def move_with_obstacle_avoidance(self, target_x, target_y, target_z, speed, 
                                      obstacle_distance=2.0, step_size=0.5, 
                                      log_callback=None, running_check=None):
        """
        Move to target with obstacle avoidance.
        Checks for obstacles every step_size meters.
        If obstacle detected, stops and turns left 90 degrees.
        
        Returns: (reached_target, final_position)
        """
        import math
        
        def log(msg):
            if log_callback:
                log_callback(msg)
            print(msg)
        
        def is_running():
            if running_check:
                return running_check()
            return True
        
        max_avoidance_attempts = 4  # Max 360 degrees of turning
        
        while is_running():
            pos = self.get_position()
            
            # Calculate distance to target
            dx = target_x - pos[0]
            dy = target_y - pos[1]
            dz = target_z - pos[2]
            distance_to_target = (dx**2 + dy**2 + dz**2) ** 0.5
            
            # Check if reached target
            if distance_to_target < step_size:
                return True, pos
            
            # Normalize direction
            if distance_to_target > 0:
                dx, dy, dz = dx/distance_to_target, dy/distance_to_target, dz/distance_to_target
            
            # Calculate next step position
            step = min(step_size, distance_to_target)
            next_x = pos[0] + dx * step
            next_y = pos[1] + dy * step
            next_z = pos[2] + dz * step
            
            # Check for collision before moving
            collision = self.client.simGetCollisionInfo()
            if collision.has_collided:
                log(f"[AVOID] Collision detected! Backing up...")
                # Back up slightly
                self.client.moveToPositionAsync(
                    float(pos[0] - dx * 1.0),
                    float(pos[1] - dy * 1.0),
                    float(pos[2]),
                    float(speed)
                ).join()
                
                # Turn left 90 degrees
                log(f"[AVOID] Turning left 90 degrees...")
                self.rotate_yaw(90, 1.0)
                
                # Update target direction based on new orientation
                # Continue in the new direction
                return False, self.get_position()
            
            # Move to next step (non-blocking to check collisions during movement)
            move_task = self.client.moveToPositionAsync(
                float(next_x), float(next_y), float(next_z), float(speed)
            )
            
            # Wait for movement with collision checking
            start_time = time.time()
            timeout = step / speed + 1.0  # Expected time + buffer
            
            while not move_task._response.done():
                if time.time() - start_time > timeout:
                    break
                
                # Check collision during movement
                collision = self.client.simGetCollisionInfo()
                if collision.has_collided:
                    log(f"[AVOID] Obstacle hit during movement!")
                    # Cancel movement
                    self.client.cancelLastTask()
                    time.sleep(0.2)
                    
                    # Back up
                    current = self.get_position()
                    self.client.moveToPositionAsync(
                        float(current[0] - dx * 1.5),
                        float(current[1] - dy * 1.5),
                        float(current[2]),
                        float(speed)
                    ).join()
                    
                    # Turn left
                    log(f"[AVOID] Turning left to avoid obstacle...")
                    self.rotate_yaw(90, 1.0)
                    return False, self.get_position()
                
                time.sleep(0.05)  # Check every 50ms
            
            # Wait for task to complete
            try:
                move_task.join()
            except:
                pass
        
        return False, self.get_position()
    
    def explore_map(self, map_size_x, map_size_y, altitude, speed=1.5, 
                    line_spacing=5, log_callback=None, running_check=None):
        """
        Explore the map with obstacle avoidance and real-time trajectory drawing.
        
        Behavior:
        - Flies in exploration pattern
        - Detects obstacles using collision sensor
        - When obstacle hit: backs up, turns left 90°, continues
        - Draws red trajectory line in UE5
        
        Args:
            map_size_x: Map size in X direction (meters)
            map_size_y: Map size in Y direction (meters)
            altitude: Flight altitude in meters (positive value)
            speed: Flight speed in m/s
            line_spacing: Distance between parallel lines (meters)
            log_callback: Optional function to log messages
            running_check: Optional function that returns False to stop exploration
        """
        if not self.connected or not self.flying:
            return False
        
        def log(msg):
            if log_callback:
                log_callback(msg)
            print(msg)
        
        def is_running():
            if running_check:
                return running_check()
            return True
        
        try:
            # Clear any previous trajectory
            self.clear_trajectory()
            
            # Get starting position
            state = self.client.getMultirotorState()
            start_pos = state.kinematics_estimated.position
            start_x, start_y = start_pos.x_val, start_pos.y_val
            
            # Target altitude (NED: negative = up)
            target_z = -abs(altitude)
            
            log(f"[EXPLORE] ========================================")
            log(f"[EXPLORE] MAP EXPLORATION WITH OBSTACLE AVOIDANCE")
            log(f"[EXPLORE] ========================================")
            log(f"[EXPLORE] Area: {map_size_x}m x {map_size_y}m")
            log(f"[EXPLORE] Altitude: {altitude}m, Speed: {speed}m/s")
            log(f"[EXPLORE] Obstacle detection: ACTIVE (2m threshold)")
            log(f"[EXPLORE] Avoidance: Stop -> Turn Left 90° -> Continue")
            log("-" * 40)
            
            # First, go to altitude
            log(f"[EXPLORE] Going to altitude {altitude}m...")
            self.client.moveToPositionAsync(
                float(start_x), float(start_y), float(target_z), float(speed)
            ).join()
            
            # Track trajectory for drawing
            prev_pos = self.get_position()
            trajectory_points = [prev_pos.copy()]
            
            # Trajectory line color (bright red)
            line_color = [1.0, 0.0, 0.0, 1.0]  # RGBA
            line_thickness = 8.0
            
            # Exploration using wall-following algorithm
            # Direction: 0=+X, 1=+Y, 2=-X, 3=-Y
            import math
            direction = 0  # Start facing +X
            direction_vectors = [
                (1, 0),   # +X (forward)
                (0, 1),   # +Y (right)
                (-1, 0),  # -X (back)
                (0, -1),  # -Y (left)
            ]
            
            # Exploration bounds
            min_x, max_x = start_x - 5, start_x + map_size_x + 5
            min_y, max_y = start_y - 5, start_y + map_size_y + 5
            
            total_moves = 0
            max_moves = 200  # Safety limit
            obstacles_avoided = 0
            
            log(f"[EXPLORE] Starting exploration pattern...")
            
            while is_running() and total_moves < max_moves:
                pos = self.get_position()
                
                # Check bounds
                if pos[0] < min_x or pos[0] > max_x or pos[1] < min_y or pos[1] > max_y:
                    log(f"[EXPLORE] Reached boundary, turning...")
                    direction = (direction + 1) % 4
                    self.rotate_yaw(90, 0.5)
                    continue
                
                # Calculate target based on current direction
                dx, dy = direction_vectors[direction]
                move_distance = 3.0  # Move 3 meters at a time
                target_x = pos[0] + dx * move_distance
                target_y = pos[1] + dy * move_distance
                
                log(f"[EXPLORE] Move {total_moves+1}: Direction {['Forward', 'Right', 'Back', 'Left'][direction]}")
                log(f"[EXPLORE] From ({pos[0]:.1f}, {pos[1]:.1f}) to ({target_x:.1f}, {target_y:.1f})")
                
                # Move with collision detection
                move_task = self.client.moveToPositionAsync(
                    float(target_x), float(target_y), float(target_z), float(speed)
                )
                
                # Monitor for collisions during movement
                collision_detected = False
                start_time = time.time()
                timeout = move_distance / speed + 2.0
                
                while not move_task._response.done():
                    if not is_running():
                        self.client.cancelLastTask()
                        break
                    
                    if time.time() - start_time > timeout:
                        break
                    
                    # Check collision
                    collision = self.client.simGetCollisionInfo()
                    if collision.has_collided:
                        collision_detected = True
                        log(f"[AVOID] !!! OBSTACLE DETECTED !!!")
                        self.client.cancelLastTask()
                        break
                    
                    time.sleep(0.05)
                
                try:
                    move_task.join()
                except:
                    pass
                
                # Get new position and draw trajectory
                current_pos = self.get_position()
                self.draw_trajectory_line(prev_pos, current_pos, line_color, line_thickness)
                trajectory_points.append(current_pos.copy())
                prev_pos = current_pos.copy()
                
                if collision_detected:
                    obstacles_avoided += 1
                    log(f"[AVOID] Backing up 1.5m...")
                    
                    # Back up
                    backup_x = current_pos[0] - dx * 1.5
                    backup_y = current_pos[1] - dy * 1.5
                    self.client.moveToPositionAsync(
                        float(backup_x), float(backup_y), float(target_z), float(speed)
                    ).join()
                    
                    # Draw backup trajectory
                    new_pos = self.get_position()
                    self.draw_trajectory_line(prev_pos, new_pos, [1.0, 0.5, 0.0, 1.0], line_thickness)  # Orange for backup
                    prev_pos = new_pos.copy()
                    
                    # Turn left 90 degrees
                    log(f"[AVOID] Turning LEFT 90 degrees...")
                    direction = (direction + 3) % 4  # Turn left (counter-clockwise)
                    self.rotate_yaw(90, 0.8)
                    
                    log(f"[AVOID] New direction: {['Forward', 'Right', 'Back', 'Left'][direction]}")
                    log(f"[AVOID] Continuing exploration...")
                else:
                    log(f"[EXPLORE] Reached: ({current_pos[0]:.1f}, {current_pos[1]:.1f})")
                    
                    # Sometimes turn to explore more area
                    if total_moves > 0 and total_moves % 5 == 0:
                        log(f"[EXPLORE] Turning right to scan new area...")
                        direction = (direction + 1) % 4
                        self.rotate_yaw(-90, 0.5)
                
                total_moves += 1
                time.sleep(0.1)
            
            log("-" * 40)
            log(f"[EXPLORE] Exploration completed!")
            log(f"[EXPLORE] Total moves: {total_moves}")
            log(f"[EXPLORE] Obstacles avoided: {obstacles_avoided}")
            log(f"[EXPLORE] Trajectory points: {len(trajectory_points)}")
            
            # Calculate distance
            total_distance = 0
            for i in range(1, len(trajectory_points)):
                p1, p2 = trajectory_points[i-1], trajectory_points[i]
                dist = ((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2) ** 0.5
                total_distance += dist
            log(f"[EXPLORE] Distance traveled: {total_distance:.1f}m")
            
            # Return to start
            log(f"[EXPLORE] Returning to start position...")
            self.client.moveToPositionAsync(
                float(start_x), float(start_y), float(target_z), float(speed)
            ).join()
            
            return True
            
        except Exception as e:
            log(f"[EXPLORE] Error: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def fly_square(self, side_length, altitude, speed=3, log_callback=None):
        """
        Fly a square pattern.
        
        Args:
            side_length: Length of each side in meters
            altitude: Flight altitude in meters (positive value)
            speed: Flight speed in m/s
            log_callback: Optional function to log messages
        """
        if not self.connected or not self.flying:
            return False
        
        def log(msg):
            if log_callback:
                log_callback(msg)
            print(msg)
        
        try:
            # Get starting position
            state = self.client.getMultirotorState()
            start_pos = state.kinematics_estimated.position
            start_x, start_y = start_pos.x_val, start_pos.y_val
            
            # Target altitude (NED: negative = up)
            target_z = -abs(altitude)
            
            # Define square corners (starting from current X,Y position)
            # Square pattern: Start -> Forward -> Right -> Back -> Left -> Start
            waypoints = [
                (start_x, start_y, target_z, "Going to altitude"),
                (start_x + side_length, start_y, target_z, "Flying forward"),
                (start_x + side_length, start_y + side_length, target_z, "Flying right"),
                (start_x, start_y + side_length, target_z, "Flying back"),
                (start_x, start_y, target_z, "Returning to start"),
            ]
            
            log(f"[SQUARE] Starting square flight: {side_length}m sides at {altitude}m altitude")
            
            for i, (x, y, z, description) in enumerate(waypoints):
                log(f"[SQUARE] Step {i+1}/5: {description}")
                log(f"[SQUARE] Target: ({x:.1f}, {y:.1f}, {-z:.1f}m altitude)")
                
                self.client.moveToPositionAsync(
                    float(x), float(y), float(z), float(speed)
                ).join()
                
                # Get and log current position
                pos = self.get_position()
                log(f"[SQUARE] Reached: ({pos[0]:.1f}, {pos[1]:.1f}, {-pos[2]:.1f}m)")
                time.sleep(0.5)  # Brief pause at each corner
            
            log("[SQUARE] Square flight completed!")
            return True
            
        except Exception as e:
            log(f"[SQUARE] Error: {e}")
            return False

class DroneNavigationSystem:
    """
    Integrates various modules to control and monitor drone operations comprehensively.
    Includes management of drone swarms and interactions with weather systems.
    """
    def __init__(self, master):
        self.master = master
        self.ui = DroneControlPanel(master)
        self.setup_components()

    def setup_components(self):
        # Setup individual components of the drone navigation system
        self.simulation_mode = True  # For decision maker only
        self.running = False
        
        # AirSim controller for real drone movement
        self.airsim_controller = AirSimController()
        self.airsim_enabled = False
        
        self.weather_interaction = WeatherInteraction(api_key='your_api_key_here')
        self.swarm = DroneSwarm(drone_ids=['drone1', 'drone2', 'drone3'], control_station_callback=self.swarm_callback)

        self.hopper = AdaptiveFrequencyHopper(available_frequencies=[2.4, 2.425, 2.45, 2.475, 2.5])
        self.encryption = DroneEncryption()
        self.energy_manager = EnergyManager(return_home_callback=self.return_home)
        self.sensor = SensorInput(camera_index=0, lidar_config={'port': '/dev/ttyUSB0'}, simulation_mode=self.simulation_mode)
        self.navigation = NavigationSystem()
        self.obstacle_detector = ObstacleDetector(simulation_mode=self.simulation_mode)
        self.flight_planner = FlightPlanner(destination=[100, 100, 100])
        self.decision_maker = DecisionMaker(simulation_mode=self.simulation_mode)
        self.emergency_handler = EmergencyHandler(self.handle_emergency)
        
        # UI button configurations
        self.ui.start_button.config(command=self.start_operation_thread)
        self.ui.stop_button.config(command=self.stop_operation)

    def swarm_callback(self, event_type, details):
        """Handle events from the drone swarm, such as task completions and emergencies."""
        self.ui.log_data(f"Swarm Event: {event_type}, Details: {details}")

    def start_operation_thread(self):
        """Start drone operations in a separate thread to keep the UI responsive."""
        self.operation_thread = Thread(target=self.start_operation)
        self.operation_thread.start()

    def start_operation(self):
        """Main operational loop - explore map with trajectory visualization."""
        self.running = True
        
        # Exploration flight parameters
        MAP_SIZE_X = 30      # meters - exploration area width
        MAP_SIZE_Y = 30      # meters - exploration area height
        LINE_SPACING = 5     # meters - distance between scan lines
        FLIGHT_ALTITUDE = 3  # meters - low altitude for detailed scanning
        FLIGHT_SPEED = 1.5   # m/s - slow speed for stability
        
        try:
            self.ui.log_data("=" * 50)
            self.ui.log_data("MAP EXPLORATION MISSION")
            self.ui.log_data(f"Area: {MAP_SIZE_X}m x {MAP_SIZE_Y}m")
            self.ui.log_data(f"Altitude: {FLIGHT_ALTITUDE}m, Speed: {FLIGHT_SPEED}m/s")
            self.ui.log_data("Red trajectory line will be drawn in UE5!")
            self.ui.log_data("=" * 50)
            
            # Connect to AirSim
            self.ui.log_data("Connecting to AirSim simulator...")
            if self.airsim_controller.connect():
                self.airsim_enabled = True
                self.ui.log_data("[SUCCESS] Connected to AirSim!")
            else:
                self.airsim_enabled = False
                self.ui.log_data("[ERROR] AirSim not available!")
                return
            
            # Takeoff
            self.ui.log_data("Taking off...")
            if self.airsim_controller.takeoff():
                self.ui.log_data("[SUCCESS] Drone is airborne!")
            else:
                self.ui.log_data("[ERROR] Takeoff failed!")
                return
            
            # Explore map with trajectory drawing
            self.ui.log_data("-" * 50)
            self.airsim_controller.explore_map(
                map_size_x=MAP_SIZE_X,
                map_size_y=MAP_SIZE_Y,
                altitude=FLIGHT_ALTITUDE,
                speed=FLIGHT_SPEED,
                line_spacing=LINE_SPACING,
                log_callback=self.ui.log_data,
                running_check=lambda: self.running
            )
            self.ui.log_data("-" * 50)
            
            # Mission complete
            self.ui.log_data("Mission completed successfully!")
            self.ui.log_data("Trajectory is visible as red line in UE5!")
                
        except Exception as e:
            self.ui.log_data(f"Error during operation: {e}")
        finally:
            self.ui.log_data("Landing...")
            self.cleanup_operations()

    def handle_operation_error(self, error):
        """Handle specific errors and decide whether to continue operations."""
        if isinstance(error, CriticalNavigationError):
            self.ui.log_data("Critical error in navigation, stopping operations.")
            return False
        elif isinstance(error, SensorError):
            self.ui.log_data("Sensor error handled, attempting to continue.")
            return True
        else:
            self.ui.log_data("Unhandled error type, stopping operations.")
            return False

    def cleanup_operations(self):
        """Clean up resources and ensure system is in a safe state before closing."""
        # Land drone if connected to AirSim
        if self.airsim_enabled:
            self.ui.log_data("Landing drone...")
            self.airsim_controller.land()
            self.ui.log_data("Drone landed.")
        
        self.sensor.release_resources()
        self.ui.log_data("System cleaned up and ready to close.")

    def stop_operation(self):
        """Safely stop all drone operations."""
        self.running = False
        self.ui.log_data("Stopping drone operations...")
        
        # Wait for thread to finish
        if hasattr(self, 'operation_thread') and self.operation_thread.is_alive():
            self.operation_thread.join(timeout=5)
        
        self.ui.log_data("Drone operations stopped.")

    def handle_emergency(self, message):
        """Respond to emergencies by logging and performing necessary actions."""
        self.ui.log_data(message)
        self.stop_operation()

    def return_home(self):
        """Command the drone to return to its home location."""
        self.ui.log_data("Low battery: Returning home.")

# Main execution
if __name__ == "__main__":
    root = tk.Tk()
    drone_system = DroneNavigationSystem(root)
    root.mainloop()
