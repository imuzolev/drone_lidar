"""
Тест подключения к Colosseum/AirSim для управления дроном.

Перед запуском убедитесь, что:
1. Unreal Engine проект Blocks запущен
2. Симуляция началась (нажат Play в редакторе)

Запуск:
    python src/test_colosseum.py
"""

import airsim
import time
import sys
import threading

class TrajectoryTracker(threading.Thread):
    """Thread to continuously draw drone trajectory."""
    def __init__(self, update_interval=0.1, color=[1.0, 0.0, 0.0, 1.0], thickness=5.0):
        super().__init__()
        # We create a new client for the thread to avoid conflicts
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.update_interval = update_interval
        self.color = color
        self.thickness = thickness
        self.running = False
        self.prev_pos = None

    def run(self):
        self.running = True
        while self.running:
            try:
                state = self.client.getMultirotorState()
                pos = state.kinematics_estimated.position
                current_pos = [pos.x_val, pos.y_val, pos.z_val]

                if self.prev_pos:
                    dist = ((current_pos[0]-self.prev_pos[0])**2 + 
                            (current_pos[1]-self.prev_pos[1])**2 + 
                            (current_pos[2]-self.prev_pos[2])**2)**0.5
                    
                    if dist > 0.05: # Only draw if moved > 5cm
                        p1 = airsim.Vector3r(self.prev_pos[0], self.prev_pos[1], self.prev_pos[2])
                        p2 = airsim.Vector3r(current_pos[0], current_pos[1], current_pos[2])
                        self.client.simPlotLineStrip(
                            [p1, p2], 
                            color_rgba=self.color, 
                            thickness=float(self.thickness), 
                            duration=-1.0, 
                            is_persistent=True
                        )
                        self.prev_pos = current_pos
                else:
                    self.prev_pos = current_pos
                
                time.sleep(self.update_interval)
            except Exception:
                # Ignore errors during tracking (e.g. connection lost on exit)
                pass

    def stop(self):
        self.running = False

def _plot_red_segment(client, p1, p2, thickness=5.0):
    """Рисует красный отрезок траектории в UE (NED координаты)."""
    try:
        v1 = airsim.Vector3r(float(p1[0]), float(p1[1]), float(p1[2]))
        v2 = airsim.Vector3r(float(p2[0]), float(p2[1]), float(p2[2]))
        client.simPlotLineStrip(
            [v1, v2],
            color_rgba=[1.0, 0.0, 0.0, 1.0],
            thickness=float(thickness),
            duration=-1.0,
            is_persistent=True,
        )
    except Exception as e:
        # Не валим тест из-за визуализации (может быть не поддержано конкретной сборкой)
        print(f"[WARNING] Не удалось нарисовать траекторию: {e}")


def test_connection():
    """Test connection to simulator."""
    print("=" * 60)
    print("Connection Test Colosseum/AirSim")
    print("=" * 60)
    
    try:
        # Create client
        client = airsim.MultirotorClient()
        print("[INFO] Client created")
        
        # Connect
        print("[INFO] Connecting to simulator...")
        client.confirmConnection()
        print("[SUCCESS] Connected!")
        
        # Get state
        state = client.getMultirotorState()
        print(f"\n[INFO] Drone State:")
        print(f"  - Position: x={state.kinematics_estimated.position.x_val:.2f}, "
              f"y={state.kinematics_estimated.position.y_val:.2f}, "
              f"z={state.kinematics_estimated.position.z_val:.2f}")
        print(f"  - Landed: {state.landed_state}")
        
        return True
        
    except Exception as e:
        print(f"[ERROR] Connection error: {e}")
        return False


def test_basic_flight():
    """Basic flight test: 1x1m square at 1m altitude."""
    print("\n" + "=" * 60)
    print("Basic Flight Test")
    print("=" * 60)
    
    # Parameters
    SIDE_M = 2.0
    ALTITUDE_M = 3.0
    SPEED_MPS = 0.5
    CORNER_PAUSE_S = 1.0
    LANDING_SPEED_MPS = 0.6

    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        # Start trajectory tracker
        tracker = TrajectoryTracker()
        tracker.start()
        
        # Get start position
        start_state = client.getMultirotorState()
        start_pos = start_state.kinematics_estimated.position
        start_x, start_y, start_z = start_pos.x_val, start_pos.y_val, start_pos.z_val
        print(f"[INFO] Start Position: ({start_x:.2f}, {start_y:.2f}, {start_z:.2f})")

        # Enable API
        print("[INFO] Enabling API control...")
        client.enableApiControl(True)
        
        # Arm
        print("[INFO] Arming drone...")
        client.armDisarm(True)
        
        # Clear markers
        try:
            client.simFlushPersistentMarkers()
        except Exception:
            pass

        # Takeoff
        print("[INFO] Taking off...")
        client.takeoffAsync(timeout_sec=10).join()
        
        # Hover
        print("[INFO] Hovering (3s)...")
        client.hoverAsync().join()
        time.sleep(3)

        # Get position after takeoff
        takeoff_state = client.getMultirotorState()
        takeoff_pos = takeoff_state.kinematics_estimated.position
        print(f"[INFO] Position after takeoff: ({takeoff_pos.x_val:.2f}, {takeoff_pos.y_val:.2f}, {takeoff_pos.z_val:.2f})")

        # Target Z: Go up 3m from START (ground)
        # NED: negative Z is up.
        target_z = start_z - ALTITUDE_M
        
        # 1. Rise to Altitude vertically
        print(f"[INFO] Rising to {ALTITUDE_M}m altitude (Target Z={target_z:.2f})...")
        # Use current X,Y to ascend vertically without side movement
        client.moveToPositionAsync(takeoff_pos.x_val, takeoff_pos.y_val, target_z, 1.0).join()
        
        # Update position after ascent
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        print(f"[INFO] Reached altitude. Pos: ({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f})")
        
        # Waypoints (NED)
        # Use current position as the base for the square
        base_x = pos.x_val
        base_y = pos.y_val
        
        waypoints = [
            (base_x + SIDE_M, base_y, target_z, "Forward (Side 1)"),
            (base_x + SIDE_M, base_y + SIDE_M, target_z, "Right (Side 2)"),
            (base_x, base_y + SIDE_M, target_z, "Back (Side 3)"),
            (base_x, base_y, target_z, "Left (Side 4 - Return)"),
        ]

        print(f"[INFO] Flying square {SIDE_M:.0f}x{SIDE_M:.0f}m at Z={target_z:.2f}...")

        for i, (x, y, z, desc) in enumerate(waypoints, 1):
            print(f"[INFO] Step {i}/{len(waypoints)}: {desc}")
            
            # Simple move first
            client.moveToPositionAsync(
                float(x), float(y), float(z), float(SPEED_MPS), 
                timeout_sec=20
            ).join()

            state = client.getMultirotorState()
            pos = state.kinematics_estimated.position
            cur = [pos.x_val, pos.y_val, pos.z_val]
            
            print(f"       -> Reached pos: ({cur[0]:.2f}, {cur[1]:.2f}, {cur[2]:.2f})")
            
            time.sleep(CORNER_PAUSE_S)

        print("[INFO] Landing at starting point...")
        # Move back to ground X,Y (start_x, start_y) before landing? 
        # User said "return to starting point and lowering to the ground".
        # The square ends at (base_x, base_y), which is where we started the square (in the air).
        # Let's just land from there.
        client.landAsync().join()
        
        # Stop tracker
        tracker.stop()
        tracker.join()

        print("[SUCCESS] Flight test completed!")
        return True
        
    except Exception as e:
        print(f"[ERROR] Flight error: {e}")
        return False
    finally:
        if "client" in locals():
            try:
                client.armDisarm(False)
                client.enableApiControl(False)
            except Exception:
                pass


def main():
    """Main function."""
    print("\n" + "=" * 60)
    print("Colosseum/AirSim Python API Test")
    print("=" * 60 + "\n")
    
    # Test connection
    if not test_connection():
        sys.exit(1)

    # Run tests without prompt
    test_basic_flight()
    
    print("\n" + "=" * 60)
    print("Testing Completed")
    print("=" * 60)


if __name__ == "__main__":
    main()
