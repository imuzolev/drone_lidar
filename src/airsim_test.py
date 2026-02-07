"""
Simple AirSim drone control script.
Connects to running AirSim simulation and performs basic flight operations.
"""
import airsim
import time

def main():
    print("Connecting to AirSim...")
    
    # Connect to AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("Connected!")
    
    # Get drone state
    state = client.getMultirotorState()
    print(f"Drone state: {state.kinematics_estimated.position}")
    
    # Enable API control
    client.enableApiControl(True)
    print("API control enabled")
    
    # Arm the drone
    client.armDisarm(True)
    print("Drone armed")
    
    # Take off
    print("Taking off...")
    client.takeoffAsync().join()
    print("Takeoff complete!")
    
    # Fly to a position (x, y, z in NED coordinates, z is negative for up)
    print("Flying to position...")
    client.moveToPositionAsync(-10, 10, -10, 5).join()
    print("Reached target position!")
    
    # Hover for 3 seconds
    print("Hovering...")
    time.sleep(3)
    
    # Return home
    print("Returning home...")
    client.moveToPositionAsync(0, 0, -10, 5).join()
    
    # Land
    print("Landing...")
    client.landAsync().join()
    print("Landed!")
    
    # Disarm
    client.armDisarm(False)
    client.enableApiControl(False)
    print("Mission complete!")

if __name__ == "__main__":
    main()
