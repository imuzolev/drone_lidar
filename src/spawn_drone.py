"""
Spawn a drone in ProjectAirSim simulation
"""
import pynng
import json
import struct
import time

def main():
    print("=" * 50)
    print("ProjectAirSim - Spawn Drone")
    print("=" * 50)
    
    services_url = "tcp://127.0.0.1:8990"
    
    print(f"\nConnecting to Services at {services_url}...")
    try:
        req_socket = pynng.Req0()
        req_socket.dial(services_url, block=True)
        req_socket.recv_timeout = 10000
        print("Connected!")
    except Exception as e:
        print(f"Connection failed: {e}")
        return
    
    # Try different service calls to understand the API
    
    # Method 1: Try plain method name
    test_methods = [
        "ping",
        "Ping", 
        "GetSimulatorState",
        "ListAssets",
        "ListObjects",
    ]
    
    for method in test_methods:
        print(f"\n--- Testing method: {method} ---")
        
        # Try different JSON formats
        formats = [
            # Format 1: Simple
            json.dumps({"method": method}),
            # Format 2: With id and version
            json.dumps({"id": 1, "method": method, "params": {}, "version": 1.0}),
            # Format 3: Just method name as string
            method,
        ]
        
        for i, fmt in enumerate(formats):
            try:
                print(f"  Format {i+1}: {fmt[:60]}...")
                req_socket.send(fmt.encode() if isinstance(fmt, str) else fmt)
                response = req_socket.recv()
                print(f"  Response: {response[:100]}")
            except pynng.Timeout:
                print(f"  Timeout")
            except Exception as e:
                print(f"  Error: {e}")
    
    # Try to spawn a drone
    print("\n--- Trying to spawn drone ---")
    spawn_request = {
        "id": 100,
        "method": "SpawnObject",
        "params": {
            "object_name": "Drone1",
            "object_type": "Quadrotor1",
            "pose": {"position": {"x": 0, "y": 0, "z": -5}, "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}},
            "scale": {"x": 1, "y": 1, "z": 1}
        },
        "version": 1.0
    }
    
    try:
        req_json = json.dumps(spawn_request)
        print(f"Sending: {req_json[:80]}...")
        req_socket.send(req_json.encode())
        response = req_socket.recv()
        print(f"Response: {response}")
    except Exception as e:
        print(f"Error: {e}")
    
    req_socket.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
