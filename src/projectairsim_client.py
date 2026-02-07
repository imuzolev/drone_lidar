"""
ProjectAirSim Python Client
Connects to ProjectAirSim using NNG protocol (ports 8989/8990)
"""
import pynng
import json
import time

class ProjectAirSimClient:
    def __init__(self, topics_port=8989, services_port=8990, host="127.0.0.1"):
        self.topics_url = f"tcp://{host}:{topics_port}"
        self.services_url = f"tcp://{host}:{services_port}"
        self.sub_socket = None
        self.req_socket = None
        
    def connect(self):
        """Connect to ProjectAirSim server"""
        print(f"Connecting to Topics at {self.topics_url}...")
        try:
            # Subscribe socket for topics
            self.sub_socket = pynng.Sub0()
            self.sub_socket.subscribe(b"")  # Subscribe to all topics
            self.sub_socket.dial(self.topics_url, block=True)
            print("Topics connected!")
        except Exception as e:
            print(f"Topics connection failed: {e}")
            
        print(f"Connecting to Services at {self.services_url}...")
        try:
            # Request socket for services
            self.req_socket = pynng.Req0()
            self.req_socket.dial(self.services_url, block=True)
            print("Services connected!")
        except Exception as e:
            print(f"Services connection failed: {e}")
            
        return self.sub_socket is not None or self.req_socket is not None
    
    def call_service(self, method, params=None):
        """Call a service method"""
        if not self.req_socket:
            print("Services not connected!")
            return None
        
        self.request_id = getattr(self, 'request_id', 0) + 1
        
        # ProjectAirSim request format: id, method, params, version
        request = {
            "id": self.request_id,
            "method": method,
            "params": params or {},
            "version": 1.0
        }
        
        try:
            request_json = json.dumps(request)
            print(f"  Sending: {request_json}")
            self.req_socket.send(request_json.encode())
            self.req_socket.recv_timeout = 5000  # 5 second timeout
            response = self.req_socket.recv()
            print(f"  Received: {len(response)} bytes")
            # Try to decode as JSON, otherwise return raw
            try:
                return json.loads(response.decode())
            except:
                return response
        except pynng.Timeout:
            print(f"Service call timed out")
            return None
        except Exception as e:
            print(f"Service call failed: {e}")
            return None
    
    def receive_topic(self, timeout_ms=1000):
        """Receive a topic message"""
        if not self.sub_socket:
            print("Topics not connected!")
            return None
            
        try:
            self.sub_socket.recv_timeout = timeout_ms
            msg = self.sub_socket.recv()
            return msg
        except pynng.Timeout:
            return None
        except Exception as e:
            print(f"Topic receive failed: {e}")
            return None
    
    def ping(self):
        """Test connection with ping"""
        result = self.call_service("ping")
        return result is not None
    
    def list_objects(self):
        """List all objects in scene"""
        return self.call_service("ListObjects")
    
    def close(self):
        """Close connections"""
        if self.sub_socket:
            self.sub_socket.close()
        if self.req_socket:
            self.req_socket.close()
        print("Connections closed.")


def main():
    print("=" * 50)
    print("ProjectAirSim Python Client Test")
    print("=" * 50)
    
    client = ProjectAirSimClient()
    
    if client.connect():
        print("\nConnection successful!")
        
        # Try to receive some topic data
        print("\nListening for topic messages (5 seconds)...")
        start_time = time.time()
        message_count = 0
        
        while time.time() - start_time < 5:
            msg = client.receive_topic(timeout_ms=500)
            if msg:
                message_count += 1
                print(f"  Received message #{message_count}: {len(msg)} bytes")
                if message_count >= 10:
                    break
        
        print(f"\nReceived {message_count} messages in 5 seconds")
        
        # Try service call
        print("\nTrying to list objects...")
        objects = client.list_objects()
        if objects:
            print(f"Objects: {objects}")
        else:
            print("No response from ListObjects service")
        
        client.close()
    else:
        print("Failed to connect to ProjectAirSim!")
        print("Make sure the simulation is running (Press Play in Unreal Editor)")


if __name__ == "__main__":
    main()
