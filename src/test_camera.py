"""
Camera test for Colosseum/AirSim.
Run: python src/test_camera.py
"""

import airsim
import sys

def test_camera():
    """Camera test."""
    print("\n" + "=" * 60)
    print("Camera Test")
    print("=" * 60)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        # Get image
        print("[INFO] Getting image from front camera...")
        responses = client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
        ])
        
        if responses and len(responses) > 0:
            response = responses[0]
            print(f"[SUCCESS] Image received!")
            print(f"  - Size: {response.width}x{response.height}")
            print(f"  - Data: {len(response.image_data_uint8)} bytes")
            return True
        else:
            print("[WARNING] No image received")
            return False
            
    except Exception as e:
        print(f"[ERROR] Camera error: {e}")
        return False

if __name__ == "__main__":
    test_camera()
