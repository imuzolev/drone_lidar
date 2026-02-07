import numpy as np
import cv2
from PIL import Image
from exceptions import SensorError

class SensorInput:
    """
    Manages the acquisition of data from various sensors installed on the drone, including cameras and LiDAR.
    """
    def __init__(self, camera_index=0, lidar_config=None, simulation_mode=True):
        self.simulation_mode = simulation_mode
        if not simulation_mode:
            self.camera = self.initialize_camera(camera_index)
            self.lidar_config = lidar_config
            self.initialize_lidar()
        else:
            self.camera = None
            self.lidar_config = lidar_config
            print("[SensorInput] Running in simulation mode")

    def initialize_camera(self, camera_index):
        """
        Initialize the camera using the provided index.
        """
        camera = cv2.VideoCapture(camera_index)
        if not camera.isOpened():
            raise SensorError(f"Failed to open camera at index {camera_index}")
        return camera

    def initialize_lidar(self):
        """
        Initialize LiDAR hardware using the provided configuration.
        """
        try:
            # Placeholder for LiDAR initialization code.
            # suppose this involves setting up connection parameters and calibration.
            print("LiDAR initialized with config:", self.lidar_config)
        except Exception as e:
            raise SensorError(f"Failed to initialize LiDAR with config {self.lidar_config}: {str(e)}")

    def get_camera_data(self):
        """
        Capture an image frame from the camera.
        """
        if self.simulation_mode:
            # Simulation: return a dummy image
            dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            return Image.fromarray(dummy_image)
        ret, frame = self.camera.read()
        if not ret:
            raise SensorError("Failed to read from camera")
        return Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    def get_lidar_data(self):
        """
        Simulate the retrieval of LiDAR data. Replace with actual data retrieval logic.
        """
        try:
            # Simulate 360-degree LiDAR data as an array of distances
            return np.random.rand(360) * 100  # Distances in meters
        except Exception as e:
            raise SensorError(f"Failed to retrieve LiDAR data: {str(e)}")

    def release_resources(self):
        """
        Release hardware resources properly to ensure a clean shutdown.
        """
        if not self.simulation_mode and self.camera:
            self.camera.release()
            cv2.destroyAllWindows()
        print("[SensorInput] Resources released")

# Example usage can be:
# try:
#     sensor_system = SensorInput(camera_index=0, lidar_config={'port': '/dev/ttyUSB0'})
#     camera_image = sensor_system.get_camera_data()
#     lidar_scan = sensor_system.get_lidar_data()
#     camera_image.show()  # Display the captured image using PIL
#     print("LiDAR Data:", lidar_scan)
# except SensorError as e:
#     print(f"Error during sensor operations: {e}")
