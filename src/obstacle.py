import numpy as np
import cv2
from PIL import Image
import torch
from torchvision import models, transforms
from exceptions import ObstacleDetectionError

class ObstacleDetector:
    """
    Uses computer vision and LiDAR data to detect and avoid obstacles in the drone's path.
    """
    def __init__(self, camera_model_path=None, lidar_model_path=None, simulation_mode=True):
        self.simulation_mode = simulation_mode
        if not simulation_mode:
            self.camera_model = self.load_model(camera_model_path, 'camera')
            self.lidar_model = self.load_model(lidar_model_path, 'lidar')
        else:
            self.camera_model = None
            self.lidar_model = None
            print("[ObstacleDetector] Running in simulation mode")
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def load_model(self, model_path, model_type):
        """
        Load a pre-trained deep learning model from the specified path.
        """
        try:
            model = models.resnet50(weights='IMAGENET1K_V1')
            if model_path:
                model.load_state_dict(torch.load(model_path))
            model.eval()
            return model
        except Exception as e:
            raise ObstacleDetectionError(f"Failed to load {model_type} model from {model_path}: {str(e)}")

    def detect_obstacles_camera(self, camera_image):
        """
        Detect obstacles using the camera model.
        """
        if self.simulation_mode:
            # Simulation: return random obstacle data
            num_obstacles = np.random.randint(0, 3)
            return [{"type": "obstacle", "distance": np.random.uniform(5, 50)} for _ in range(num_obstacles)]
        try:
            image = Image.open(camera_image).convert('RGB')
            image_tensor = self.transform(image).unsqueeze(0)
            with torch.no_grad():
                outputs = self.camera_model(image_tensor)
            detected_objects = self.process_outputs(outputs)
            return detected_objects
        except Exception as e:
            raise ObstacleDetectionError(f"Camera obstacle detection failed: {str(e)}")

    def detect_obstacles_lidar(self, lidar_data):
        """
        Process LiDAR data to detect obstacles.
        """
        try:
            lidar_tensor = torch.tensor(lidar_data, dtype=torch.float32).unsqueeze(0)
            with torch.no_grad():
                outputs = self.lidar_model(lidar_tensor)
            detected_objects = self.process_outputs(outputs)
            return detected_objects
        except Exception as e:
            raise ObstacleDetectionError(f"Lidar obstacle detection failed: {str(e)}")

    def process_outputs(self, outputs):
        """
        Process model outputs to extract obstacle information.
        """
        # Placeholder for output processing logic
        return outputs  # we can adjust based on actual output format

# Example usagecan be:
# try:
#     detector = ObstacleDetector('camera_model.pth', 'lidar_model.pth')
#     camera_obstacles = detector.detect_obstacles_camera('path_to_image.jpg')
#     lidar_obstacles = detector.detect_obstacles_lidar([0.1, 0.2, ..., 0.3])  # Example LiDAR data array
#     print("Camera Detected Obstacles:", camera_obstacles)
#     print("LiDAR Detected Obstacles:", lidar_obstacles)
# except ObstacleDetectionError as e:
#     print(f"Error during obstacle detection: {e}")
