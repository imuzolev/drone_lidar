import torch
from torchvision import transforms
from PIL import Image
import os
from .decision_net import DecisionNet

class DecisionMaker:
    def __init__(self, model_path=None, simulation_mode=True):
        """
        Initialize the DecisionMaker with a model.

        Args:
        model_path (str): Optional path to a pretrained model. If not provided,
                          the model will be initialized with random weights for testing.
        simulation_mode (bool): If True, use simple simulation logic instead of model.
        """
        self.simulation_mode = simulation_mode
        
        if simulation_mode:
            self.model = None
            self.device = None
            print("[DecisionMaker] Running in simulation mode")
            return
            
        # Determine if CUDA is available and set the appropriate device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Initialize the DecisionNet model and transfer it to the designated device
        self.model = DecisionNet().to(self.device)
        
        # Load a pretrained model if a path is provided
        if model_path:
            try:
                self.model.load_state_dict(torch.load(model_path, map_location=self.device))
                self.model.eval()  # Set the model to evaluation mode
            except FileNotFoundError:
                print("Model file not found. Please check the path and try again.")
                raise
        else:
            # Initialize model with random weights for testing if no model path is provided
            self.model.load_state_dict({k: torch.rand(*v.size()) for k, v in self.model.state_dict().items()})
        
        # Define the image transformations
        self.transform = transforms.Compose([
            transforms.Resize((64, 64)),  # Resize the input images to 64x64
            transforms.ToTensor(),  # Convert images to PyTorch tensors
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # Normalize the images
        ])

    def make_decision(self, image_data, sensor_data):
        """
        Make a decision based on the input image and sensor data.

        Args:
        image_data: Path to the input image file or PIL Image object.
        sensor_data (list): Sensor data inputs as a list of numerical values.

        Returns:
        int: The decision index predicted by the model.
        """
        if self.simulation_mode:
            # Simulation: return random decision
            # 0: forward, 1: left, 2: right, 3: up, 4: down, 5: hover
            import random
            decisions = ["forward", "left", "right", "up", "down", "hover"]
            return random.choice(decisions)
        
        # Check if the image file exists
        if isinstance(image_data, str):
            if not os.path.exists(image_data):
                raise FileNotFoundError(f"The specified image path {image_data} does not exist.")
            image = Image.open(image_data).convert('RGB')
        else:
            image = image_data.convert('RGB')
        
        # Open the image, convert it to RGB, apply transformations, and move it to the device
        image = self.transform(image).unsqueeze(0).to(self.device)
        
        # Convert sensor data to a PyTorch tensor and move it to the device
        sensor_data = torch.tensor(sensor_data, dtype=torch.float).unsqueeze(0).to(self.device)
        
        # Make a prediction with the model
        with torch.no_grad():
            outputs = self.model(image, sensor_data)
            _, predicted = torch.max(outputs, 1)
            action = predicted.item()
        
        return action
