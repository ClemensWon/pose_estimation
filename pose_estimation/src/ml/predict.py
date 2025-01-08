import os
import sys
import torch
import torch.nn as nn
from torchvision import transforms, models
from PIL import Image

# Ensure imports always refer to the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from dataset import PoseEstimationDataset
from model import PoseEstimationModel

class PoseEstimator:
    def __init__(self, checkpoint_path, device=None):
        """
        Initializes the PoseEstimator class.
        
        Parameters:
        - checkpoint_path (str): Path to the trained model checkpoint.
        - device (torch.device): Device to run the model on (CPU or CUDA). Defaults to auto-detection.
        """
        self.device = device if device else torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.checkpoint_path = checkpoint_path
        
        # Check if checkpoint exists
        if not os.path.exists(self.checkpoint_path):
            raise FileNotFoundError(f"Checkpoint path does not exist: {self.checkpoint_path}")
        
        # Load the model
        self.model = self._load_model()

    def _load_model(self):
        """
        Loads the pose estimation model from the checkpoint.
        """
        feature_extractor = models.resnet18(pretrained=False)
        feature_extractor = nn.Sequential(*list(feature_extractor.children())[:-2])
        model = PoseEstimationModel(feature_extractor, feature_dim=512)

        # Load the checkpoint
        checkpoint = torch.load(self.checkpoint_path, map_location=self.device)
        model.load_state_dict(checkpoint['model_state_dict'])
        model = model.to(self.device)
        model.eval()

        return model

    def preprocess_image(self, image_path):
        """
        Preprocesses the input image to match the training pipeline.
        
        Parameters:
        - image_path (str): Path to the input image.
        
        Returns:
        - torch.Tensor: Preprocessed image tensor.
        """
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Image path does not exist: {image_path}")

        transform = transforms.Compose([
            transforms.Resize((480, 640)),  # Resize to match training
            transforms.ToTensor(),          # Convert to tensor
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),  # Normalize
        ])

        # Load and preprocess the image
        image = Image.open(image_path).convert("RGB")  # Ensure RGB
        return transform(image).unsqueeze(0)  # Add batch dimension

    def predict(self, image_path):
        """
        Predicts the pose for the given image.
        
        Parameters:
        - image_path (str): Path to the input image.
        
        Returns:
        - dict: Pose estimation results in the specified format.
        """
        image = self.preprocess_image(image_path).to(self.device)
        prediction = self.model(image)
        x, y, z = prediction[0].tolist()

        data_entry = {
            "image_name": "target_object",
            "object_id": "123",
            "object_type": "target",
            "camera_to_object": {
                "translation": [x, y, z],
                "rotation": [0, 0, 0, 0]
            }
        }

        return data_entry


