import os
import torch
import torch.nn as nn
from torchvision import transforms, models
from torch.utils.data import DataLoader
from tqdm import tqdm
from dataset import PoseEstimationDataset
from model import PoseEstimationModel
from PIL import Image

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
checkpoint_path = "pose_estimation/model/best_pose_estimation_model.pth"
feature_extractor = models.resnet18(pretrained=False)
feature_extractor = nn.Sequential(*list(feature_extractor.children())[:-2])
model = PoseEstimationModel(feature_extractor, feature_dim=512)
checkpoint = torch.load(checkpoint_path, map_location=device)
model.load_state_dict(checkpoint['model_state_dict'])
model = model.to(device)
model.eval()


def preprocess_image(image_path):
    '''
    Preprocess the input image to match the training pipeline.
    '''
    transform = transforms.Compose([
        transforms.Resize((480, 640)),  # Resize to match training
        transforms.ToTensor(),          # Convert to tensor
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),  # Normalize
    ])
    
    # Load and preprocess the image
    image = Image.open(image_path).convert("RGB")  # Ensure RGB
    return transform(image).unsqueeze(0)  # Add batch dimension


def predict(image_path):
    '''
    Predict the pose using the trained model
    '''
    image = preprocess_image(image_path)
    prediction = model(image)
    return prediction