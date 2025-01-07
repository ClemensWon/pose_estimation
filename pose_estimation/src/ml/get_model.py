import os
import torch
import torch.nn as nn
from torchvision import transforms, models
from torch.utils.data import DataLoader
from tqdm import tqdm
from dataset import PoseEstimationDataset
from model import PoseEstimationModel

def get_model():
    '''
    Load the trained model for pose estimation
    Get prediction by calling model(image)
    '''
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    checkpoint_path = "pose_estimation/model/best_pose_estimation_model.pth"
    feature_extractor = models.resnet18(pretrained=False)
    feature_extractor = nn.Sequential(*list(feature_extractor.children())[:-2])
    model = PoseEstimationModel(feature_extractor, feature_dim=512)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    model.load_state_dict(checkpoint['model_state_dict'])
    model = model.to(device)
    model.eval()
    return model
