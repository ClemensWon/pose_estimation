import os
import json
import torch
import torch.nn as nn
from torchvision import transforms, models
from torch.utils.data import Dataset, DataLoader, random_split
from tqdm import tqdm
from PIL import Image
import numpy as np

class PoseEstimationModel(nn.Module):
    def __init__(self, feature_extractor, feature_dim=1280):
        super(PoseEstimationModel, self).__init__()
        self.feature_extractor = feature_extractor
        self.pool = nn.AdaptiveAvgPool2d((1, 1))
        self.fc = nn.Sequential(
            nn.Flatten(),
            nn.Linear(feature_dim, 128),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(128, 3)  # 3 for position
        )

    def forward(self, x):
        # x: (batch, channels, height, width)
        features = self.feature_extractor(x)  # Extract features: (batch, feature_dim, h, w)
        features = self.pool(features).view(features.size(0), -1)  # Global average pooling: (batch, feature_dim)
        output = self.fc(features)  # Fully connected layers
        return output