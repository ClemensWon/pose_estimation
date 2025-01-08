import os
import json
import torch
import torch.nn as nn
from torchvision import transforms, models
from torch.utils.data import Dataset, DataLoader, random_split
from tqdm import tqdm
from PIL import Image
import numpy as np

class PoseEstimationDataset(Dataset):
    def __init__(self, images_dir, annotations_file, indices=None, transform=None):
        self.images_dir = images_dir
        self.transform = transform

        # Load annotations
        with open(annotations_file, 'r') as f:
            self.annotations = json.load(f)

        # If indices are provided, filter the annotations
        if indices is not None:
            self.annotations = [self.annotations[i] for i in indices]

    def __len__(self):
        return len(self.annotations)

    def __getitem__(self, idx):
        item = self.annotations[idx]

        # Image loading and transformation
        image_path = os.path.join(self.images_dir, f"{item['image_name']}.jpg")
        image = Image.open(image_path).convert('RGB')
        if self.transform:
            image = self.transform(image)

        # Extract position
        position = torch.tensor(item['camera_to_object']['translation'], dtype=torch.float32)

        return image, position
