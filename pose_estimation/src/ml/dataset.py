import os
import json
import torch
from torch.utils.data import Dataset
from PIL import Image
from torchvision import transforms

class PoseEstimationDataset(Dataset):
    def __init__(self, image_dir, label_dir, transform=None):
        self.image_dir = image_dir
        self.label_dir = label_dir
        self.image_filenames = sorted(os.listdir(image_dir))
        self.transform = transform

    def __len__(self):
        return len(self.image_filenames)

    def __getitem__(self, idx):
        # Load image
        image_path = os.path.join(self.image_dir, self.image_filenames[idx])
        image = Image.open(image_path).convert("RGB")

        # Apply transformations
        if self.transform:
            image = self.transform(image)

        # Load label
        label_path = os.path.join(self.label_dir, self.image_filenames[idx].replace('.png', '.json'))
        with open(label_path, 'r') as f:
            label = json.load(f)
        translation = torch.tensor(label['translation'], dtype=torch.float32)  # [x, y, z]
        rotation = torch.tensor(label['rotation'], dtype=torch.float32)        # [qw, qx, qy, qz]

        # Combine translation and rotation
        pose = torch.cat([translation, rotation])  # [x, y, z, qw, qx, qy, qz]

        return image, pose
