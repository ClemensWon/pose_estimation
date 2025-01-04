import os
import json
import torch
import torch.nn as nn
from torchvision import transforms, models
from torch.utils.data import Dataset, DataLoader, random_split
from tqdm import tqdm
from PIL import Image
import numpy as np

class SingleViewDataset(Dataset):
    def __init__(self, images_dir, annotations_file, transform=None):
        self.images_dir = images_dir
        self.transform = transform

        # Load annotations
        with open(annotations_file, 'r') as f:
            self.annotations = json.load(f)

    def __len__(self):
        return len(self.annotations)

    def __getitem__(self, idx):
        item = self.annotations[idx]

        # Image loading and transformation
        image_path = os.path.join(self.images_dir, f"{item['image_name']}.jpg")
        image = Image.open(image_path).convert('RGB')
        if self.transform:
            image = self.transform(image)

        # Extract position and orientation
        position = torch.tensor(item['camera_to_object']['translation'], dtype=torch.float32)
        orientation = torch.tensor(item['camera_to_object']['rotation'], dtype=torch.float32)

        return image, position, orientation

# Define Attention-Based Model
class AttentionSingleViewModel(nn.Module):
    def __init__(self, feature_extractor, feature_dim=512):
        super(AttentionSingleViewModel, self).__init__()
        self.feature_extractor = feature_extractor
        self.pool = nn.AdaptiveAvgPool2d((1, 1))
        self.fc = nn.Sequential(
            nn.Flatten(),
            nn.Linear(feature_dim, 128),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(128, 7)  # 3 for position + 4 for orientation
        )

    def forward(self, x):
        # x: (batch, channels, height, width)
        features = self.feature_extractor(x)  # Extract features: (batch, feature_dim, h, w)
        features = self.pool(features).view(features.size(0), -1)  # Global average pooling: (batch, feature_dim)
        output = self.fc(features)  # Fully connected layers
        return output

# Load Data
images_dir = 'catkin_ws\\src\\pose_estimation\\pose_estimation\\dataset\\saved_images'
annotations_file = 'catkin_ws\\src\\pose_estimation\\pose_estimation\\dataset\\dataset.json'

transform = transforms.Compose([
    transforms.Resize((480, 640)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

dataset = SingleViewDataset(images_dir, annotations_file, transform=transform)

# Split dataset into training, validation, and test sets
train_size = int(0.8 * len(dataset))
val_size = int(0.1 * len(dataset))
test_size = len(dataset) - train_size - val_size
train_dataset, val_dataset, test_dataset = random_split(dataset, [train_size, val_size, test_size])

train_loader = DataLoader(train_dataset, batch_size=4, shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=4, shuffle=False)
test_loader = DataLoader(test_dataset, batch_size=4, shuffle=False)

# Load Model
feature_extractor = models.resnet18(pretrained=True)
feature_extractor = nn.Sequential(*list(feature_extractor.children())[:-2])
model = AttentionSingleViewModel(feature_extractor, feature_dim=512)

# Check for CUDA
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = model.to(device)

# Define Loss and Optimizer
criterion_position = nn.MSELoss()
def angular_distance_loss(q1, q2):
    q1 = nn.functional.normalize(q1, dim=-1)
    q2 = nn.functional.normalize(q2, dim=-1)
    return torch.mean(1 - torch.sum(q1 * q2, dim=-1))

optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=5, gamma=0.1)

ori_weight = 2
# Training Loop
for epoch in range(20):
    model.train()
    epoch_loss = 0.0
    with tqdm(train_loader, desc=f"Epoch {epoch + 1} [Training]") as t:
        for images, positions, orientations in t:
            images, positions, orientations = images.to(device), positions.to(device), orientations.to(device)
            optimizer.zero_grad()
            predictions = model(images)
            pos_pred, ori_pred = predictions[:, :3], predictions[:, 3:]
            pos_loss = criterion_position(pos_pred, positions)
            ori_loss = angular_distance_loss(ori_pred, orientations)
            loss = pos_loss + ori_weight * ori_loss
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()
            t.set_postfix(loss=loss.item())

    lr_scheduler.step()
    print(f"Epoch {epoch + 1}, Training Average Loss: {epoch_loss / len(train_loader):.4f}")

    # Validation Loop
    model.eval()
    val_loss = 0.0
    with torch.no_grad():
        with tqdm(val_loader, desc=f"Epoch {epoch + 1} [Validation]") as t:
            for images, positions, orientations in t:
                images, positions, orientations = images.to(device), positions.to(device), orientations.to(device)
                predictions = model(images)
                pos_pred, ori_pred = predictions[:, :3], predictions[:, 3:]
                pos_loss = criterion_position(pos_pred, positions)
                ori_loss = angular_distance_loss(ori_pred, orientations)
                loss = pos_loss + ori_weight * ori_loss
                val_loss += loss.item()
                t.set_postfix(loss=loss.item())

    print(f"Epoch {epoch + 1}, Validation Average Loss: {val_loss / len(val_loader):.4f}")

# Testing Loop
model.eval()
test_loss = 0.0
with torch.no_grad():
    with tqdm(test_loader, desc="Testing") as t:
        for images, positions, orientations in t:
            images, positions, orientations = images.to(device), positions.to(device), orientations.to(device)
            predictions = model(images)
            pos_pred, ori_pred = predictions[:, :3], predictions[:, 3:]
            pos_loss = criterion_position(pos_pred, positions)
            ori_loss = angular_distance_loss(ori_pred, orientations)
            loss = pos_loss + ori_weight * ori_loss
            test_loss += loss.item()
            t.set_postfix(loss=loss.item())

print(f"Test Average Loss: {test_loss / len(test_loader):.4f}")

# Predict and Compare on a Single Item
model.eval()
with torch.no_grad():
    sample_idx = 0  # Select the first item from the test dataset
    sample_image, ground_truth_pos, ground_truth_ori = test_dataset[sample_idx]
    sample_image = sample_image.unsqueeze(0).to(device)  # Add batch dimension
    prediction = model(sample_image)

    predicted_pos = prediction[0, :3].cpu().numpy()
    predicted_ori = prediction[0, 3:].cpu().numpy()

    print("Ground Truth Position:", ground_truth_pos.numpy())
    print("Predicted Position:", predicted_pos)
    print("Ground Truth Orientation:", ground_truth_ori.numpy())
    print("Predicted Orientation:", predicted_ori)
