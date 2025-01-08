import os
import json
import torch
import torch.nn as nn
from torchvision import transforms, models
from torch.utils.data import Dataset, DataLoader, random_split
from tqdm import tqdm
from PIL import Image
import numpy as np
from dataset import PoseEstimationDataset
from model import PoseEstimationModel

# Load Data
images_dir = 'pose_estimation/dataset/saved_images'
annotations_file = 'pose_estimation/dataset/train_dataset.json'

# Define custom Gaussian noise augmentation
class AddGaussianNoise(object):
    def __init__(self, mean=0.0, std=0.05):
        self.mean = mean
        self.std = std

    def __call__(self, tensor):
        noise = torch.randn(tensor.size()) * self.std + self.mean
        return tensor + noise

    def __repr__(self):
        return f"{self.__class__.__name__}(mean={self.mean}, std={self.std})"

train_transform = transforms.Compose([
    transforms.Resize((480, 640)),
    transforms.ToTensor(),
#    AddGaussianNoise(mean=0.0, std=0.02),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

val_transform = transforms.Compose([
    transforms.Resize((480, 640)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

# Split dataset indices into training and validation
dataset = PoseEstimationDataset(images_dir, annotations_file)
dataset_size = len(dataset)
indices = list(range(dataset_size))
train_size = int(0.8 * dataset_size)
val_size = dataset_size - train_size
train_indices, val_indices = indices[:train_size], indices[train_size:]

train_dataset = PoseEstimationDataset(images_dir, annotations_file, indices=train_indices, transform=train_transform)
val_dataset = PoseEstimationDataset(images_dir, annotations_file, indices=val_indices, transform=val_transform)

# Create DataLoaders
train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)

# Load Model
feature_extractor = models.resnet18(pretrained=True)
feature_extractor = nn.Sequential(*list(feature_extractor.children())[:-2])
model = PoseEstimationModel(feature_extractor, feature_dim=512)

# Check for CUDA
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = model.to(device)

# Define Loss and Optimizer
criterion_position = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=5, gamma=0.1)
loss_scale = 100

# Early Stopping Parameters
patience = 5
no_improvement_count = 0

# Initialize variables for model checkpointing
best_val_loss = float('inf')
save_dir = "pose_estimation/model"
os.makedirs(save_dir, exist_ok=True)

# Training Loop
for epoch in range(50):
    model.train()
    epoch_loss = 0.0
    with tqdm(train_loader, desc=f"Epoch {epoch + 1} [Training]") as t:
        for images, positions in t:
            images, positions = images.to(device), positions.to(device)
            optimizer.zero_grad()
            predictions = model(images)
            pos_pred = predictions
            pos_loss = criterion_position(pos_pred, positions)
            loss = pos_loss * loss_scale
            loss.backward()
            # Gradient Clipping to avoid exploding gradients
            #torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            epoch_loss += loss.item()
            t.set_postfix(loss=loss.item())

    print(f"Epoch {epoch + 1}, Training Average Loss: {epoch_loss / len(train_loader):.4f}")

    # Validation Loop
    model.eval()
    val_loss = 0.0
    with torch.no_grad():
        with tqdm(val_loader, desc=f"Epoch {epoch + 1} [Validation]") as t:
            for images, positions in t:
                images, positions = images.to(device), positions.to(device)
                predictions = model(images)
                pos_pred = predictions
                pos_loss = criterion_position(pos_pred, positions)
                loss = pos_loss * loss_scale
                val_loss += loss.item()
                t.set_postfix(loss=loss.item())

    val_loss_avg = val_loss / len(val_loader)
    print(f"Epoch {epoch + 1}, Validation Average Loss: {val_loss_avg:.4f}")

    # Update Learning Rate Scheduler
    lr_scheduler.step(val_loss_avg)
    
    # Early Stopping and Checkpointing
    if val_loss_avg < best_val_loss:
        best_val_loss = val_loss_avg
        no_improvement_count = 0
        best_model_path = os.path.join(save_dir, "best_pose_estimation_model.pth")
        torch.save({
            'epoch': epoch + 1,
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'val_loss': best_val_loss
        }, best_model_path)
        print(f"New best model saved to {best_model_path} with Validation Loss: {best_val_loss:.4f}")
    else:
        no_improvement_count += 1
        print(f"No improvement for {no_improvement_count} epoch(s).")

    if no_improvement_count >= patience:
        print("Early stopping triggered.")
        break