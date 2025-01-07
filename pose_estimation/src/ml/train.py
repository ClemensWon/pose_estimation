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
images_dir = 'catkin_ws\\src\\pose_estimation\\pose_estimation\\dataset\\saved_images'
annotations_file = 'catkin_ws\\src\\pose_estimation\\pose_estimation\\dataset\\train_dataset.json'

transform = transforms.Compose([
    transforms.Resize((480, 640)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

dataset = PoseEstimationDataset(images_dir, annotations_file, transform=transform)

# Split dataset into training, validation, and test sets
train_size = int(0.8 * len(dataset))
val_size = int(0.2 * len(dataset))
train_dataset, val_dataset = random_split(dataset, [train_size, val_size])

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

# Initialize variables for model checkpointing
best_val_loss = float('inf')
save_dir = "catkin_ws\\src\\pose_estimation\\pose_estimation\\model"
os.makedirs(save_dir, exist_ok=True)

# Training Loop
for epoch in range(10):
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
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
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

    # Checkpointing: Save the model if validation loss improves
    if val_loss_avg < best_val_loss:
        best_val_loss = val_loss_avg
        best_model_path = os.path.join(save_dir, "best_pose_estimation_model.pth")
        torch.save({
            'epoch': epoch + 1,
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'val_loss': best_val_loss
        }, best_model_path)
        print(f"New best model saved to {best_model_path} with Validation Loss: {best_val_loss:.4f}")