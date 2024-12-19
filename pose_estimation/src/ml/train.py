import torch
from torch.utils.data import DataLoader
from torchvision import transforms
from PIL import Image
from model import PoseEstimationModel
from dataset import PoseEstimationDataset, pose_loss

def pose_loss(predictions, targets, weight_position=1.0, weight_orientation=1.0):
    # Split predictions and targets into translation and rotation
    pred_translation = predictions[:, :3]
    pred_rotation = predictions[:, 3:]
    target_translation = targets[:, :3]
    target_rotation = targets[:, 3:]

    # Position loss (e.g., MSE)
    position_loss = torch.mean((pred_translation - target_translation) ** 2)

    # Orientation loss (quaternion loss)
    pred_rotation = pred_rotation / torch.norm(pred_rotation, dim=1, keepdim=True)  # Normalize quaternion
    target_rotation = target_rotation / torch.norm(target_rotation, dim=1, keepdim=True)
    orientation_loss = 1.0 - torch.sum(pred_rotation * target_rotation, dim=1)  # Dot product
    orientation_loss = torch.mean(orientation_loss)

    # Weighted sum of losses
    return weight_position * position_loss + weight_orientation * orientation_loss

# Initialize model, optimizer, and loss
model = PoseEstimationModel()
optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)

# Paths to dataset
image_dir = "dataset/images/"
label_dir = "dataset/labels/"

# Image preprocessing
transform = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # ImageNet normalization
])

# Create dataset and data loader
dataset = PoseEstimationDataset(image_dir=image_dir, label_dir=label_dir, transform=transform)
data_loader = DataLoader(dataset, batch_size=16, shuffle=True, num_workers=4)  # Batch size of 16

num_epochs = 100

# Training loop
for epoch in range(num_epochs):
    model.train()
    running_loss = 0.0
    
    for images, labels in data_loader:
        images, labels = images.to(device), labels.to(device)

        # Forward pass
        predictions = model(images)
        loss = pose_loss(predictions, labels)

        # Backward pass and optimization
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        running_loss += loss.item()

    print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {running_loss/len(data_loader):.4f}")
