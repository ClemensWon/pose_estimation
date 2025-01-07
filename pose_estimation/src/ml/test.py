import os
import torch
import torch.nn as nn
from torchvision import transforms, models
from torch.utils.data import DataLoader
from tqdm import tqdm
from dataset import PoseEstimationDataset
from model import PoseEstimationModel

# Load Data
images_dir = 'pose_estimation/dataset/saved_images'
annotations_file = 'pose_estimation/dataset/test_dataset.json'
transform = transforms.Compose([
    transforms.Resize((480, 640)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])
test_dataset = PoseEstimationDataset(images_dir, annotations_file, transform=transform)
test_loader = DataLoader(test_dataset, batch_size=32, shuffle=False)

# Load Model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
checkpoint_path = "pose_estimation/model/best_pose_estimation_model.pth"
feature_extractor = models.resnet18(pretrained=False)
feature_extractor = nn.Sequential(*list(feature_extractor.children())[:-2])
model = PoseEstimationModel(feature_extractor, feature_dim=512)
checkpoint = torch.load(checkpoint_path, map_location=device)
model.load_state_dict(checkpoint['model_state_dict'])
model = model.to(device)
model.eval()
print(f"Model loaded from {checkpoint_path}")

# Define Loss Function
criterion_position = nn.MSELoss()
loss_scale = 100

# Testing Loop
test_loss = 0.0
with torch.no_grad():
    with tqdm(test_loader, desc="Testing") as t:
        for images, positions in t:
            images, positions = images.to(device), positions.to(device)
            predictions = model(images)
            pos_pred = predictions
            pos_loss = criterion_position(pos_pred, positions)
            loss = pos_loss * loss_scale
            test_loss += loss.item()
            t.set_postfix(loss=loss.item())

print(f"Test Average Loss: {test_loss / len(test_loader):.4f}")

# Predict and Compare on a Single Item
with torch.no_grad():
    sample_idx = 0  # Select the first item from the test dataset
    sample_image, ground_truth_pos = test_dataset[sample_idx]
    sample_image = sample_image.unsqueeze(0).to(device)  # Add batch dimension
    prediction = model(sample_image)

    predicted_pos = prediction.cpu().numpy()

    print("Ground Truth Position:", ground_truth_pos.numpy())
    print("Predicted Position:", predicted_pos)