import numpy as np
import torch

def evaluate_model(model, data_loader):
    model.eval()
    total_translation_error = 0.0
    total_orientation_error = 0.0

    with torch.no_grad():
        for images, labels in data_loader:
            images, labels = images.to(device), labels.to(device)

            predictions = model(images)
            pred_translation = predictions[:, :3]
            pred_rotation = predictions[:, 3:]
            true_translation = labels[:, :3]
            true_rotation = labels[:, 3:]

            # Calculate errors
            translation_error = torch.sqrt(torch.sum((pred_translation - true_translation) ** 2, dim=1)).mean()
            orientation_error = torch.acos(torch.clamp(torch.sum(pred_rotation * true_rotation, dim=1), -1.0, 1.0)).mean()

            total_translation_error += translation_error.item()
            total_orientation_error += orientation_error.item()

    print(f"Translation RMSE: {total_translation_error / len(data_loader):.4f}")
    print(f"Orientation Angular Error: {total_orientation_error / len(data_loader):.4f} radians")
