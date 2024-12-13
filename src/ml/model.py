import torch.nn as nn
import torchvision.models as models

class PoseEstimationModel(nn.Module):
    def __init__(self):
        super(PoseEstimationModel, self).__init__()
        # Load pre-trained ResNet backbone
        self.backbone = models.resnet50(pretrained=True)
        
        # Replace the final layer with a custom regression head
        num_features = self.backbone.fc.in_features
        self.backbone.fc = nn.Linear(num_features, 7)  # 3 for translation, 4 for quaternion

    def forward(self, x):
        return self.backbone(x)
