train_test_split = 0.9
train_val_split = 0.8
early_stopping_patience = 5
unfreeze_pretrained_epoch = 5
# pretrained_weights = torchvision.models.ResNet18_Weights.IMAGENET1K_V1
# optimizer = torch.optim.Adam
lr=1e-5
weight_decay=1e-5
# scheduler = torch.optim.lr_scheduler.StepLR
step_size=5
gamma=0.5
# loss = nn.MSELoss
loss_scale = 10
batch_size = 32
epochs = 70

