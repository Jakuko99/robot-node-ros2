import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torchvision import datasets, transforms
import matplotlib.pyplot as plt
import numpy as np

BATCH_SIZE = 64
NUM_EPOCHS = 10
LEARNING_RATE = 0.001


class RobotSwarmOptimizerNetwork(nn.Module):
    """
    Network inputs:
    - Robot's current positions (x, y, theta)
    - Global map data merged from robots (occupancy grid) - dynamic size
    - Points of interest (unexplored areas, targets)

    Network outputs:
    - Next goal position (x, y)
    - Mapping priorities for unexplored areas
    """

    def __init__(self):
        super(RobotSwarmOptimizerNetwork, self).__init__()


class SwarmMapProcessor(nn.Module):
    """
    Network inputs:
    - Global map data (occupancy grid) - dynamic size for merging with local map
    - Local map data from individual robots (occupancy grid) - dynamic size
    - Robot's current positions (x, y, theta)

    Network outputs:
    - Merged global map data (occupancy grid) - dynamic size
    - Confidence scores for map areas
    """

    def __init__(self, in_channels=2, pose_dim=7):
        super(SwarmMapProcessor, self).__init__()
        # CNN for occupancy grids (2 channels, 500x500)
        # Encoder
        self.enc1 = nn.Sequential(
            nn.Conv2d(in_channels, 32, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 32, 3, padding=1),
            nn.ReLU(),
        )
        self.pool1 = nn.MaxPool2d(2)

        self.enc2 = nn.Sequential(
            nn.Conv2d(32, 64, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 64, 3, padding=1),
            nn.ReLU(),
        )
        self.pool2 = nn.MaxPool2d(2)

        self.enc3 = nn.Sequential(
            nn.Conv2d(64, 128, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(128, 128, 3, padding=1),
            nn.ReLU(),
        )

        # Pose encoder (7D -> embedding)
        self.pose_fc = nn.Sequential(
            nn.Linear(pose_dim, 64), nn.ReLU(), nn.Linear(64, 128), nn.ReLU()
        )

        # Bottleneck fusion
        self.fusion = nn.Conv2d(128 + 128, 128, 1)

        # Decoder
        self.up1 = nn.ConvTranspose2d(128, 64, 2, stride=2)
        self.dec1 = nn.Sequential(
            nn.Conv2d(128, 64, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 64, 3, padding=1),
            nn.ReLU(),
        )

        self.up2 = nn.ConvTranspose2d(64, 32, 2, stride=2)
        self.dec2 = nn.Sequential(
            nn.Conv2d(64, 32, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 32, 3, padding=1),
            nn.ReLU(),
        )

        # Final output layer (1 channel: merged occupancy grid)
        self.out_conv = nn.Conv2d(32, 1, 1)

    def forward(self, occ_grids, pose):
        # Encoder
        e1 = self.enc1(occ_grids)
        p1 = self.pool1(e1)
        e2 = self.enc2(p1)
        p2 = self.pool2(e2)
        e3 = self.enc3(p2)

        # Pose -> embedding
        pose_feat = self.pose_fc(pose)
        pose_feat = pose_feat.unsqueeze(-1).unsqueeze(-1)
        pose_feat = pose_feat.expand(-1, -1, e3.shape[2], e3.shape[3])  # broadcast

        # Fusion
        fused = torch.cat([e3, pose_feat], dim=1)
        fused = self.fusion(fused)

        # Decoder with skip connections
        u1 = self.up1(fused)
        d1 = self.dec1(torch.cat([u1, e2], dim=1))

        u2 = self.up2(d1)
        d2 = self.dec2(torch.cat([u2, e1], dim=1))

        out = self.out_conv(d2)
        return torch.sigmoid(out)


if __name__ == "__main__":
    model = SwarmMapProcessor()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)
    print(model)
