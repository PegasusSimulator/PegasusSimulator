"""
| File: rotations.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Implements utilitary rotations between ENU and NED inertial frame conventions and FLU and FRD body frame conventions.
"""
#import numpy as np
import torch

import pytorch3d.transforms as transforms

# Quaternion for rotation between ENU and NED INERTIAL frames
# NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
# ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
# This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
# Note: this quaternion follows the convention [qw, qx, qy, qz]
# q_ENU_to_NED -> [0.0, 0.70711, 0.70711, 0.0]

# Rotation from the ENU inertial frame to the NED inertial frame of reference
def rot_ENU_to_NED(device, dtype):
    q_ENU_to_NED = torch.tensor([0.0, 0.70711, 0.70711, 0.0], dtype=torch.float32, device=device)
    return transforms.quaternion_to_matrix(q_ENU_to_NED)

# Quaternion for rotation between body FLU and body FRD frames
# +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
# to Forward, Left, Up (base_link) frames and vice-versa.
# This rotation is symmetric, so q_FLU_to_FRD == q_FRD_to_FLU.
# Note: this quaternion follows the convention [qw, qx, qy, qz]
# q_FLU_to_FRD -> [0.0, 1.0, 0.0, 0.0]

# Rotation from the FLU body frame to the FRD body frame
def rot_FLU_to_FRD(device, dtype):
    q_FLU_to_FRD = torch.tensor([0.0, 1.0, 0.0, 0.0], dtype=torch.float32, device=device)
    return transforms.quaternion_to_matrix(q_FLU_to_FRD)

