"""
| File: rotations.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Implements utilitary rotations between ENU and NED inertial frame conventions and FLU and FRD body frame conventions.
"""
import numpy as np
from scipy.spatial.transform import Rotation

# Quaternion for rotation between ENU and NED INERTIAL frames
# NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
# ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
# This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
# Note: this quaternion follows the convention [qx, qy, qz, qw]
q_ENU_to_NED = np.array([0.70711, 0.70711, 0.0, 0.0])

# A scipy rotation from the ENU inertial frame to the NED inertial frame of reference
rot_ENU_to_NED = Rotation.from_quat(q_ENU_to_NED)

# Quaternion for rotation between body FLU and body FRD frames
# +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
# to Forward, Left, Up (base_link) frames and vice-versa.
# This rotation is symmetric, so q_FLU_to_FRD == q_FRD_to_FLU.
# Note: this quaternion follows the convention [qx, qy, qz, qw]
q_FLU_to_FRD = np.array([1.0, 0.0, 0.0, 0.0])

# A scipe rotation from the FLU body frame to the FRD body frame
rot_FLU_to_FRD = Rotation.from_quat(q_FLU_to_FRD)
