#!/usr/bin/env python

import numpy as np
from ..state import State
from scipy.signal import butter, filtfilt

class Thruster:

    def __init__(self):
        
        # Thruster constants
        self._rotor_drag_coefficient: float = 0.0
        self._rolling_moment_coefficient: float = 0.0
        self._max_rot_velocity: float = 0.0
        self._motor_constant: float = 0.0
        self._moment_constant: float = 0.0
        self._time_constant_up: float = 0.0
        self._time_constant_down: float = 0.0
        self._rotor_velocity_slowdown_sim: float = 0.0
        
        # Low pass filter for the thrusters

    def update(self, state: State, dt: float) -> dict[str, float | np.ndarray]:
        
        return None