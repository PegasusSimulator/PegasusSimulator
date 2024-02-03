Vision
======

This sensor simulates VIO data by transforming the vehicle position and orientation to the PX4
coordinate system, adding random noise and bias. The resulting covariance has the principal diagonal
filled with the variance of the random variables = noise_density².

.. automodule:: pegasus.simulator.logic.sensors.vision
   :members:
   :undoc-members:
   :show-inheritance:
