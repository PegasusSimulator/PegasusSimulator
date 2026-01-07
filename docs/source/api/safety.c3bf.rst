C3BF-Based Safe Quadrotor Control
=================================


Overview
--------
This module introduces a Collision Cone Control Barrier Function (C3BF) as a
velocity-level safety filter for quadrotor control. The C3BF enforces collision
avoidance constraints while preserving the nominal behavior of the nonlinear
geometric controller.

The safety filter operates by minimally modifying the commanded velocity to
maintain forward invariance of a safe set defined by obstacle geometry.

Motivation
----------
The baseline nonlinear controller provides accurate tracking of aggressive
minimum-snap trajectories but does not explicitly reason about safety. The C3BF
layer augments the controller with real-time collision avoidance without
requiring trajectory replanning or modification of the underlying control law.

Key Features
------------
- Velocity-level safety filtering
- Collision cone formulation for spherical obstacles
- Closed-form projection for single constraints
- Modular integration with existing controllers
- Negligible computational overhead

Implemented Variants
--------------------
The following C3BF variants are provided:

- **Projection-based C3BF**: closed-form velocity projection for single obstacles
- **Spherical obstacle C3BF**: geometry-aware constraint using obstacle radius

Limitations
-----------
The current implementation assumes static spherical obstacles and applies
constraints independently. Multiple simultaneous constraints are handled
conservatively via sequential projection.

Future extensions include full QP-based solvers and dynamic obstacle handling.


API Reference
-------------
.. automodule:: PegasusSimulator.examples.utils.c3bf
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: PegasusSimulator.examples.utils.nonlinear_controller_with_c3bf
   :members:
   :undoc-members:
   :show-inheritance:
