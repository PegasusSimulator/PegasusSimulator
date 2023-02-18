Create a Custom Backend
=======================

In this tutorial, you will learn how to create a custom ``control backend`` that receives the current state of the vehicle and
its sensors and computes the control inputs to give to the rotors of the drone. The control laws implemented in this tutorial
and the math behind them are well described in :cite:p:`Pinto2021`, :cite:p:`mellinger_kumar`.

In this tutorial we will endup with a simulation that does **NOT** use ``PX4-Autopilot`` , and instead will use a custom nonlinear
controller to make the vehicle follow a pre-determined trajectory, written in just a few lines of pure Python code.

.. note::

   The Control Backend API is what was used to implement the MAVLink/PX4 backend. This interface is very powerful as it not only allows
   the user to create custom communication protocols (such as ROS) for controlling the vehicle, as well as defining and testing new
   control algorithms directly in pure Python code. This is specially usefull for conducting MATLAB-like simulations,
   as we can implement and test algorithms directly in a 3D environment by writing a few lines of Python code without having to deal
   with `Mavlink`, `PX4` or `ROS` like in other simulators.

0. Preparation
--------------

This tutorial assumes that you have followed the :ref:`Your First Simulation` section first.

1. Code
-------

The tutorial corresponds to the ``4_python_single_vehicle.py`` example in the ``examples`` directory.

.. literalinclude:: ../../../examples/4_python_single_vehicle.py
   :language: python
   :emphasize-lines: 30-31,68-74
   :linenos:

The next code section corresponds to the ``nonlinear_controller.py`` in the ``examples/utils`` directory.


.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :emphasize-lines: 6,13-23,78-81,92-95,110-118,121-127,135-141,144-150
   :linenos:

2. Explanation
--------------

To start a simulation that will use our custom ``control backend`` , 

.. literalinclude:: ../../../examples/4_python_single_vehicle.py
   :language: python
   :lines: 30-31
   :linenos:
   :lineno-start: 30

.. literalinclude:: ../../../examples/4_python_single_vehicle.py
   :language: python
   :lines: 68-74
   :linenos:
   :lineno-start: 68

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 6
   :linenos:
   :lineno-start: 6

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 13-23
   :linenos:
   :lineno-start: 13

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 78-81
   :linenos:
   :lineno-start: 78

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 92-95
   :linenos:
   :lineno-start: 92

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 110-118
   :linenos:
   :lineno-start: 110

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 121-127
   :linenos:
   :lineno-start: 121

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 135-141
   :linenos:
   :lineno-start: 135

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 144-150
   :linenos:
   :lineno-start: 144

3. Execution
------------

Now let's run the Python script:

.. code:: bash

   ISAACSIM_PYTHON examples/4_python_single_vehicle.py