Create a Custom Controller
==========================

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
   :emphasize-lines: 30-31,68-75
   :linenos:

The next code section corresponds to the ``nonlinear_controller.py`` in the ``examples/utils`` directory.


.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :emphasize-lines: 18,24-44,114-117,121-124,143-151,154-160,168-174,177-183
   :linenos:

2. Explanation
--------------

To start a pre-programed simulation using a different control backend other than ``MAVLink`` we include our custom 
``control backend`` module written in pure Python. 

.. literalinclude:: ../../../examples/4_python_single_vehicle.py
   :language: python
   :lines: 30-31
   :linenos:
   :lineno-start: 30

We now create a multirotor configuration and set the multirotor backend to be our ``NonlinearController`` object. That's it! 
It is that simple! Instead of using the `MAVLink/PX4`` control we will use a pure Python controller written by us. The rest 
of the script is the same as in the previous tutorial. 

.. literalinclude:: ../../../examples/4_python_single_vehicle.py
   :language: python
   :lines: 68-75
   :linenos:
   :lineno-start: 68

Now, let's take a look on how the ``NonlinearControl`` class is structured and how you can build your own. We must start by
including the ``Backend`` class found in the ``pegasus.simulator.logic.backends`` API. To explore all the functionalities
offered by this class, check the :ref:`Backend` API reference page.

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 18
   :linenos:
   :lineno-start: 18

Next, we define a class that inherits the `Backend`` class. This class must implement a few methods to be compliant
with the ``Multirotor`` API. In this example we will implement an "hard-coded" nonlinear geometrical controller for the
multirotor to track a pre-defined trajectory.

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 24-44
   :linenos:
   :lineno-start: 24

The first method that should be implemented is ``start()`` . This method gets invoked by
the vehicle when the simulation starts. You should perform any initialization of your algorithm or communication protocol
here.

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 114-117
   :linenos:
   :lineno-start: 114

The ``stop()``  method gets invoked by
the vehicle when the simulation stops. You should perform any cleanup here.

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 121-124
   :linenos:
   :lineno-start: 121

The ``update_sensor(sensor_type: str, data)``  method gets invoked by the vehicle at every physics step iteration (it is used as a callback) for 
each sensor that generated output data. It receives as input a string which describes the type of sensor and the data produced by that sensor.
It is up to you to decide on how to parse the sensor data and use it. In this case we are not going to use the data produced
by the simulated sensors for our controller. Instead we will get the state of the vehicle directly from the simulation and 
use that to feedback into our control law.

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 143-151
   :linenos:
   :lineno-start: 143

.. note::

   You can take a look on how the ``MavlinkBackend`` is implemented to check on how to parse sensor data produced by IMU, GPS, etc. 
   A simple strategy is to use an if-statement to check for which sensor we are receive the data from and parse it accordingly, for example:

   .. code:: Python

      if sensor_type == "imu":
         # do something
      elif sensor_type == "gps":
         # do something else
      else:
         # some sensor we do not care about, so ignore

   Check the Sensors :ref:`API Reference` to check what type of data each sensor generates. For most cases, it will be a dictionary.

The ``update_state(state: State)`` method is called at every physics steps with the most up-to-date ``State`` of the vehicle. The state
object contains:

- **position** of the vehicle's body frame with respect to the inertial frame, expressed in the inertial frame;
- **attitude** of the vehicle's body frame with respect to the inertial frame, expressed in the inertial frame;
- **linear velocity** of the vehicle's body frame, with respect to the inertial frame, expressed in the inertial frame;
- **linear body-velocity** of the vehicle's body frame, with respect to the inertial frame, expressed in the vehicle's body frame;
- **angular velocity** of the vehicle's body frame, with respect to the inertial frame, expressed in the vehicle's body frame;
- **linear acceleration** of the vehicle's body frame, with respect to the inertial frame, expressed in the inertial frame.

All of this data is filled adopting a **East-North-Down (ENU)** convention for the Inertial Reference Frame and a **Front-Left-Up (FLU)** 
for the vehicle's body frame of reference. 

.. note::

   In case you need to adopt other conventions, the ``State`` class also provides methods to return the state of the vehicle in
   North-East-Down (NED) and Front-Right-Down (FRD) conventions, so you don't have to make those conversions manually. Check
   the :ref:`State` API reference page for more information.

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 154-160
   :linenos:
   :lineno-start: 154

The ``input_reference()`` method is called at every physics steps by the vehicle to retrieve from the controller a list of floats 
with the target angular velocities in [rad/s] to use as reference to apply to each vehicle rotor. The list of floats
returned by this method should have the same length as the number of rotors of the vehicle.

.. note::

   In order to have a general controller, you might not want to hard-code the number of rotors of the vehicle in the controller.
   To take this into account, the :ref:`Backend` object is initialized with a reference to the :ref:`Vehicle` object
   when this gets instantiated. Therefore, it will access to all of its attributes, such as the number of rotors and methods
   to convert torques and forces in the body-frame to target rotor angular velocities!

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 168-174
   :linenos:
   :lineno-start: 168


The ``update(dt: float)`` method gets invoked at every physics step by the vehicle. It receives as argument the amount of time
elapsed since the last update (in seconds). This is where you should define your controller/communication layer logic and 
update the list of reference angular velocities for the rotors.

.. literalinclude:: ../../../examples/utils/nonlinear_controller.py
   :language: python
   :lines: 177-183
   :linenos:
   :lineno-start: 177

3. Execution
------------

Now let's run the Python script:

.. code:: bash

   ISAACSIM_PYTHON examples/4_python_single_vehicle.py

This should open a stage with a blue ground-plane with an 3DR Iris vehicle model in it. The simulation should start playing automatically and the stage being rendered. 
The vehicle will automatically start flying and performing a very fast relay maneuvre. If you miss it, you can just hit the stop/play
button again to repeat it. 

Notice now, that unlike the previous tutorial, if you open ``QGroundControl`` you won't see the vehicle. This is normal, because
now we are not using the ``MAVLink`` control backend, but instead our custom ``NonlinearControl`` backend.