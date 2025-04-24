Simulation with Moving People
=============================

In this tutorial, you will learn how to add moving people to your standalone simulations. This is useful for dynamic applications
such as target tracking, surveillance, and search and rescue missions.

.. image:: /_static/pegasus_people_example.png
            :width: 600px
            :align: center
            :alt: Simulating drones and people using Pegasus

.. note::

   Here we follow a different approach to simulate moving people than what is provided in the official NVIDIA Isaac Sim documentation. We believe
   that our API is simpler and more intuitive to use in most user cases. I also needed this for my own Ph.D. research :)

0. Preparation
--------------

This tutorial assumes that you have followed the :ref:`Your First Simulation` section first.

1. Code
-------

The tutorial corresponds to the ``9_people.py`` example in the ``examples`` directory.

.. literalinclude:: ../../../examples/9_people.py
   :language: python
   :emphasize-lines: 23-48,50-54,61-62, 69-90, 124-127, 129-135
   :linenos:

2. Explanation
--------------

To start a pre-programmed simulation with moving people, you need to ensure that the ``People`` extension provided by NVIDIA is enabled. Note, in Isaac 4.1.0 we also need to create a new stage for the people extension to start properly.

.. literalinclude:: ../../../examples/9_people.py
   :language: python
   :lines: 23-48, 50-54
   :linenos:
   :lineno-start: 23

We also need to import the ``Person`` and the ``PersonController`` classes. This follows the same strategy adopted for the vehicles and the control backends.

.. literalinclude:: ../../../examples/9_people.py
   :language: python
   :lines: 61-62
   :linenos:
   :lineno-start: 61

The ``PersonController`` is an interface class that a user defined controller must inherit from. This controller is responsible for defining the behavior of the person in the simulation.
In this example, our controller makes a person walk in circles, but you can define any behavior you want. For instance, you could:

   * read the keyboard input to move the person around the world.
   * read the target position from a ros topic,
   * read the target position from a file,
   * etc...

.. literalinclude:: ../../../examples/9_people.py
   :language: python
   :lines: 69-90
   :linenos:
   :lineno-start: 69

.. note::

   To check all the functions that you can implement in your controller, check the ``PersonController`` class in the API reference.

The next step is to create a person in the simulation. But, you let's imagine you don't know which 3D models are available. Well, you can call the static method ``Person.get_character_asset_list()`` function to list all the available models.

.. literalinclude:: ../../../examples/9_people.py
   :language: python
   :lines: 124-127
   :linenos:
   :lineno-start: 124

Now that you know which models you can load, you can create a person in the simulation. You can set the initial position, orientation, and the controller that will define the behavior of the person.
Note that if you just want to send a person to a given position manually (without using a controller), you can! Just check "Person 2".

.. literalinclude:: ../../../examples/9_people.py
   :language: python
   :lines: 129-135
   :linenos:
   :lineno-start: 129

3. Execution
------------

Now let's run the Python script:

.. code:: bash

   ISAACSIM_PYTHON examples/9_people.py

This should open a stage with a blue ground-plane with an 3DR Iris vehicle model in it. The simulation should start playing automatically and the stage being rendered. 
You will see 2 people being simulated with one of them walking in circles.

If you open ``QGroundControl`` you can control the vehicle.