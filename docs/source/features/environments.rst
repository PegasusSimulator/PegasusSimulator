Environments
============

At the moment, we only provide in the :ref:`Params` API a dictionary named ``SIMULATION_ENVIRONMENTS``
which stores the path to ``Isaac Sim`` pre-made worlds. As we update this simulation framework, expect
the list of default simulation environments to grow.

List of provided simulation environments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. table::
    :widths: 25 17 

    +---------------------------+-------------------------+
    | World                     | Name                    |
    +===========================+=========================+
    | |default_environment|     | Default Environment     |
    +---------------------------+-------------------------+
    | |black_gridroom|          | Black Gridroom          |
    +---------------------------+-------------------------+
    |                           |                         |
    +---------------------------+-------------------------+
    |                           |                         |
    +---------------------------+-------------------------+
    |                           |                         |
    +---------------------------+-------------------------+
    |                           |                         |
    +---------------------------+-------------------------+
    |                           |                         |
    +---------------------------+-------------------------+

.. note::

    In this initial version it is not possible to spawn a custom 3D USD world using the Pegasus Simulator GUI. 
    If you use the Pegasus Simulator in extension mode and want to use your custom worlds, for now you need
    manually drag and drop the assets into the viewport like a cavemen 👌️. This is for sure a feature in the :ref:`Roadmap`.

.. Definition of the image alias
.. |default_environment| image:: /_static/worlds/Default\ Environment.png
.. |black_gridroom| image:: /_static/worlds/Black\ Gridroom.png