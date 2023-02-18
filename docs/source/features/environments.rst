Environments
============

At the moment, we only provide in the :ref:`Params` API a dictionary named ``SIMULATION_ENVIRONMENTS``
which stores the path to ``Isaac Sim`` pre-made worlds. As we update this simulation framework, expect
the list of default simulation environments to grow.

List of provided simulation environments
----------------------------------------

.. table::
    :widths: 25 17 

    +----------------------------+--------------------------+
    | World                      | Name                     |
    +============================+==========================+
    | |default_environment|      | Default Environment      |
    +----------------------------+--------------------------+
    | |black_gridroom|           | Black Gridroom           |
    +----------------------------+--------------------------+
    | |curved_gridroom|          | Curved Gridroom          |
    +----------------------------+--------------------------+
    | |hospital|                 | Hospital                 |
    +----------------------------+--------------------------+
    | |office|                   | Office                   |
    +----------------------------+--------------------------+
    | |simple_room|              | Simple Room              |
    +----------------------------+--------------------------+
    | |warehouse|                | Warehouse                |
    +----------------------------+--------------------------+
    | |warehouse_with_forklifts| | Warehouse with Forklifts |
    +----------------------------+--------------------------+
    | |warehouse_with_shelves|   | Warehouse with Shelves   |
    +----------------------------+--------------------------+
    | |full_warehouse|           | Full Warehouse           |
    +----------------------------+--------------------------+
    | |flat_plane|               | Flat Plane               |
    +----------------------------+--------------------------+
    | |rought_plane|             | Rough Plane              |
    +----------------------------+--------------------------+
    | |slope_plane|              | Slope Plane              |
    +----------------------------+--------------------------+
    | |stairs_plane|             | Stairs Plane             |
    +----------------------------+--------------------------+

To spawn one of the provided environments when using the Pegasus Simulator
in standalone application mode, you can just add to your code:

.. code:: Python

    from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

    # Start the Pegasus Interface
    pg = PegasusInterface()

    # Load the environment
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

To index the dictionary of pre-made simulation environments, just use the names of the columns table.

.. note::

    In this initial version it is not possible to spawn a custom 3D USD world using the Pegasus Simulator GUI. 
    If you use the Pegasus Simulator in extension mode and want to use your custom worlds, for now you need
    manually drag and drop the assets into the viewport like a cavemen 👌️. This is for sure a feature in the :ref:`Roadmap`.

    However, when using the Pegasus Simulator in standalone application mode, i.e. Python scripting,
    you can load your own custom USD files using the ``load_environment(usd_path)`` method.

.. Definition of the image alias
.. |default_environment| image:: /_static/worlds/Default\ Environment.png
.. |black_gridroom| image:: /_static/worlds/Black\ Gridroom.png
.. |curved_gridroom| image:: /_static/worlds/Curved\ Gridroom.png
.. |hospital| image:: /_static/worlds/Hospital.png
.. |office| image:: /_static/worlds/Office.png
.. |simple_room| image:: /_static/worlds/Simple\ Room.png
.. |warehouse| image:: /_static/worlds/Warehouse.png
.. |warehouse_with_forklifts| image:: /_static/worlds/Warehouse\ with\ Forklifts.png
.. |warehouse_with_shelves| image:: /_static/worlds/Warehouse\ with\ Shelves.png
.. |full_warehouse| image:: /_static/worlds/Full\ Warehouse.png
.. |flat_plane| image:: /_static/worlds/Flat\ Plane.png
.. |rought_plane| image:: /_static/worlds/Rough\ Plane.png
.. |slope_plane| image:: /_static/worlds/Slope\ Plane.png
.. |stairs_plane| image:: /_static/worlds/Stairs\ Plane.png

Setting the Map Global Coordinates
----------------------------------

By default, the latitude, longitude and altitude of the origin of the simulated world
is set to the geographic coordinates of `Instituto Superior Técnico, Lisbon (Portugal)`, i.e.:

- **latitude=** 90.0 (º)
- **longitude=** 38.736832 (º)
- **altitude=** -9.137977 (m)

You can change the default coordinates by either:

1. Using the GUI of the Pegasus Simulator when operating in extension mode.

    .. image:: /_static/features/setting_geographic_coordinates.png
        :width: 600px
        :align: center
        :alt: Setting the geographic coordinates

2. Use the methods provided by :class:`PegasusInterface`, i.e:

    .. code:: Python

        from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

        # Start the Pegasus Interface
        pg = PegasusInterface()

        # Change only the global coordinates for this instance of the code
        # Future code runs will keep the same default coordinates
        pg.set_global_coordinates(latitude, longitude, altitude)

        # Change the default global coordinates for the simulator
        # This will be saved for future runs
        pg.set_new_global_coordinates(latitude, longitude, altitude) 