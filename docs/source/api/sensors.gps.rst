GPS
===

In order to guarantee full compatibility with the PX4 navigation system, the projection from 
local to global coordinate system, i.e., latitude and longitude is performed by transforming 
the vehicle position, to the geographic coordinate system using the azymuthal equidistant 
projection in accordance with the World Geodetic System (WGS84) :cite:p:`azimuthal_projection`, 
:cite:p:`Snyder1987`.

.. automodule:: pegasus.simulator.logic.sensors.gps
   :members:
   :undoc-members:
   :show-inheritance:
