from pegasus_research.pegasus_isaac_ext.world.logic.state import State


class Vehicle:
    """
    A very general class that will be used to represent a vehicle type
    """

    def __init__(self):
        
        # Create an empty list of sensors that the vehicle contains
        self.sensors = []

    def add_sensor(self, sensor, name: str):

        # Save the sensor in the list
        # TODO: check in runtime if sensor inherits Sensor object
        if sensor is not None:
            self.sensors[name] = sensor

    def update(self):
        pass