import carb
from threading import Lock

class KeyboardHandler:

    # Keyboard handler instance
    # We can only assume one keyboard, so let's make this a singleton!
    _instance = None

    # Lock for safe multi-threading
    _lock: Lock = Lock()

    # Movement keys
    up = False
    down = False
    left = False
    right = False

    def __init__(self):
        pass

    def __new__(cls):
        """[summary]

        Returns:
            VehicleManger: the single instance of the VehicleManager class 
        """

        # Use a lock in here to make sure we do not have a race condition
        # when using multi-threading and creating the first instance of the VehicleManager
        with cls._lock:
            if cls._instance is None:
                cls._instance = object.__new__(cls)
            else:
                carb.log_info("KeyboardHandler is defined already, returning the previously defined one")

            return KeyboardHandler._instance

    def __del__(self):
        """Destructor for the object"""
        KeyboardHandler._instance = None
        return

    def event_handler(self, event):

        # Check if the event results from a keyboard key being pressed
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:

            # Handle the WASD keys (typical for movement)
            if event.input == carb.input.KeyboardInput.W:
                KeyboardHandler.up = True
            if event.input == carb.input.KeyboardInput.S:
                KeyboardHandler.down = True
            if event.input == carb.input.KeyboardInput.A:
                KeyboardHandler.left = True
            if event.input == carb.input.KeyboardInput.D:
                KeyboardHandler.right = True

            # Handle the arrow keys
            if event.input == carb.input.KeyboardInput.UP:
                KeyboardHandler.up = True
            if event.input == carb.input.KeyboardInput.DOWN:
                KeyboardHandler.down = True
            if event.input == carb.input.KeyboardInput.LEFT:
                KeyboardHandler.left = True
            if event.input == carb.input.KeyboardInput.RIGHT:
                KeyboardHandler.right = True
    
    def reset(self):
        
        # Reset event for the movement of the vehicle
        KeyboardHandler.up = False
        KeyboardHandler.down = False
        KeyboardHandler.left = False
        KeyboardHandler.right = False