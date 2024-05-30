from omni.anim.people.scripts.character_behaviour import CharacterBehaviour
from .go_to import BetterGoTo
import omni.usd

class PeopleBehaviour(CharacterBehaviour):

    def __init__(self, prim_path):
        super().__init__(prim_path)

        self._goto = BetterGoTo(self._prim_path)
        self._goto.set_speed(0.5)
        self._goto.set_arrival_threshold(0.1)

    def assign_goal(self):

        target_path = self.prim_path.AppendPath()
        self.target_prim = self.stage.GetPrimAtPath(target_path)

    def read_commands_from_file(self):
        return [["GoTo", "3", "3", "0", "0"]]
    
    def get_command(self, command):

        if command[0] == "GoTo":
            return BetterGoTo(self.character, command, self.navigation_manager)
        
        return super().get_command(command)
    
    def on_update(self, current_time: float, delta_time: float):
        """
        Called on every update. Initializes character at start, publishes character positions and executes character commands.
        :param float current_time: current time in seconds.
        :param float delta_time: time elapsed since last update.
        """
        if self.character is None:
            if not self.init_character():
                return

        if self.avoidanceOn:
            self.navigation_manager.publish_character_positions(delta_time, 0.5)

        if self.commands:
            goal_position = omni.usd.utils.get_world_transform_matrix(self.target_prim).ExtractTranslation()
            new_goal = (goal_position[0], goal_position[1], goal_position[2], 0)
            self.execute_command(self.commands, delta_time)
            self.current_command.update_path(new_goal)