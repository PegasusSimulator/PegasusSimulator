from omni.anim.people.scripts.commands.goto import GoTo

class BetterGoTo(GoTo):

    def __init__(self, character, command, navigation_manager):
        super().__init__(character, command, navigation_manager)

    def update_path(self, new_command):
        self.navigation_manager.generate_goto_path(new_command)
