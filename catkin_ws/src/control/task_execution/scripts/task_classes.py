class Command:
    """abstract class representing a command"""
    def finished() -> bool:
        """indicates if the command can be considered as executed"""
    def execute() -> bool:
        """attempts to execute command
        returns a bool indicating if it succeeded"""
    def abort():
        """stops all movement"""

class PoseCommand(Command):
    """moves the arm to a requested pose (position + orientation of the end effector)"""
    def execute() -> bool:
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""

class StraightMoveCommand(Command):
    """moves the end effector in a straight line by a certain distance in a certain direction without affecting its orientation"""
    def execute() -> bool:
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""

class GripperManipulationCommand(Command):
    """opens/closes the gripper to a desired position"""
    def execute() -> bool:
        """publishes on /arm_control/joint_cmd topic for the motor controller"""


class Task:
    """abstract class representing a task"""
    def constructCommandChain():
        """constructs the chain of the commands that constitue the task"""
    def finished() -> bool:
        """indicates if task has finished"""
    def executeNextCommand() -> bool:
        """attempts to execute next command on the command chain
        returns a bool indicating if it succeeded"""
    def execute():
        """executes all commands"""
    def abort():
        """stops all movement"""


class PressButton(Task):
    def constructCommandChain():
        command_chain = [
            PoseCommand(),   # go at a predetermined position in front of the button with gripper facing towards it
            StraightMoveCommand(),   # go forward enough to press the button
            StraightMoveCommand()   # go backwards
        ]

class FlipSwitch(Task):
    def constructCommandChain():
        command_chain = [
            PoseCommand(),   # go at a predetermined position in front of the flip with gripper facing towards it
            StraightMoveCommand(),   # go forward enough to press the switch
            StraightMoveCommand()   # go backwards
        ]

class RotateSwitch(Task):

class PickUpCable(Task):

class PlugInCable(Task):

class PickUpProbe(Task):

class PlaceProbeOnContainer(Task):

class PickProbeFromContainer(Task):

class InsertProbeInSoil(Task):

class PickUpJumper(Task):

class PutJumperOnPins(Task):

class GoToHomePosition(Task):


class ManualMotion(Task):
    """special task allowing manual movement of the end effector (gripper) in a certain direction in a straight line with unchanging orientation"""
    def __init__(self, direction, velocity):
        """calls setNextGoal to set the first goal"""
        super().__init__()
    def setVelocity(velocity):
        """changes the velocity of the command"""
    def setNextGoal():
        """adds a StraightMoveCommand to the command chain that has a goal further in the direction of the task"""
