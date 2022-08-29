import sys
import rospy
import time
import math
import copy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from task_execution.srv import PoseGoal, JointGoal
import task_execution.quaternion_arithmetic as qa


class Command:
    """abstract class representing a command"""
    def finished(self):
        """indicates if the command can be considered as executed"""
    def execute(self):
        """attempts to execute command
        returns a bool indicating if it succeeded"""
    def abort(self):
        """stops all movement"""

class PoseCommand(Command):
    """moves the arm to a requested pose (position + orientation of the end effector)"""
    def __init__(self, pose=None, cartesian=False):
        self.pose = pose    # geometry_msgs.msg.Pose
        self.cartesian = cartesian
        self.finished = False

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        rospy.wait_for_service('/arm_control/pose_goal')
        try:
            proxy = rospy.ServiceProxy('/arm_control/pose_goal', PoseGoal)
            cmd_id = 0  # TODO: increment id at each command
            resp = proxy(cmd_id, self.pose, self.cartesian, False, False, "aaaaaaa")
            self.finished = resp.ok
        except rospy.ServiceException as e:
            # TODO: handle the exception (maybe)
            print("Service call failed: %s"%e)


class StraightMoveCommand(Command):
    """moves the end effector in a straight line by a certain distance in a certain direction without affecting its orientation"""
    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""

class GripperManipulationCommand(Command):
    """opens/closes the gripper to a desired position"""
    def execute(self):
        """publishes on /arm_control/joint_cmd topic for the motor controller"""


class Task(object):
    """abstract class representing a task"""
    def __init__(self):
        self.cmd_counter = 0
        self.command_chain = []
        self.constructCommandChain()
        self.aborted = False
        self.pause_time = 0

    def constructCommandChain(self):
        """constructs the chain of the commands that constitue the task"""

    def finished(self):
        """indicates if task has finished"""
    
    def update_world(self):
        """update the objects in the world"""

    def setupNextCommand(self):
        """gives the required information to the next command"""

    def currentCommandValidated(self):
        """indicates after the request of execution of the command if the outcome is satisfactory and the next command can begin"""
        return True

    def stopCondition(self):
        """indicates if task should be stopped because a command can't be executed"""
        return False

    def executeNextCommand(self):
        """attempts to execute next command on the command chain
        returns a bool indicating if it succeeded"""
        self.setupNextCommand()
        self.command_chain[self.cmd_counter].execute()
        if self.stopCondition():
            return False
        while not self.currentCommandValidated():
            self.setupNextCommand()
            self.command_chain[self.cmd_counter].execute()
            if self.stopCondition():
                return False
        self.cmd_counter += 1
        return True
        
    def execute(self):
        """executes all commands"""
        for _ in range(len(self.command_chain)):
            self.executeNextCommand()
            time.sleep(self.pause_time)
    
    def abort(self):
        """stops all movement"""


class PressButton(Task):
    def __init__(self, btn_pose):
        super(PressButton, self).__init__()
        self.btn_pose = btn_pose
        self.press_distance = 0.1
        self.pause_time = 2
    
    def currentCommand(self):
        return self.command_chain[self.cmd_counter]

    def getPressPosition(self):
        p = qa.point_image([0, 0, 1], self.btn_pose.orientation)
        d = 0.045*0
        if self.cmd_counter != 1:
            d += self.press_distance
        p = qa.mul(d, p)
        p = qa.mul(-1, p)   # TODO: direction is reversed for some reason
        res = qa.quat_to_point(qa.add(self.btn_pose.position, p))
        print("p is " + str(qa.quat_to_point(p)))
        print("btn position is " + str(self.btn_pose.position))
        print("result is " + str(res))
        print()
        return res
    
    def getPressOrientation(self):
        # for now
        q = qa.quat([0,0,1], math.pi)
        return qa.mul(self.btn_pose.orientation, q)

    def setupNextCommand(self):
        cmd = self.currentCommand()
        if self.cmd_counter == 0:
            cmd.pose = geometry_msgs.msg.Pose()
            cmd.pose.position = self.getPressPosition()
            cmd.pose.orientation = self.getPressOrientation()
        elif self.cmd_counter == 1:
            cmd.pose = geometry_msgs.msg.Pose()
            cmd.pose.orientation = self.getPressOrientation()
            cmd.pose.position = self.getPressPosition()
            cmd.cartesian = True
        elif self.cmd_counter == 2:
            cmd.pose = geometry_msgs.msg.Pose()
            cmd.pose.position = self.getPressPosition()
            cmd.pose.orientation = self.getPressOrientation()
            cmd.cartesian = True

    def constructCommandChain(self):
        self.command_chain = [
            PoseCommand(),
            PoseCommand(),]
        #    PoseCommand()
        #]
        """self.command_chain = [
            PoseCommand(),   # go at a predetermined position in front of the button with gripper facing towards it
            StraightMoveCommand(),   # go forward enough to press the button
            StraightMoveCommand()   # go backwards
        ]"""

class FlipSwitch(Task):
    def constructCommandChain(self):
        command_chain = [
            PoseCommand(),   # go at a predetermined position in front of the flip with gripper facing towards it
            StraightMoveCommand(),   # go forward enough to press the switch
            StraightMoveCommand()   # go backwards
        ]

"""class RotateSwitch(Task):

class PickUpCable(Task):

class PlugInCable(Task):

class PickUpProbe(Task):

class PlaceProbeOnContainer(Task):

class PickProbeFromContainer(Task):

class InsertProbeInSoil(Task):

class PickUpJumper(Task):

class PutJumperOnPins(Task):

class GoToHomePosition(Task):"""


class ManualMotion(Task):
    """special task allowing manual movement of the end effector (gripper) in a certain direction in a straight line with unchanging orientation"""
    def __init__(self, direction, velocity):
        """calls setNextGoal to set the first goal"""
        super().__init__()
    def setVelocity(self, velocity):
        """changes the velocity of the command"""
    def setNextGoal(self):
        """adds a StraightMoveCommand to the command chain that has a goal further in the direction of the task"""

