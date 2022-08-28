#!/usr/bin/env python

import sys
import copy
import time
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import task_execution.msg
from task_execution.srv import PoseGoal, PoseGoalResponse, JointGoal, JointGoalResponse


SUCCESS = 5
FAIL = 17
MOTOR_COUNT = 7


class Pose(geometry_msgs.msg.Pose):
    def __init__(self, deprecated=False):
        super().__init__(self)
        self._deprecated = deprecated
    
    def deprecated(self):
        return self._deprecated

    def forget(self):
        self._deprecated = True


class JointState:
    pass


class Planner:
    """recieves goals to reach and sends information to MoveIt in order for it to compute a trajectory"""
    NONE = 0
    JOINT_GOAL = 1
    POSE_GOAL = 2
    CARTESIAN_GOAL = 3

    def __init__(self):
        # initialize ROS ===============================================================================================
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("trajectory_planner_node", anonymous=True)

        # publishers ===================================================================================================
        self.end_of_mvt_pub = rospy.Publisher("/arm_control/end_of_movement", task_execution.msg.cmdOutcome, queue_size=5)
        self.display_trajectory_pub = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # subscribers ==================================================================================================
        rospy.Service("/arm_control/pose_goal", PoseGoal, self.handle_pose_goal)
        rospy.Service("/arm_control/joint_goal", JointGoal, self.handle_joint_goal)
        rospy.Subscriber("/arm_control/world_update", geometry_msgs.msg.Pose, self.object_callback)
        rospy.Subscriber("/arm_control/joint_telemetry", sensor_msgs.msg.JointState, self.telemetry_callback)

        # objects from MoveIt Commander needed to communicate with MoveIt ==============================================
        group_name = "astra_arm"    # the planning group of the arm
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        # ==============================================================================================================
        self.actual_joint_positions = [0.0]*MOTOR_COUNT
        self.moving = False
        self.current_goal_type = Planner.NONE
        self.current_goal = None

        self.objects = []

        self.set_max_velocity_and_acceleration(1, 1)

    def get_end_effector_pose(self):
        """
        TODO
        pose with respect to base the arm
        """
        pose = geometry_msgs.msg.Pose()
        # TODO
        return pose

    def set_max_velocity_and_acceleration(self, vel=0.1, acc=0.1):
        """
        Sets maximal velocity and acceleration scaling factors
        Allowed values are in (0,1].
        The default value is set in the joint_limits.yaml of the robot_config package
        """
        self.move_group.set_max_velocity_scaling_factor(vel)
        self.move_group.set_max_acceleration_scaling_factor(acc)

    def quaternion(self, axis, angle):
        orientation = geometry_msgs.msg.Quaternion()
        axis = self.normalize(axis)
        orientation.w = math.cos(angle/2)
        orientation.x = axis[0]*math.sin(angle/2)
        orientation.y = axis[1]*math.sin(angle/2)
        orientation.z = axis[2]*math.sin(angle/2)
        return orientation

    def normalize(self, axis):
        n = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
        axis = (axis[0]/n, axis[1]/2, axis[2]/n)
        return axis
        
    def handle_pose_goal(self, req):
        """
        Listens to /arm_control/pose_goal topic
        """
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        if self.moving:
            # TODO: send message to indicate non execution
            return PoseGoalResponse(False)
        goal_type = Planner.CARTESIAN_GOAL if req.cartesian else Planner.POSE_GOAL
        goal = req.goal
        """x = req.aaaa
        goal = copy.deepcopy(self.move_group.get_current_pose().pose)
        goal.orientation = req.goal.orientation
        if x == "x":
            goal.orientation.x = req.goal.orientation.x
            rospy.logwarn("xxxxxxxxxxxxxxxxx :       " + str(goal.orientation.x))
        elif x == "y":
            goal.orientation.y = req.goal.orientation.y
            rospy.logwarn("yyyyyyyyyyyyyyyyy :       " + str(goal.orientation.y))
        elif x == "z":
            goal.orientation.z = req.goal.orientation.z
            rospy.logwarn("zzzzzzzzzzzzzzzzz :       " + str(goal.orientation.z))
        elif x == "w":
            goal.orientation.w = req.goal.orientation.w
            rospy.logwarn("wwwwwwwwwwwwwwwww :       " + str(goal.orientation.w))"""
        self.achieve_goal(req.id, goal, goal_type)
        return PoseGoalResponse(True)

    def handle_joint_goal(self, req):
        """
        Listens to /arm_control/joint_goal topic.
        """
        if self.moving:
            # TODO: send message to indicate non execution
            return JointGoalResponse(False)
        self.achieve_goal(req.id, req.goal.data, Planner.JOINT_GOAL)
        return JointGoalResponse(True)

    def object_callback(self, msg):
        """
        Listens to /arm_control/world_update topic
        """
        # TODO

    def telemetry_callback(self, msg):
        """
        Listens to /arm_control/joint_telemetry topic
        """
        self.actual_joint_positions = msg.position

    def plan(self, goal, goal_type):
        """
        Attempts finding a trajectory reaching the given goal.
        :param goal: TODO
        :param goal_type: one of the following : JOINT_GOAL, POSE_GOAL, CARTESIAN_PATH
        :return: a tuple : a bool indicating if planning succeeded, the trajectory found
        """
        self.current_goal_type = goal_type
        self.current_goal = goal
        if goal_type == Planner.JOINT_GOAL:
            rospy.loginfo("PLANNING JOINT GOAL")
            self.move_group.set_joint_value_target(goal)
            result = self.move_group.plan()
            if len(result) > 0 and isinstance(result[0], bool):
                success, plan, planning_time, error_code = result
            else:
                plan = result
                success = bool(plan.joint_trajectory.points)
        elif goal_type == Planner.POSE_GOAL:
            rospy.loginfo("PLANNING POSE GOAL")
            self.move_group.set_pose_target(goal)
            result = self.move_group.plan()
            if len(result) > 0 and isinstance(result[0], bool):
                success, plan, planning_time, error_code = result
            else:
                plan = result
                success = bool(plan.joint_trajectory.points)
            self.move_group.clear_pose_targets()
        elif goal_type == Planner.CARTESIAN_GOAL:
            rospy.loginfo("PLANNING CARTESIAN PATH")
            # set orientation of the end effector to be the same as current
            goal.orientation = self.move_group.get_current_pose().pose.orientation  # TODO: is this needed ? I don't remember the purpose of it
            wpose = copy.deepcopy(self.move_group.get_current_pose().pose)
            wpose.position.x = goal.position.x
            wpose.position.y = goal.position.y
            wpose.position.z = goal.position.z
            waypoints = [wpose]
            plan, fraction = self.move_group.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=0.0)  # TODO: change the jump_threshold
            success = fraction == 1
        else:
            raise   # TODO

        if success:
            rospy.loginfo("PLANNING SUCCEEDED")
        else:
            rospy.logerr("PLANNING FAILED")
        return success, plan

    def start_movement(self):
        """
        Sets the planner in a moving state (new commands will be ignored while in moving state).
        """
        self.moving = True

    def stop_movement(self):
        """
        Exits moving state.
        """
        # important, even if the goal has been met, to make sure any residual movement stops
        self.move_group.stop()

        self.moving = False

    def execute_plan(self, plan):
        """
        Executes a precomputed plan (trajectory).
        :param plan: RobotTrajectory instance
        """
        self.start_movement()
        self.move_group.execute(plan, wait=True)
        self.stop_movement()

    def evaluate_execution_outcome(self):
        """
        TODO
        """
        if self.current_goal_type == Planner.JOINT_GOAL:
            # TODO
            pass
        elif self.current_goal_type == Planner.POSE_GOAL:
            actual_pose = self.get_end_effector_pose()
            # TODO: compare it with self.goal
        elif self.current_goal_type == Planner.CARTESIAN_GOAL:
            # TODO 
            pass

    def achieve_goal(self, cmd_id, goal, goal_type):
        """
        TODO
        """
        success, plan = self.plan(goal, goal_type)
        if success:
            self.execute_plan(plan)
        self.send_feedback(cmd_id, success)     # TODO: use evaluate_execution_outcome instead

    def display_trajectory(self, plan):
        """
        Display a precomputed plan in Rviz.
        :param plan: RobotTrajectory instance
        """
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_pub.publish(display_trajectory)

    def add_box_to_world(self, pose, dimensions):
        """
        TODO: modify this function
        Add and object to the world that will be used to compute collision free trajectories.
        """
        rospy.sleep(0.2)    # crucial for some reason

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose = pose
        name = "aaaaa"
        self.scene.add_box(name, p, dimensions)
        self.objects.append(name)

        rospy.sleep(.1)

    def clear_world(self):
        """
        Remove all added objects.
        """
        # TODO

    def send_feedback(self, cmd_id, success):
        """
        Send feedback about the outcome of a movement.
        """
        outcome = task_execution.msg.cmdOutcome()
        outcome.id = cmd_id
        outcome.code = (SUCCESS if success else FAIL)
        self.end_of_mvt_pub.publish(outcome)
    
    def spin(self):
        rate = rospy.Rate(25)   # 25hz
        t = time.time()
        while not rospy.is_shutdown():
            if time.time()-t > 3:
                t = time.time()
                rospy.logwarn(str(self.move_group.get_current_pose().pose.orientation)+"\n")
            rate.sleep()
        # rospy.spin()


if __name__ == "__main__":
    Planner().spin()
