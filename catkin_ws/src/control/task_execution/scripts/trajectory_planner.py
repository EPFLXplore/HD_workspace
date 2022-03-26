import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg


SUCCESS = 5
FAIL = 17
MOTOR_COUNT = 7


class Planner:
    """recieves goals to reach and sends information to MoveIt in order for it to compute a trajectory"""
    JOINT_GOAL = 0
    POSE_GOAL = 1
    CARTESIAN_GOAL = 2

    def __init__(self):
        # initialize ROS ===============================================================================================
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        # publishers ===================================================================================================
        self.end_of_mvt_pub = rospy.Publisher("/arm_control/end_of_movement", std_msgs.msg.UInt8, queue_size=5)
        self.display_trajectory_pub = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # objects from MoveIt Commander needed to communicate with MoveIt ==============================================
        group_name = "arm_group"    # the planning group of the arm
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        # ==============================================================================================================
        self.actual_joint_positions = [0.0]*MOTOR_COUNT
        self.moving = False

        self.objects = []

        self.set_max_velocity_and_acceleration(1, 1)

        rospy.spin()

    def set_max_velocity_and_acceleration(self, vel=0.1, acc=0.1):
        """
        Sets maximal velocity and acceleration scaling factors
        Allowed values are in (0,1].
        The default value is set in the joint_limits.yaml of the robot_config package
        """
        self.move_group.set_max_velocity_scaling_factor(vel)
        self.move_group.set_max_acceleration_scaling_factor(acc)

    def pose_goal_callback(self, msg: geometry_msgs.msg.Pose):
        """
        Listens to /arm_control/pose_goal topic
        """
        if self.moving:
            return
        self.achieve_goal(msg, Planner.POSE_GOAL)

    def joint_goal_callback(self, msg: std_msgs.msg.Float32MultiArray):
        """
        Listens to /arm_control/joint_goal topic.
        """
        if self.moving:
            return
        self.achieve_goal(msg, Planner.JOINT_GOAL)

    def object_callback(self, msg: geometry_msgs.msg.Pose):
        """
        Listens to /arm_control/world_update topic
        """

    def telemetry_callback(self, msg: sensor_msgs.msg.JointState):
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
        if goal_type == Planner.JOINT_GOAL:
            rospy.loginfo("PLANNING JOINT GOAL")
            self.move_group.set_joint_value_target(goal)
            success, plan, planning_time, error_code = self.move_group.plan()
        elif goal_type == Planner.POSE_GOAL:
            rospy.loginfo("PLANNING POSE GOAL")
            self.move_group.set_pose_target(goal)
            success, plan, planning_time, error_code = self.move_group.plan()
            self.move_group.clear_pose_targets()
        elif goal_type == Planner.CARTESIAN_GOAL:
            rospy.loginfo("PLANNING CARTESIAN PATH")
            # set orientation of the end effector to be the same as current
            goal.orientation = self.move_group.get_current_pose().pose.orientation
            waypoints = [goal]
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

    def achieve_goal(self, goal, goal_type):
        success, plan = self.plan(goal, goal_type)
        if success:
            self.execute_plan(plan)
        self.send_feedback(success)

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

    def send_feedback(self, success):
        """
        Send feedback about the outcome of a movement.
        """
        outcome = std_msgs.msg.UInt8()
        outcome.data = (SUCCESS if success else FAIL)
        self.end_of_mvt_pub.publish(outcome)


if __name__ == "__main__":
    Planner()
