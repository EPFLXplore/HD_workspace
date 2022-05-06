class ManualController:
    def cmdCallback(msg: std_msgs/Int8MultiArray):
        """listens to /arm_control/manual_axis_command topic"""
    def initiateTask():
        """starts a ManuaMotion task"""
    def updateTask():
        """updates tasks velocity if new and sets next goal if command is still received"""
    def abortTask():
        """stops the task"""


def main():
    """init ros and subscribe to manual commands from manager"""