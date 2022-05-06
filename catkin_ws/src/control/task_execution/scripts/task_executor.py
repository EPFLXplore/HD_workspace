class Executor:
    def taskAssignementCallback(msg: Task):
        """listens to arm_control/task_assignment topic"""
    def assignTask(task: Task):
        """assigns the task"""
    def initiateTask():
        """starts assigned task"""
    def abortTask():
        """stops the assigned task"""


def main():
    """init ros and subscribe to task commands from manager"""
