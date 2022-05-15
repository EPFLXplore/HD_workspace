'''
toy_rover.py is a simple toy rover file that tests the rospy functionality of the fsm when interacting with roscore

PK 2021
'''

from re import T
import rospy
from std_msgs.msg import String, Int8


class ToyRover():
    '''
    ToyRover class to test the ros functionality of the fsm.
    '''

    def __init__(self):
        #initialize the node

        #first thing is gonna be having the FSM send out the current state
        rospy.Subscriber("sc_state", String, self.scienceStateCallback)
        rospy.Subscriber("sc_measurement", String, self.scienceMeasurementCallback)
        rospy.Subscriber("sc_info", String, self.scienceInfoCallback)

        #now we need to have the toyrover handle the input output
        self.first_input = rospy.Publisher("sc_cmd", Int8, queue_size=1)

    def scienceMeasurementCallback(self, data):
        '''
        Callback function for the science_measurement subscribed
        '''
        print("")
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def scienceInfoCallback(self, data):
        '''
        Callback function for the science_info subscribed
        '''
        print("")
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def scienceStateCallback(self, data):
        '''
        Callback function for the science_state subscribed
        '''
        print("")
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        
    def run(self):
        while True:
            control = int(raw_input("Integer Command (See Chart): "))
            self.first_input.publish(control)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("toyRover")
    rover = ToyRover()
    rover.run()


    


        
