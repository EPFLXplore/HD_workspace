'''
toy_rover.py is a simple toy rover file that tests the rospy functionality of the fsm when interacting with roscore

PK 2021
'''

# from distutils.command.build_scripts import first_line_re
from re import T
import rospy
from std_msgs.msg import String, Int8, UInt8, Int8MultiArray


class ToyRover():
    '''
    ToyRover class to test the ros functionality of the fsm.
    '''

    def __init__(self):
        #initialize the node

        #first thing is gonna be having the FSM send out the current state
        rospy.Subscriber("detection/state", UInt8, self.hdStateCallback)
        rospy.Subscriber("detection/current_element", String, self.scienceMeasurementCallback)
        # rospy.Subscriber("sc_info", String, self.scienceInfoCallback)

        #now we need to have the toyrover handle the input output
        self.first_input = rospy.Publisher("Task", Int8MultiArray, queue_size=1)
        #opening command is [1,3]

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

    def hdStateCallback(self, data):
        '''
        Callback function for the science_state subscribed
        '''
        print("")
        rospy.loginfo(rospy.get_caller_id() + "I heard %d", data.data)
        
    def run(self):
        while True:
            control = raw_input("Integer Command (See Chart): ")
            ctrl_list = control.split()
            ctrl_list = [int(a) for a in ctrl_list]
            data = Int8MultiArray()
            # print ctrl_list[0]
            data.data = ctrl_list
            self.first_input.publish(data)
            # self.first_input.publish(control)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("toyRover")
    rover = ToyRover()
    rover.run()


    


        
