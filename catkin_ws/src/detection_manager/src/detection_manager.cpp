//ros includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Image.h>

//c++ includes
#include <iostream>
#include <sstream>
#include "detection_manager.h"

#define LOW_BYTE_TRONCATION 0x00FF //multiplication by this value allows for troncation to the lower byte of a 16-bit int
#define RELIABILITY_THRESHOLD 10   //minimum number of frames that detect the desired AR tag of (x frames) before an error is sent

using namespace std;

//Control Station Topic Values
//Task
static int8_t ctrl_task;
static int8_t ctrl_command;
static int8_t semi_auto_id;
static int8_t mode;
static int8_t task_progress;

//Arm Control Topic Values
static uint8_t end_of_movement;
static uint8_t end_of_task;

//Detection Software Topic Values
static uint8_t state = 1;   //start in Inactive
static int16_t detected_elements[Number_of_Elements][7];        //note that an element is actually a sub-task.
                                                                //In theory, two elements could be linked to the same physical object on the control panel in the case where 
                                                                //two different manipulations would be made to the same physical object
static uint8_t current_element;
//Images???

int main(int argc, char **argv)
{
    ros::init(argc,argv, "listener_node");
    ros::NodeHandle manager;
    ros::Rate loop_rate(10);

    //Publishers setup
    ros::Publisher state_pub = manager.advertise<std_msgs::UInt8>("detection/state",1000);
    std_msgs::UInt8 state_msg;
    ros::Publisher current_element_pub = manager.advertise<std_msgs::UInt8>("detection/current_element",1000);
    std_msgs::UInt8 current_element_msg;

    
    //Subscribers setup
    ros::Subscriber task_sub = manager.subscribe("Task",                    1000, task_Callback);
    ros::Subscriber HD_SemiAuto_Id_sub = manager.subscribe("HD_SemiAuto_Id",1000, HD_SemiAuto_Id_Callback);
    ros::Subscriber HD_mode_sub = manager.subscribe("HD_mode",              1000, HD_mode_Callback);
    
    ros::Subscriber end_of_movement_sub = manager.subscribe("/arm_control/end_of_movement", 1000, end_of_movement_Callback);
    ros::Subscriber end_of_task_sub = manager.subscribe("/arm_control/end_of_task",         1000, end_of_task_Callback);
    
    ros::Subscriber TaskProgress_sub = manager.subscribe("TaskProgress", 1000, TaskProgress_Callback);
    
    //ros::Subscriber state_sub = manager.subscribe("detection/state",                            1000, state_Callback);
    ros::Subscriber detected_elements_sub = manager.subscribe("detection/detected_elements",    1000, detected_elements_Callback);
    //ros::Subscriber current_element_sub = manager.subscribe("detection/current_element",        1000, current_element_Callback);
    ros::Subscriber bounding_boxes_sub = manager.subscribe("detection/bounding_boxes",          1000, bounding_boxes_Callback);
    ros::Subscriber RGB_intel_sub = manager.subscribe("detection/RGB_intel",                    1000, RGB_intel_Callback);
    ros::Subscriber RGB_webcam_1_sub = manager.subscribe("detection/RGB_webcam_1",              1000, RGB_webcam_1_Callback);
    ros::Subscriber RGB_webcam_2_sub = manager.subscribe("detection/RGB_webcam_2",              1000, RGB_webcam_2_Callback);
    
    while(ros::ok()){
        state = determine_state();
        state_msg.data = state;
        state_action();
        state_pub.publish(state_msg);       //not sure about order of operations here, I think it depends on how long each thing takes
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

//checks current state and topic values, determines what state the detection software should be in. changes state.
int determine_state(void){
    switch(state){
        case INACTIVE:
            inactive_state_change_check();
            break;

        case INITIALISATION:
            initialisation_state_change_check();
            break;

        case WAIT:
            waiting_state_change_check();
            break;
        
        case MEASUREMENT:
            measurement_state_change_check();
            break;
        
        case ERROR:
            error_state_change_check();
            break;

        default:
            ROS_INFO("huh, a default state");
            ros::shutdown();
    }

    ROS_INFO("the state is %i", state);
}

//carries out actions that should occur in current state
void state_action(void){
    switch(state){
        case INACTIVE:
            break;

        case INITIALISATION:
            initialisation_action();
            break;

        case WAIT:
            waiting_action();
            break;
        
        case MEASUREMENT:
            measurement_action();
            break;
        
        case ERROR:
            error_action();
            break;

        default:
            ROS_INFO("huh, a default action");
    }
}

//callbacks need to return void, value changes should affect global static variables
void task_Callback(const std_msgs::Int8MultiArray::ConstPtr& task_array)//const std_msgs::Int8MultiArray::ConstPtr& task_array)
{
    ctrl_task = task_array->data[0];
    ctrl_command = task_array->data[1];

    ROS_INFO("task = %i", ctrl_task);
    ROS_INFO("command = %i", ctrl_command);
}

void HD_SemiAuto_Id_Callback(const std_msgs::Int8::ConstPtr& received_semi_auto_id){semi_auto_id = received_semi_auto_id->data;}
void HD_mode_Callback(const std_msgs::Int8::ConstPtr& received_mode){mode = received_mode->data;}
 
void end_of_movement_Callback(const std_msgs::UInt8::ConstPtr& received_end_of_movement){end_of_movement = received_end_of_movement->data;}
void end_of_task_Callback(const std_msgs::UInt8::ConstPtr& received_end_of_task){end_of_task = received_end_of_task->data;}

void TaskProgress_Callback(const std_msgs::Int8::ConstPtr& received_taskprogress){task_progress = received_taskprogress->data;} // not sure what we are doing with task progress exactly

//void state_Callback(const std_msgs::UInt8::ConstPtr& state){}
void detected_elements_Callback(const std_msgs::Int16MultiArray::ConstPtr& received_detected_elements)      //need to figure out how Aly is sending
{
    if(current_element == MANUAL_CTRL || current_element == ALL)
    {
        for(uint8_t element_counter = ELEMENT1; element_counter < Number_of_Elements; element_counter++)
        {
            for(uint8_t data_counter = 0; data_counter < sizeof(detected_elements)/sizeof(detected_elements[0]); data_counter++)
            {
                detected_elements[element_counter][data_counter] = received_detected_elements->data[data_counter];
            }
        }
    }
}
//void current_element_Callback(const std_msgs::UInt8::ConstPtr& current_element){}
void bounding_boxes_Callback(const sensor_msgs::Image::ConstPtr& received_bounding_boxes){}                 // image stuff
void RGB_intel_Callback(const sensor_msgs::Image::ConstPtr& received_RGB_intel){}                           // image stuff
void RGB_webcam_1_Callback(const sensor_msgs::Image::ConstPtr& received_RGB_webcam_1){}                     // image stuff
void RGB_webcam_2_Callback(const sensor_msgs::Image::ConstPtr& received_RGB_webcam_2){}                     // image stuff

//State Transtions - check if a change from the current state should be done and then changes to the correct state
void inactive_state_change_check(void)
{
    if(ctrl_task == MAINTENANCE && ctrl_command == LAUNCH)  //activation from control station
        state = INITIALISATION;
    else
        state = INACTIVE;
}
void initialisation_state_change_check(void)
{
    if( ctrl_task == MAINTENANCE &&
        ctrl_command == ABORT)                              //abort from control station
        state = INACTIVE;
    else if( ctrl_task == MAINTENANCE && 
        ctrl_command == LAUNCH && 
        current_element == ALL &&
        task_progress == true)                              //waiting for arm to move to home position  
        state = WAITING;
    else
        state = INITIALISATION;
}
void waiting_state_change_check(void)
{
    if( ctrl_task == MAINTENANCE &&                         //abort from control station
        ctrl_command == ABORT)                              
        state = INACTIVE;
    else if(ctrl_task == MAINTENANCE &&                     //all tasks complete
        ctrl_command == LAUNCH &&
        end_of_movement == true &&
        end_of_task == true &&
        task_progress == true &&
        detected_elements_check() == 1)                                                   
        state = INACTIVE;
    else if(ctrl_task == MAINTENANCE &&                     //end of manipulation attempt (verification received)
        ctrl_command == LAUNCH &&                           //how to check semi_auto_id situation? how to treat retry situation -> ctrl station sends same element id
        (semi_auto_id != MANUAL_CTRL ||
        semi_auto_id != Number_of_Elements) &&
        end_of_movement == 1 &&
        end_of_task == 1)
        state = INITIALISATION;
    else if(ctrl_task == MAINTENANCE &&                     //end of movement (during manipulation attempt), update relative position
        ctrl_command == LAUNCH &&
        (semi_auto_id < Number_of_Elements) &&
        end_of_movement == 1 &&
        end_of_task == 0)
        state = MEASUREMENT;
    else
        state = WAITING;
}
void measurement_state_change_check(void)
{
    if( ctrl_task == MAINTENANCE &&                         //abort from control station
        ctrl_command == ABORT)
        state = INACTIVE;
    else if(ctrl_task == MAINTENANCE &&                     //finished updating measurements
        ctrl_command == LAUNCH)//                           //detected elements update should probably be a service
        //detected_elements_check();              
        state = WAITING;
    else if(ctrl_task == MAINTENANCE &&                     //reliability error
        ctrl_command == LAUNCH &&
        semi_auto_id != MANUAL_CTRL &&
        end_of_task == 0 &&
        detected_elements_check() == -1)
        state = ERROR;
    else
        state = MEASUREMENT;
}
void error_state_change_check(void)
{
    if( ctrl_task == MAINTENANCE &&                         //abort from control station
        ctrl_command == ABORT)
        state = INACTIVE;
    else
        state = INITIALISATION;
}

int8_t initialisation_action(void)
{
    current_element = ALL;
    //cameras init/vision node start
    return 1;
}

int8_t waiting_action(void)
{
    while(end_of_movement == 0)
    {
        sleep(1);   //sleep 1 second
    }
    if(end_of_task == 1 && semi_auto_id == current_element && current_element != ALL) // && verification not sent)   //waiting for verification from control station
    {
        //send request for verification to Control station
        uint16_t verification_response; //= response to request = 0b0...0 or 0b100...00
        detected_elements[current_element][0] += verification_response;
    }
    while(end_of_task == 1 && semi_auto_id == current_element)// && verification sent and received)       //wait to receive new target element from ctrl
    {
        sleep(1);
    }

}

int8_t measurement_action(void)
{
    //call measurement functions
    //update values in detected_elements
    //publish detected_elements
}

int8_t error_action(void)
{
    sleep(1);   //time to be detected
    return 0;
}

int detected_elements_check(void)
{
    //check for all done
    if(state == WAITING)
    {
        int8_t all_done = 0;
        for(auto element : detected_elements)
        {
            if(element[0] < 0)
                all_done++;
        }
        if(all_done == Number_of_Elements)
            return 1;
    }
    //check for reliability error on current element
    else if(state == MEASUREMENT)
    {
        uint8_t current_elem_reliability = detected_elements[current_element][0] & LOW_BYTE_TRONCATION;
        if(current_elem_reliability < RELIABILITY_THRESHOLD)
        {
            return -1;
        }
    }
    return 0;
}