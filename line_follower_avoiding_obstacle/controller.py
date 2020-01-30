import vrep
import numpy as np

"""
Alberto Rosas Garcia 2019

This script is a helper script for the Controller class, and executes actions decided on by the network
It basically creates Twist message and then specifies the message based on the inputted action
"""

class Controller(object):
    """
    The controller class decides which action to take depending on the sensor information.
    """

    def __init__(self):
        super(Controller, self).__init__()

        # defines the states
        self.state  =   {0:self.forward,
                      1:self.left_turn,
                      2:self.right_turn,
                      3:self.stop_state}
        
        self.action = 0
        self.stop_robot = False
        self.last_curve = 0  #helper in case we don't detect the line.
        self.timer = -1500  #counter that helps the agent when we are avoiding and obstacle before returning to the line.
        self.detected_obstacle = False #helper to determine if the detected object will not affect the current trajectory of the agent.
        self.safety_distance  = 0.18

        print ("Initialized Controller")
        
    def define_action_vision(self):
        """
        Sets the action to move depending on the Vision sensor readings
        """
        
        if self.stop_robot:     #if the agent detects red line then stop
            self.action = 3
            return
        
        if self.binary_image[0] and self.binary_image[2]: #if left and right vision sensor detect
            self.detected_obstacle = False
            self.timer = -1500
            
            if self.last_curve == 2:
                self.action = 1
                
            else:
                self.action = 2     
                
        elif self.binary_image[0]:
            self.action = 1
            self.last_curve = 1
            self.detected_obstacle = False
            self.timer = -1500
            
        elif self.binary_image[2]:
            self.action = 2
            self.last_curve = 2
            self.detected_obstacle=False
            self.timer = -1500
            
        elif self.binary_image[1]:
            self.action = 0
            self.detected_obstacle = False
            self.timer = -1500
            
        elif vrep.simxGetLastCmdTime(self.client_id) - self.timer<600: # if time <0.8 seconds, returns to line
            self.action = 0
        else:
            self.action = self.last_curve # If we don't detect any line, then the agent follow the last_curve helper action.
   
    
    def define_action_proximitiy(self, vectordistance):
        """
        Sets the action depending on the ultrasonic sensor
        """
        
        if not self.detected_obstacle and np.abs(vectordistance)>self.safety_distance: #go forward
            self.action =  0
            
        elif vectordistance>= 0: #turn right
            self.action = 2
            self.timer = vrep.simxGetLastCmdTime(self.client_id)
            if not self.detected_obstacle:
                self.last_curve = 1  #agent will have to go left after avoiding the object
                self.detected_obstacle = True
                
        else:  #turn left
            self.action = 1
            self.timer = vrep.simxGetLastCmdTime(self.client_id)
            if not self.detected_obstacle:
                self.last_curve = 2 #agent will have to go right after avoiding the object
                self.detected_obstacle = True            
               
    
    def forward(self):
        """
        Agent move forward
        """
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[0],self.speed,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[1],self.speed,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[2],self.speed,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[3],self.speed,vrep.simx_opmode_oneshot);
        


    def left_turn(self):
        """
        Agent turn left
        """
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[0],self.speed,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[1],0.1*self.speed,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[2],self.speed,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[3],0.1*self.speed,vrep.simx_opmode_oneshot);



    def right_turn(self):
        """
        Agent turn right
        """
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[0],0.1*self.speed,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[1],self.speed,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[2],0.1*self.speed,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[3],self.speed,vrep.simx_opmode_oneshot);


    def stop_state(self):
        """
        Agent stop
        """
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[0],0,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[1],0,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[2],0,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.client_id,self.actuators[3],0,vrep.simx_opmode_oneshot);

