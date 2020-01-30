import sys
import os

cwd = os.path.abspath('.') + '/vrep_api'
sys.path.append(cwd)

from controller import Controller
from sensor_information import SensorInformation

import vrep

"""
Alberto Rosas Garcia 2019

This script is the main script of this project, which brings together reading information from the sensors, control system strategy and robot actuation.
The code works as follows:

1. Sensor Information are received and processed by SensorReceiver. We receive the information from 3 1x1 resolution Vision Sensor and 1 ultrasonic sensor.
2. We process the Vision Image sensor information and the ultrasonic sensor information
3. Processed information is sent to Controller class to determine action.
4. The agent proceed with the determined action.
5. This process continues indefinitely for as long as the Vision Sensor detects a red line.
"""

class Agent(Controller, SensorInformation, object):
    """
    Brain of the Robot, which inherits from SensorInformation and Controller.
    """

    def __init__(self, client_id, dinamic_obs, actuators, visionsensor, ultrasensor, ultraSensorSides):
        
        #super calls to parent classes
        super(Agent, self).__init__()
        
        self.client_id = client_id
        self.dinamic_obs = dinamic_obs
        self.actuators = actuators
        self.visionsensor = visionsensor
        self.ultrasensor = ultrasensor
        self.speed = 11 #speed of the agent
        self.ultrasensorsides = ultraSensorSides
        
        
    def robot_control(self):
        """
        Given action, will execute a specificed behavior from the robot
        action:
         0 = forward
         1 = leftTurn
         2 = rightTurn
         3 = stop
        """
        self.state[self.action].__call__()
        

    def run(self):
        """
        The main run loop
        """
        while not self.stop_robot:
            self.process_image()
            vectordistance=self.proximity_detection()         
            if self.avoid_obstacle:
                self.define_action_proximitiy(vectordistance)   
            else:
                self.define_action_vision()
            self.robot_control()
            
        #stop simulation
        vrep.simxStopSimulation(self.client_id,vrep.simx_opmode_oneshot)
    

if __name__ == '__main__':
    
    # initializes api client
    vrep.simxFinish(-1) # just in case, close all opened connections
    client_id = vrep.simxStart('127.0.0.1',19997,True,True,5000,1) # Connect to V-REP
    returnCode=vrep.simxStartSimulation(client_id,vrep.simx_opmode_oneshot)
    
    if client_id!=-1:
        print('Connected to remote API server')
        returnCode,dinamic_obs=vrep.simxGetObjectHandle(client_id,'Pioneer_p3dx',vrep.simx_opmode_blocking)
        returnCode,rueda_dd=vrep.simxGetObjectHandle(client_id,'rollingJoint_rl',vrep.simx_opmode_blocking)
        returnCode,rueda_di=vrep.simxGetObjectHandle(client_id,'rollingJoint_rr',vrep.simx_opmode_blocking)
        returnCode,rueda_td=vrep.simxGetObjectHandle(client_id,'rollingJoint_fl',vrep.simx_opmode_blocking)
        returnCode,rueda_ti=vrep.simxGetObjectHandle(client_id,'rollingJoint_fr',vrep.simx_opmode_blocking)
        returnCode,ultra_sensor=vrep.simxGetObjectHandle(client_id,'Proximity_sensorford',vrep.simx_opmode_blocking)
        returnCode,VisionSensor=vrep.simxGetObjectHandle(client_id,'VisionSensor',vrep.simx_opmode_blocking)
        returnCode,VisionSensorleft=vrep.simxGetObjectHandle(client_id,'VisionSensorleft',vrep.simx_opmode_blocking)
        returnCode,VisionSensorright=vrep.simxGetObjectHandle(client_id,'VisionSensorright',vrep.simx_opmode_blocking)
        returnCode,ultra_sensor_right=vrep.simxGetObjectHandle(client_id,'Proximity_sensorleft',vrep.simx_opmode_blocking)
        returnCode,ultra_sensor_left=vrep.simxGetObjectHandle(client_id,'Proximity_sensorright',vrep.simx_opmode_blocking)
        actuators=[rueda_dd,rueda_di,rueda_td,rueda_ti]
        visionSensor=[VisionSensorleft,VisionSensor,VisionSensorright]
        ultraSensor=ultra_sensor
        ultraSensorSides=[ultra_sensor_left, ultra_sensor_right]
        armJoints=[0,0,0,0]
        for i in range(0,4):
            returnCode,armJoints[i]=vrep.simxGetObjectHandle(client_id,'youBotArmJoint'+str(i+1),vrep.simx_opmode_blocking)
        
        #initialize sensors
        for i in range(0,3):
            returnCode,resolution,dataimage=vrep.simxGetVisionSensorImage(client_id,visionSensor[i],1,vrep.simx_opmode_streaming_split+4000)
        returnCode,detectionState,detectedPoint,x,y=vrep.simxReadProximitySensor(client_id,ultraSensor,vrep.simx_opmode_streaming)
    else:
        print("Failed connection to V-REP")
        
    #initializes Robot Controller and runs it 
    node = Agent(client_id, dinamic_obs, actuators, visionSensor, ultraSensor, ultraSensorSides)
    node.run()