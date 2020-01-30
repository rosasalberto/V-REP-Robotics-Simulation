import vrep
import numpy as np

"""
Alberto Rosas Garcia 2019

helper class for the Agent class. Receives and process images from the agent environtment.
"""

class SensorInformation(object):
    """
    Class that reads and provides information from the sensors
    """

    def __init__(self):
        super(SensorInformation, self).__init__()
        
        self.color_line = [20,30] # color margin of the lines to follow
        self.color_stop = [80,105] # color margin of the line to stop
        
        self.cv_image=[0,0,0]
        self.binary_image = [0,0,0]             
        self.avoid_obstacle=False              
        
        print ("Initialize SensorInformation")


    def process_image(self):
        """
        Receives 3 1x1 resolution images and process the information to determine if Vision sensors detect black line to follow or red line to stop
        """
        for i in range(0,3):
            return_code,resolution,dataimage=vrep.simxGetVisionSensorImage(self.client_id,self.visionsensor[i],1,vrep.simx_opmode_buffer)
            if dataimage:
                self.cv_image[i] = dataimage
                
        h = np.array(self.cv_image)
        self.binary_image = ((h > self.color_line[0]) & (h < self.color_line[1]))
        self.stop_robot = np.all((h > self.color_stop[0]) & (h < self.color_stop[1]))
        
    def proximity_detection(self):
        """
        Check if the ultrasonic sensor is detecting an object and returns x coordinate
        """
        return_code,detection_state,detectedPoint,x,y=vrep.simxReadProximitySensor(self.client_id,self.ultrasensor,vrep.simx_opmode_streaming)
        if detection_state:
            self.avoid_obstacle=True
        else:
            self.avoid_obstacle=False   
            
        return detectedPoint[0] #return the value of the x coordinate.
    

            
    

            
            
    