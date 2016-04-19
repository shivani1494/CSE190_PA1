#!/usr/bin/env python

import rospy
import random
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData
from cse_190_assi_1.msg import temperatureMessage
from std_msgs.msg import Bool
from read_config import read_config
from math import exp


 " this entire package can be thought of as a robot, with this being robot's brain"
 " the other being robot's sensors etc "
 " self refers to the object itself, the class for which the fucntion is defined"


class Robot():
    def __init__(self):
    	"""this is the constructor??"""

    	"""read the config file, this is a hashmap"""
 		self.config = read_config()
    	
        """this is telling that this class is a node, I can publish and subscribe from this"""      
        rospy.init_node('Robot', anonymous=True)
    	
       """this is the matrix that we will 
        be updating with each and every move, initially all positions are equally probable"""
        probMatrix[4][5] = 1/20 

        "run a loop to over all the moves and update the probMatrix as you go...."

        "temperature update"
        temperatureSensorUpdate()

        "how do I know that the updated belief about my pos is correct?"

        "texture update"
        textureSensorUpdate()

        "how do I know that the updated belief about my pos is correct?"

        "move update"
        "moveRobotUpdate()"





    def getNormalizationConstant(self, reading, textureTrue):

        normalizatnCnst = 0
        i = 0
        j = 0

        "check what the normalization constant will be??????"
        for i in range ( len(self.tempMatrix) )
            for j in range( len( self.tempMatrix[i] ) )
                if textureTrue:
                    normalizatnCnst = normalizatnCnst + getJointPTextureAndX(tempReading, i, j)
                else !textureTrue:
                    normalizatnCnst = normalizatnCnst + getJointPTempAndX(tempReading, i, j) "is this how we call the function??"

            j = 0 "update j"
            i++

        return normalizatnCnst


    def temperatureSensorUpdate(self): 

        "subscribe to temperature topic"
    	rospy.Subscriber("/temp_sensor/activation", String, callback)

    	rospy.sleep
        """why do we need to sleep"""


        """we need to publish to it so that now, it knows I want values"""
        """so now it is ACTIVATED to send values to you"""
    	rospy.Publisher("/temp_sensor/data", true) "shou"
    	
    	"""" rospy.spin(): rospy is  library that allows us to use all classses etc"""

        
        "this is a matrix for the temperature"
        tempMatrix = self.config["pipe_map"]

        "standard deviation for temperature"
        tempStdDev = self.config["temp_noise_std_dev"]

        tempReading = 0
        "call the callback function here to get the reading"
        callback(tempReading)

        "publish the reading"


        normalizatnCst = getNormalizationConstant(tempReading, false) 	

 		"using the reading update the probabiltiy matrix"
        i = 0
        j = 0
        for i in range(len(self.probMatrix))
            for j in range( len( self.probMatrix[i] ) )
                self.probMatrix[i][j] = calculateProbXGivenT(i, j, tempReading, normalizatnCst)
            i++
            j = 0
       

    "probabiltiy of x_i, y_i, given temperature = tempReading"
 	def calculateProbXGivenT(self, x, y, tempReading, normalizatnCst):

        "So this is saying what is the P of temp=H/C/-, given the grid value"
        "why does this formula work?????"

        return getJointPTempAndX(tempReading, x, y)/normalizatnCst


    def getJointPTempAndX(self, tempReading, x ,y):

        if self.tempMatrix[x][y] == "H":
            gridTemp = 40 
        elif self.tempMatrix[x][y] == "C":
            gridTemp = 20
        else self.tempMatrix[x][y] == "-":
            gridTemp = 25

        error = gridTemp - tempReading

        constant = 1.0/((1.414 * 3.14) * self.tempStdDev)

        "print .... " 

        eExp = math.exp(-0.5*( (math.pow(error, 2) )/( 2*math.pow(tempStdDev, 2) ) ) )

        "probability of T=reading, given X=x_i"
        probTGivenX = constant * eExp

        prior = self.probMatrix[x][y]

        jointTempAndXProb = probTGivenX * prior

        return jointTempAndXProb

    
    def textureSensorUpdate(self):

        textureReading = textureSensorClient()
        textureMap = self.config["texture_map"]
        
        getNormalizationConstant(textureReading, true)

        i = 0
        j = 0
        for i in range(len(self.probMatrix))
            for j in range( len( self.probMatrix[i] ) )
                if textureMap[i][j] == textureReading:
                    self.probMatrix[i][j] = calculateProbXGivenT(tempReading, i, j, normalizatnCnst)
            i++
            j = 0

    def getJointPTextureGivenX(self, probTexture, i, j):
        return probTexture * self.probMatrix[i][j]

    def calculateProbXGivenTexture(self, i, j, textureReading, normalizatnCnst):
        
        if textureMap[i][j] == textureReading:
            jointP = getJointPTextureGivenX(self.config("prob_tex_correct"), i, j)
        else textureMap[i][j] != textureReading:
            jointP = getJointPTextureGivenX(1 - self.config("prob_tex_correct"), i, j)

        return jointP/normalizatnCnst
    
                



    "def moveRobotUpdate(self):"






 

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)






"connect to the texture sensor client properly!!!!!!"
def textureSensorClient():
    rospy.wait_for_service('requestTexture')
    requestTexture = rospy.ServiceProxy('requestTexture', requestTexture)   
    textureResponse = requestTexture()
    textureReading = textureResponse.data

    "I do need to send in my current x_i to actually get what the texture value"
    "is at that point"

    return textureReading



 "callback is a function that you should implement and "
 "it will set the value in temperature_data which is "
 "of type temperature message, so that now you can use it"

 "Do we put a ; after any line in python ???????????"
 "how to declare a matrix ????????"

 "print values to see what you are getting and did you expect that??"
        "assert before passing values into a function"

"why dont we pass in self into this?? because it is not a method of this class"
"but well it is a method of some class??"


"I do understand how everythign is working mechanically and all the prob"
"but I dont understand how this model really works to give us the locations"
"feels like I have been given with a model but I dont understand why should this"
"always work and nto fail!"





        