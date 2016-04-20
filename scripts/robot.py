#!/usr/bin/env python

import rospy
import random
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, requestTexture
from cse_190_assi_1.msg import temperatureMessage
from std_msgs.msg import Bool
from read_config import read_config
from math import exp


class Robot():
    def __init__(self):
        
        self.config = read_config()
        
        rospy.init_node('Robot', anonymous=True)
    
        self.probMatrix = []
        self.row = [1.0/20, 1.0/20, 1.0/20, 1.0/20, 1.0/20]
        self.probMatrix.append(self.row)
        self.probMatrix.append(self.row)
        self.probMatrix.append(self.row)
        self.probMatrix.append(self.row)
        
        rospy.Subscriber("/temp_sensor/data", temperatureMessage, self.callback)

        rospy.sleep(5)
        "why are we doing this??? why are we importing these data types from somewhere else???"
        self.pub = rospy.Publisher("/temp_sensor/activation", Bool, queue_size=10)        

        rospy.wait_for_service('requestTexture')
        self.requestTexture = rospy.ServiceProxy('requestTexture', requestTexture)  
        self.temperatureSensorUpdate()
        


    def getNormalizationConstant(self, reading, textureTrue):

        normalizatnCnst = 0

        for i in range ( len(self.tempMatrix) ):
            for j in range( len( self.tempMatrix[i] ) ):
                if textureTrue == true:
                    normalizatnCnst = normalizatnCnst + getJointPTextureAndX(tempReading, i, j)
                else: 
                    normalizatnCnst = normalizatnCnst + getJointPTempAndX(tempReading, i, j)

        return normalizatnCnst


    def temperatureSensorUpdate(self): 

        self.pub.publish(True)

        tempMatrix = self.config["pipe_map"]

        tempStdDev = self.config["temp_noise_std_dev"]


    def calculateProbXGivenT(self, x, y, tempReading, normalizatnCst):
        return getJointPTempAndX(tempReading, x, y)/normalizatnCst


    def getJointPTempAndX(self, tempReading, x ,y):

        if self.tempMatrix[x][y] == "H":
            gridTemp = 40 
        elif self.tempMatrix[x][y] == "C":
            gridTemp = 20
        else:
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

        for i in range(len(self.probMatrix)):
            for j in range( len( self.probMatrix[i] ) ):
                self.probMatrix[i][j] = calculateProbXGivenT(textureReading, i, j, normalizatnCnst)
        
           

    def getJointPTextureGivenX(self, probTexture, i, j):
        return probTexture * self.probMatrix[i][j]

    def calculateProbXGivenTexture(self, i, j, textureReading, normalizatnCnst):
        
        if textureMap[i][j] == textureReading:
            jointP = getJointPTextureGivenX(self.config("prob_tex_correct"), i, j)
        else:
            jointP = getJointPTextureGivenX(1 - self.config("prob_tex_correct"), i, j)

        return jointP/normalizatnCnst
    
                



    def moveRobotUpdate(self):






 

    def callback(data):

        normalizatnCst = getNormalizationConstant(tempReading, false)   

        for i in range(len(self.probMatrix)):
            for j in range( len( self.probMatrix[i] ) ):
                self.probMatrix[i][j] = calculateProbXGivenT(i, j, tempReading, normalizatnCst)   

        textureSensorUpdate()
        
        move_list = self.config("move_list")

        for i in range(len(move_list)):
            for j in range(len(move_list[i])):
                

        moveRobotUpdate()



        "you keep sending me values right and these are coming in to me through the call back funtion"
        "now again I need the tempertaure values and I need to keep going until I received all"
        "walked through the entire move list"

     
    def textureSensorClient():
         
        textureResponse = self.requestTexture()
        textureReading = textureResponse.data
        return textureReading

if __name__ == '__main__':
     Robot()


        

