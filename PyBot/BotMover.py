'''
Created on 08.10.2016

@author: mario
'''
from math import pi, cos, sin

class BotDiffWheel(object):
    '''
    Implements all required functions regarding a differential wheel robot
    We need:
    DifferentialWheel movement model
    Unicycle movement model for designing the controller
    PID controller
    Odomoetry based on wheel encoder
    '''
    def __init__(self, wheeldist,wheelradius,xpos=0,ypos=0,phi = 0,starttime = 0):
        '''
        We need to know the distance between the two wheels and wheelradius
        in SI unit m
        xpos, ypos (m): robot initial position
        phi (rad): robot initial orientation (xdir = 0)
        starttime (s) : starttime
        
        '''
        self.L = wheeldist
        self.R = wheelradius
        self.x = xpos
        self.y = ypos
        self.phi = phi
        self.t = starttime

    def getVwheels(self,v,omega):
        '''
        Returns (vr, vl) as right and left speed of wheel
        for a given v (speedvector m/s), and omega (angular speed rad/s)
        '''
        vr = (2*v + omega*self.L)/2*self.R
        vl = (2*v - omega*self.L)/2*self.R
        return vr,vl

class WheelEncoder():
    '''
    Implements a wheelencoder and its supporting functions
    '''
    def __init__(self,ticksperrevol):
        '''
        Needs 
        ticks per revolution
        '''
        self.N = ticksperrevol

    def getDwheel(self,deltaticks,robot):
        '''
        Calculates the distance traveled by one wheel after counted deltaticks 
        '''
        dist = 2* pi * robot.R * deltaticks / self.N
        return dist
    
    def getUpdatePos(self,ticksleft,ticksright,robot):
        '''
        Calculates Distance traveled by centerpoint (two wheels)
        '''
        Dleft = self.getDwheel(ticksleft)
        Dright = self.getDheel(ticksright)
        Dcenter = (Dleft + Dright) / 2
        robophi = robot.phi #current robot orientation
        robox = robot.x
        roboy = robot.y
        xupd = robox + Dcenter * cos(robophi)
        yupd = roboy + Dcenter * sin(robophi)
        phiupd = robophi + (Dright - Dleft)/robot.L
        return xupd,yupd,phiupd
