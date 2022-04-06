import numpy as np
from joy.plans import Plan
import os
import json
import tinyik


class Move(Plan):
    def __init__(self, app):
        Plan.__init__(self,app)
        self.app = app

        arm = tinyik.Actuator(['z',[1.,0.,0.],'y',[5.,0.,0.],'y',[5.,0.,0.]])
       
        self.coordIdx = -1 #which of these coords its currently at
        self.calibResult = [] #the resulting sets of angles

    def behavior(self):
        #when the button is pushed, record the current set of angles as corresponding with the 
        #current calibration position (unless this is the first press), then move to an estimate of the 
        #next position (unless this is the last time)
        if self.coordIdx != -1:
            self.calibResult.append(self.calibCoordsWorld[self.coordIdx,:] - self.app.a.getTool(self.app.a.getGoalThetas()))
            print('\nRecorded calibration position #%d' % self.coordIdx)
        self.app.a.getTool(self.app.a.getGoalThetas())
    