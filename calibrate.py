import numpy as np
from joy.plans import Plan
import os
import json

class Calibrate(Plan):
    def __init__(self, app):
        Plan.__init__(self,app)
        self.app = app
        self.calibCoords = np.array([[0, 0, 0], [0.5, 0, 0], [1, 0, 0], [1, 0.5, 0], [1, 1, 0], [0.5, 1, 0], [0, 1, 0], [0, 0.5, 0]]) #these are placeholders
        self.calibCoords = self.calibCoords*2.5
        self.calibCoords = self.calibCoords + np.array([[8.5/2, 8, 0]])

        self.calibCoordsWorld = []
        for rc in self.calibCoords:
            self.calibCoordsWorld.append(self.app.a.paperToWorld(rc))
        self.calibCoordsWorld = np.array(self.calibCoordsWorld)

        self.coordIdx = -1 #which of these coords its currently at
        self.calibResult = [] #the resulting sets of angles

    def paperToWorld(self, paperCoord):
        hPaper = np.array(list(paperCoord) + [1]) #make homogenous coord
        hWorld =  self.app.Tws2w @ self.app.Tp2ws  @ hPaper
        return hWorld[:3]/hWorld[3] #divide out normalization component
  

    def behavior(self):
        #when the button is pushed, record the current set of angles as corresponding with the 
        #current calibration position (unless this is the first press), then move to an estimate of the 
        #next position (unless this is the last time)
        if self.coordIdx != -1:
            self.calibResult.append(self.calibCoordsWorld[self.coordIdx,:] - self.app.a.getTool(self.app.a.getGoalThetas()))
            print('\nRecorded calibration position #%d' % self.coordIdx)
        self.app.a.getTool(self.app.a.getGoalThetas())
    