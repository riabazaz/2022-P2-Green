import numpy as np
from joy.plans import Plan
import os
import json

class Calibrate(Plan):
    def __init__(self, app):
        Plan.__init__(self,app)
        self.app = app
        self.calibCoords = np.array([[0, 0, 0], [0.5, 0, 0], [1, 0, 0], [1, 0.5, 0], [1, 1, 0], [0.5, 1, 0], [0, 1, 0], [0, 0.5, 0]]) #these are placeholders
        self.cal = self.recCoords*2.5
        self.recCoords = self.recCoords + np.array([[8.5/2, 8, 0]])

        self.recCoordsWorld = []
        for rc in self.recCoords:
            self.recCoordsWorld.append(self.app.a.paperToWorld(rc))
        self.recCoordsWorld = np.array(self.recCoordsWorld)

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
            self.calibResult.append(self.recCoordsWorld[self.coordIdx,:] - self.app.a.getTool(self.app.a.getGoalThetas()))
            print('\nRecorded calibration position #%d' % self.coordIdx)
        self.coordIdx += 1 #move to next one

        if self.coordIdx<len(self.recCoords):
            print('\nMoving near next calibration position')
            wLoc = self.recCoordsWorld[self.coordIdx] #get world coordinates of current target position 
            newAngs = self.app.a.inverseKinematics(wLoc)
            print('\nNew estimated angles (rad): ', newAngs)

            self.app.arm[0].set_pos(newAngs[0]*180/np.pi*100)
            self.app.arm[1].set_pos(newAngs[1]*180/np.pi*100)
            self.app.arm[2].set_pos(newAngs[2]*180/np.pi*100)
        yield
    