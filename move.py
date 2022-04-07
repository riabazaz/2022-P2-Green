
import tinyik
from joy.plans import Plan
from joy import progress
from numpy import linspace,zeros,pi,rad2deg, append
from scipy.interpolate import griddata

#armSpec = asarray([
#        [0,0.02,1,5,0],
#        [0,1,0,5,1.57],
#        [0,1,0,5,0],
#      ]).T

class Move( Plan ):
    def __init__(self,app,*arg,**kw):
        Plan.__init__(self,app,*arg,**kw)
        self.app = app
        self.steps = []
        #Make another arm with the same orientation and arm lenght segments as
        #what is defined in armSpec. This is what is used for inverse kinematics.
        self.moveArm = tinyik.Actuator(['z',[0.,0.,0.],'y',[5.,0.,0.],'y',[5.,0.,0.]])
        self.moveArm.angles = self.app.armSpec[-1,:]
        self.pos = []   #Goal position for move. Will either be grid point or square corner. 
        self.calibrated = False
        self.CalDone = False
        self.square = False
        self.currentPos = []
        self.nextPos = []
        self.curr = 0
    #used to get initial joint angles before autonomous move
    def syncArm(self):
        ang = zeros(len(self.app.arm))
        for i,motor in enumerate(self.app.arm):
            ang[i] = motor.get_goal()*(pi/18000.)   #convert angles from centidegrees to radians
        self.moveArm.angles = ang
    
    #for moving towwards desired position
    def behavior(self):
        angOffset = zeros(len(self.app.arm))

        # if it is calibrated, we want to draw the line to the next position using the calibrated values
        if self.calibrated:
            self.pos = append(self.moveArm.ee, [0])
 

        self.syncArm()     
        if self.CalDone == False or self.square == False or len(self.currentPos) == 0:
            #forward kinematics - get current end effector position given joint angles
            self.currentPos = self.app.idealArm.getTool(self.moveArm.angles)


        progress(str(self.pos))
        progress(str(self.currentPos))

        #Create a number of evenly spaced steps between current position and goal position
        self.steps = linspace(self.currentPos,self.pos,5)[:,:-1]    #Can adjust number of steps.
        #execute movement along path of steps
        for stepCount,step in enumerate(self.steps):
            progress('Step #%d, %s' % (stepCount,str(step)))
            self.app.currStep = stepCount
            self.moveArm.ee = step
            for i,motor in enumerate(self.app.arm):
                #Calculate angles to move each step and move the motors 
                motor.set_pos(rad2deg(self.moveArm.angles[i]+angOffset[i])*100)    #feed in angle to set_pos as centidegrees
            yield self.forDuration(4)
        progress('Move complete')