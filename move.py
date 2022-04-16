
import tinyik
from joy.plans import Plan
from joy import progress
from numpy import linspace,zeros,pi,rad2deg, append, round, maximum, deg2rad
from scipy.interpolate import interp1d, griddata 
from math import floor

#armSpec = asarray([
#        [0,0.02,1,5,0],
#        [0,1,0,5,1.57],
#        [0,1,0,5,0],
#      ]).T
'''
class Move( Plan ):
    def __init__(self,app,*arg,**kw):
        Plan.__init__(self,app,*arg,**kw)
        self.app = app
        self.steps = []
        #Make another arm with the same orientation and arm lenght segments as
        #what is defined in armSpec. This is what is used for inverse kinematics.
        self.moveArm = tinyik.Actuator(['z',[0.,0.,0.],'y',[10.,0.,0.],'y',[10.,0.,0.]])
        self.moveArm.angles = self.app.armSpec[-1,:]
        self.pos = []   #Goal position for move. Will either be grid point or square corner. 
        self.calibrated = False
        self.square = False
        self.currentPos = []
        self.nextPos = []
        self.curr = 0
        self.step_constant = 1/7
    #used to get initial joint angles before autonomous move
    def syncArm(self):
        ang = zeros(len(self.app.arm))
        for i,motor in enumerate(self.app.arm):
            ang[i] = motor.get_pos()*(pi/18000.)   #convert angles from centidegrees to radians
        self.moveArm.angles = ang
    
    #for moving towwards desired position
    def behavior(self):

        # if it is calibrated, we want to draw the line to the next position using the calibrated values
        if self.calibrated:
            self.pos = append(self.moveArm.ee, [1])
        # set pos to the first calibration point if not calibrated
        else:
            self.pos = self.app.calib_grid[0]
 
        self.syncArm()     
        if self.square == False or len(self.currentPos) == 0:
            #forward kinematics - get current end effector position given joint angles
            self.currentPos = self.app.idealArm.getTool(self.moveArm.angles)


        progress(str(self.pos))
        progress(str(self.currentPos))

        # find the largest angle in degrees
        self.moveArm.ee = self.pos[:3]
        progress('move arm angles' + str(self.moveArm.angles))
        largest_delta = 0
        
        for i,motor in enumerate(self.app.arm):
            delta = abs(rad2deg(self.moveArm.angles[i]) - (motor.get_pos()/100))
            progress('current arm angles'  + str(motor.get_pos()))
            if delta > largest_delta:
                largest_delta = abs(delta)
        progress(str(largest_delta))
        num_steps = maximum(1,int(largest_delta * self.step_constant))

        #Create a number of evenly spaced steps between current position and goal position
        self.steps = linspace(self.currentPos,self.pos,num_steps)[:,:-1]    #Can adjust number of steps.
        #execute movement along path of steps
        for stepCount,step in enumerate(self.steps):
            progress('Step #%d, %s' % (stepCount,str(step)))
            self.app.currStep = stepCount
            progress(step)
            self.moveArm.ee = step
            for i,motor in reversed(list(enumerate(self.app.arm))):
                #Calculate angles to move each step and move the motors 
                motor.set_pos(rad2deg(self.moveArm.angles[i])*100)    #feed in angle to set_pos as centidegrees
            yield self.forDuration(4)
        progress('Move complete')
'''

class MoveInterpolation( Plan ):
    def __init__(self,app,*arg,**kw):
        Plan.__init__(self,app,*arg,**kw)
        self.app = app
        self.points = self.app.calib_grid_paper
        self.calib_ang_b = self.app.calib_ang_b
        self.calib_ang_a = self.app.calib_ang_a
        self.calib_ang_s = self.app.calib_ang_s
        self.square = self.app.square_paper

        self.bottom = self.app.bottom_motor
        self.arm = self.app.arm_motor
        self.string = self.app.string_motor


    def interpolateLocation(self, x, y):
        """
        function that provides angles given destination coords
        """
        #fx = interp1d([0,215.9], [0,XPTS])
        # x = (XPTS-1)/215.9 * xDes        
        # y = (YPTS-1)/279.4 * yDes
        
        # print(xDes,yDes)
        progress(str(self.points))
        progress(str(self.calib_ang_b))
        ba = griddata(self.points, self.calib_ang_b,(x,y))
        aa = griddata(self.points, self.calib_ang_a,(x,y))
        sa = griddata(self.points, self.calib_ang_s,(x,y))
        
        return(ba,aa,sa)
   
    def goToPos(self, x,y):
        """
        moves the arm to the input coords
        """
        angles = self.interpolateLocation(x,y)
        
        self.bottom.set_pos(angles[0])
        self.arm.set_pos(angles[1])
        self.string.set_pos(angles[2])

        return angles
    
    def drawStrokes(self,xi,yi,xf,yf):
        #dist = pow(pow(xi-xf,2)+pow(yi-yf,2),.5)
        #numpoints = 5#floor(dist / (25.4*1.5)) + 1
        #if numpoints < 2:
        #    numpoints = 2
        numpoints = 5
        progress("here")
        xInterpolate = interp1d([0,numpoints-1],[xi,xf])
        yInterpolate = interp1d([0,numpoints-1],[yi,yf])
        
        for i in range(int(numpoints)):
            progress("here")
            angles = self.goToPos(xInterpolate(i), yInterpolate(i))
            progress(str(angles))
        

   #for moving towwards desired position
    def behavior(self):
        pos0 = self.square[0]
        progress('square 0 ' + str(self.square[0]))
        #pos1 = self.square[1]
        #pos2 = self.square[2]
        #pos3 = self.square[3]

        # Move to the last calibration point
        last_calib_point = self.points[-1]
        progress("bottom current: " + str(self.bottom.get_pos()) + " bottom goal:" + str(self.calib_ang_b[-1][-1]))
        progress("arm current: " + str(self.arm.get_pos()) + " arm goal:" + str(self.calib_ang_a[-1][-1]))
        progress("string current: " + str(self.string.get_pos()) + " string goal:" + str(self.calib_ang_s[-1][-1]))
        self.bottom.set_pos(self.calib_ang_b[-1][-1])
        self.arm.set_pos(self.calib_ang_a[-1][-1])
        self.string.set_pos(self.calib_ang_s[-1][-1])
        
        progress('points ' + str(self.points[-1]))

        self.drawStrokes(last_calib_point[0],last_calib_point[1],pos0[0],pos0[1])
        

        progress("line drawn")
        #self.drawStrokes(pos0[0],pos0[1],pos1[0],pos1[1])
        #self.drawStrokes(pos1[0],pos1[1],pos2[0],pos2[1])
        #self.drawStrokes(pos2[0],pos2[1],pos3[0],pos3[1])
        yield