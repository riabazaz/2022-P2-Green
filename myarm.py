#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""
import sys
import os
from numpy import linspace,dot,zeros,pi,asarray,meshgrid,ones,c_,save,load,array, rad2deg
from arm import Arm
from joy.decl import KEYDOWN,K_k,K_o, K_DOWN, K_UP, K_a, K_z, K_s, K_x, K_d, K_c, K_ESCAPE, K_f, K_v, K_m
from joy import progress, JoyApp
from move import MoveInterpolation
from motorPlans import * 

class MyArm(JoyApp):
    def __init__(self,x,y,s,arm,string,bottom,easel,*arg,**kw):
      ###
      ### Student team selection -- transform from workspace coordinates to world
      ###

      JoyApp.__init__(self,*arg,**kw)
      

      # calibration parameters
      self.x_step = 1 # cm
      self.y_step = 1 # cm

      self.current_x = 0
      self.current_y = 0

      #first three columns represent axis. Last column represents
      #translation. Adjust last column to adjust workspace placement relative to arm.
      #Base of arm is always at world origin 
      self.bottom_motor = getattr(self.robot.at, bottom)
      self.arm_motor = getattr(self.robot.at, arm)
      self.string_motor = getattr(self.robot.at, string)
      self.easel_motor = getattr(self.robot.at, easel)

      progress("Connecting " + arm + " as left module")
      progress("Connecting " + string + " as right module")
      progress("Connecting " +  bottom + " as bottom module")
      progress("Connecting " +  easel + " as easel module")

      self.square_pos_x = x
      self.square_pos_y = y
      self.square_scale = s
      
      # self.move = Move(self)     #move plan
     
    # Define 4 corners of square in paper coordinates
    # columns(left to right): x,y,z coordinates, 1's
    # scale s represents full length of square side
    def createSquare(self, x,y,s):
      square_p = asarray([
        [x-0.5*s, y+0.5*s, 0, 1],     #upper left
        [x+0.5*s, y+0.5*s, 0, 1],     #upper right
        [x+0.5*s, y-0.5*s, 0, 1],     #bottom right
        [x-0.5*s, y-0.5*s, 0, 1]      #bottom left
        ])

      #Convert all coordinates for square to world coordinates
      return square_p # dot(square_p, self.Tp2w.T),

    #Create calibration grid on paper. These are points to move to during calibration.
    def createGrid(self, x_spacing, y_spacing):
      # create the array for the x points
      x_lin = [0]
      while x_lin[-1]+x_spacing < 20: # 20cm is roughly 8inch
        x_lin.append(x_lin[-1]+x_spacing) # *0.394 
      
      # create the array for the y points
      y_lin = [0]
      while y_lin[-1]+y_spacing < 20: # 27cm is roughly 11inch 
        y_lin.append(y_lin[-1]+y_spacing) 

      nx, ny = (len(x_lin),len(y_lin))     #can be adjusted to add more calibration points
      xv,yv = meshgrid(x_lin,y_lin,indexing='xy')
      grid = c_[xv.reshape(nx*ny,1), yv.reshape(nx*ny,1), zeros((nx*ny,1)), ones((nx*ny,1))]
      progress(str(grid))
      grid_idx = list(range(nx*ny))
      for i in range(nx-(nx % 2)):
          idx = nx*(2*i+1)
          grid_idx[idx:idx+nx] = grid_idx[idx:idx+nx][::-1]
      progress(str(nx))
      progress(str(ny))
      return grid, nx, ny #dot(grid,self.Tp2w.T),
      
    def onStart(self):
      #ArmAnimatorApp.onStarcalib_gridt(self)
      self.calib_grid_paper, self.nx, self.ny = self.createGrid(x_spacing=8,y_spacing=8) # self.calib_grid_world
      progress(str(self.nx))
      progress(str(self.ny))
      self.calib_idx = 0
      self.square_paper = self.createSquare(self.square_pos_x, self.square_pos_y,self.square_scale)
      #if calibration file exists, load calibration array in here, and skip over next part
      #also set calibrated == true so that it calculates offset
      #manually delete existing calibration array file before moving on to new arena
      if(os.path.exists("calib_ang_b.npy")):
          self.calib_ang_b = load("calib_ang_b.npy")
          self.calib_ang_a = load("calib_ang_a.npy")
          self.calib_ang_s = load("calib_ang_s.npy")
      else:
          #This is the matrix you save your angles in and use to calculate angle offset
          self.calib_ang_b = zeros((self.nx, self.ny)) # bottom motor angle array
          self.calib_ang_a = zeros((self.nx, self.ny)) # arm motor angle array
          self.calib_ang_s = zeros((self.nx, self.ny)) # string motor angle array
      
      self.br = BottomRight(self)  
      self.bl = BottomLeft(self)
      self.ar = ArmDown(self)
      self.al = ArmUp(self)
      self.sr = StringRight(self)
      self.sl = StringLeft(self)
      self.er = EaselRight(self)
      self.el = EaselLeft(self)

      self.move = MoveInterpolation(self)

    def onEvent(self,evt):
      if evt.type == KEYDOWN:
        # SIMULATION ONLY
        if evt.key == K_DOWN:
          progress('moved down a row')
          self.current_y -= 1
          self.current_x = 0
          if self.current_y < 0:
            self.current_y = 0
          return
        # SIMULATION ONLY
        if evt.key == K_UP:
          progress('moved up a row')
          self.current_y += 1
          self.current_x = 0
          return
        
        if evt.key == K_m:
          self.move.start()
          progress('Move plan started!')

        #Press 'o' to store new angle
        if evt.key == K_o:
            progress('storing angle postion')

            # store all current angles motors and store the angles
            self.calib_ang_b[self.current_x][self.current_y] = self.bottom_motor.get_pos() # get current bottom motor angle
            self.calib_ang_a[self.current_x][self.current_y] = self.arm_motor.get_pos() # get current arm motor angle
            self.calib_ang_s[self.current_x][self.current_y] = self.string_motor.get_pos() # get current string motor angle

            progress('angle stored!')

            if self.current_y == self.ny - 1 and self.current_x == self.nx - 1:
              progress('fully calibrated')
              
              save("calib_ang_b.npy", self.calib_ang_b)
              save("calib_ang_a.npy", self.calib_ang_a)
              save("calib_ang_c.npy", self.calib_ang_c)

            elif self.current_x == self.nx - 1:
              self.current_x = 0
              self.current_y += 1
              progress('Go to next point up, far left')
            else:
              progress('Go to next point to the right')
              self.current_x += 1
            return
          # Manual movements
          # row of 'a' on QWERTY keyboard increments motors
        if evt.key == K_a and not (self.br.isRunning() or self.bl.isRunning()):
          self.br.start()

        elif evt.key == K_s and not (self.ar.isRunning() or self.al.isRunning()):
          self.ar.start()

        elif evt.key == K_d and not (self.sr.isRunning() or self.sl.isRunning()):
          self.sr.start()

        # row of 'z' in QWERTY keyboard decrements motors
        elif evt.key == K_z and not (self.br.isRunning() or self.bl.isRunning()):
          self.bl.start()

        elif evt.key == K_x and not (self.ar.isRunning() or self.al.isRunning()):
          self.al.start()
          
        elif evt.key == K_c and not (self.sr.isRunning() or self.sl.isRunning()):
          self.sl.start()
        
        elif evt.key == K_f and not (self.er.isRunning() or self.er.isRunning()):
          self.el.start()
          
        elif evt.key == K_v and not (self.er.isRunning() or self.el.isRunning()):
          self.er.start()

        if evt.key == K_ESCAPE:
          progress("Exiting program. Have a nice day!")
          exit(0)
            
      return JoyApp.onEvent(self,evt)


if __name__=="__main__": 
  from sys import argv, stdout, exit
  #give default values to the command line arguments
  robot = None
  bottom_motor = "#bottom "
  arm_motor = "#arm "
  string_motor = "#string "
  easel_motor = "#easel "

  #process the command line arguments
  args = list(argv[1:])
  while args:
    arg = args.pop(0)

    if arg=='--mod-count' or arg=='-c':
    #detects number of modules specified after -c
      N = int(args.pop(0))
      robot = dict(count=N)
    
    elif arg=='--bottom' or arg=='-b':
      bottom_motor = args.pop(0)

    elif arg=='--arm' or arg=='-a':
      arm_motor = args.pop(0)

    elif arg=='--string' or arg=='-s':
      string_motor = args.pop(0)

    elif arg=='--easel' or arg=='-e':
      easel_motor = args.pop(0)

    elif arg=='--help' or arg == '-h':
    #help
      stdout.write("""
  Usage: %s [options]
    This program controls forward movement, turning, and moving
    autonomously to waypoints.
    
    Command Line Options:
      --mod-count <number> | -c <number>
        Search for specified number of modules at startup
      --arm <motor | -a <motor>
      --string <motor> | -s <motor>
      --bottom <motor> | -b <motor>
        Specify the motors used for moving and turret
        Ex command:
        Currently use : $ python3 myarm.py -c 3 -a Nx11 -s Nx17 -b Nx32 -e Nx3C
        Testing :       $ python3 myarm.py -c 3 -a Nx14 -s NxE9 -b Nx3C

        NOTE: to use robot modules you MUST specify a -c option
    """ % argv[0])
      exit(1)
  x,y,s = 4,8,2
    #Initial test
    # 
  app = MyArm(x,y,s, arm_motor, string_motor, bottom_motor, easel_motor, robot=robot
                    ## Uncomment the next line (cfg=...) to save video frames;
                    ## you can use the frameViewer.py program to view those
                    ## frames in real time (they will not display locally)
                    #      cfg=dict(logVideo="f%04d.png")
                    )
  app.run()