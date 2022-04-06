#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""
from curses import KEY_DOWN, KEY_LEFT
from joy.decl import progress, KEYDOWN, K_UP, K_DOWN, K_LEFT, K_RIGHT
from numpy import asarray
from p2sim import ArmAnimatorApp
from math import pi
import numpy as np

class MyArmSim(ArmAnimatorApp):
    def __init__(self,Tp2ws,**kw):
      ###
      ### Student team selection -- transform from workspace coordinates to world
      ### 
      Tws2w = asarray([
           [1,0,0, -5], 
           [0,1,0, -12],
           [0,0,1,0],
           [0,0,0,  1]
      ])
      ###

      self.angle_delta = 500

      ### Arm specification
      ###
      armSpec = asarray([
        [0,0.0001,1,0,-1.57], # base rotation around the z-axis
        [0,1,0,5,-1.57], # arm rotation around the y-axis #1.57
        [0,1,0,5,0] #the arm extending/unextending 
      ]).T

      # squareTarget = []
      # scale = 2.5

      self.calibMotorCoords = []

      ArmAnimatorApp.__init__(self,armSpec,Tws2w,Tp2ws,
        simTimeStep=0.1, # Real time that corresponds to simulation time of 0.1 sec
        **kw
      )

    def show(self,fvp):
      fvp.plot3D([0],[0],[0],'^k',ms=10) # Plot black triangle at origin
      return ArmAnimatorApp.show(self,fvp)

    def onStart(self):
      ArmAnimatorApp.onStart(self)
      ###
      ### TEAM CODE GOES HERE
      ###

    ###
    ### TEAM event handlers go here
    ###    Handle events as you see fit, and return after
    def on_K_r(self,evt):
      progress("(say) r was pressed")
    
    def move_until_correct(self, index):
      delta = -500
      while (self.arm[index].get_pos() < self.arm[index].get_goal() - self.angle_delta) or (self.arm[index].get_pos() > self.arm[index].get_goal() + self.angle_delta):
        self.arm[index].set_pos(self.arm[index].get_goal() + delta)
        delta = delta*-1
        progress("current pos: " + str(self.arm[index].get_pos()))
        progress("goal pos: " + str(self.arm[index].get_goal()))
      return


    def onEvent(self,evt):
      # Ignore everything except keydown events
      progress(str(self.arm[0].get_pos()))
      if evt.type == KEYDOWN:

        if evt.key == K_UP:
          # perform calibration
          self.calibrate.start()
          
        if evt.key == K_DOWN:
          # save calibration points
          #self.calibMotorCoords.append(np.array([self.app.arm[x].get_goal()*(pi/18000) for x in range(3)]))
          progress("goal angles: " + str(self.arm[0].get_goal()) 
            + ", " + str(self.arm[1].get_goal()) 
            + ", " + str(self.arm[2].get_goal()))
          progress("current angles: " + str(self.arm[0].get_pos()) 
            + ", " + str(self.arm[1].get_pos()) 
            + ", " + str(self.arm[2].get_pos()))
          return

        if evt.key == K_RIGHT:
          progress("adjusting")
          self.move_until_correct(0)
          self.move_until_correct(1)
          self.move_until_correct(2)


        if evt.key == K_LEFT:
          for coor in self.calibMotorCoords:
            for i in range(3):
              self.arm[i].set_pos(coor[i])
          return

        p = "asd".find(evt.unicode)
        if p>=0:
          self.arm[p].set_pos(self.arm[p].get_goal() + 500)
          return
        # row of 'z' in QWERTY keyboard decrements motors
        p = "zxc".find(evt.unicode)
        if p>=0:
          self.arm[p].set_pos(self.arm[p].get_goal() - 500)
          return
      return ArmAnimatorApp.onEvent(self,evt)
      ## disable this block (change to 0) to use on_K for these keys
      '''
      # if point in calibration grid reached
      if evt.key == K_r:
        self.calibrate.start()
        progress("r key was pressed")
      '''
      return ArmAnimatorApp.onEvent(self,evt)

if __name__=="__main__":
  # Transform of paper coordinates to workspace
  Tp2ws = asarray([
       [0.7071,0,-0.7071,0],
       [0,     1,      0,0],
       [0.7071,0, 0.7071,0],
       [0,     0,      0,1]
  ])
  Tp2ws=asarray([[  1.  ,   0.  ,   0.  ,   0.16],
       [  0.  ,   0.71,   0.71,   1.92],
       [  0.  ,  -0.71,   0.71,  10.63],
       [  0.  ,   0.  ,   0.  ,   1.  ]])
  app = MyArmSim(Tp2ws,
     ## Uncomment the next line (cfg=...) to save video frames;
     ## you can use the frameViewer.py program to view those
     ## frames in real time (they will not display locally)
     # cfg=dict(logVideo="f%04d.png")
    )
  app.run()
