#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""
from curses import KEY_DOWN
from joy.decl import progress, KEYDOWN, K_UP, K_DOWN, KEYUP
from numpy import asarray
from p2sim import ArmAnimatorApp

class MyArmSim(ArmAnimatorApp):
    def __init__(self,Tp2ws,**kw):
      ###
      ### Student team selection -- transform from workspace coordinates to world
      ###
      Tws2w = asarray([
           [1,0,0, 0],
           [0,1,0, -10],
           [0,0,1,0],
           [0,0,0,  1]
      ])
      ###
      ### Arm specification
      ###
      armSpec = asarray([
        [0,0,1,5,0], #the base rotating left/right
        [1,0,0,5,0.785], #the base rotating up/down
        [1,0,0,5,0] #the arm extending/unextending 
      ]).T
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
    
    def onEvent(self,evt):
      # Ignore everything except keydown events
      if evt.type == KEYDOWN:
        progress("key was pressed")
        # row of 'a' on QWERTY keyboard increments motors
        if evt.key == K_UP:
          self.arm[0].set_pos(self.arm[0].get_goal() + 500)
          return
        elif evt.key == K_DOWN:
          self.arm[0].set_pos(self.arm[0].get_goal() - 500)
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
