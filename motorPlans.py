from joy import JoyApp, progress
from joy.decl import *
from joy.plans import Plan
import math
from math import pi
import numpy as np
from numpy import linspace


DELTA = 100

# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# This plan handles all bottom motor turns.
# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
class BottomRight(Plan):
  def __init__(self,app,*arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.bottom = self.app.bottom_motor

  def onStop(self):
    pass

  def behavior(self):
    mb = self.bottom.get_pos()
    self.bottom.set_pos(mb - DELTA)
    yield 0.005


class BottomLeft(Plan):
  def __init__(self,app,*arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.bottom = self.app.bottom_motor

  def onStop(self):
    pass

  def behavior(self):
    mb = self.bottom.get_pos()
    # for k in linspace(0, 10, 1):
    self.bottom.set_pos(mb + DELTA)
    yield 0.005




# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# This plan handles all arm motor turns.
# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-


class ArmRight(Plan):
  def __init__(self,app,*arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.arm = self.app.arm_motor

  def onStop(self):
    pass

  def behavior(self):
    ma = self.arm.get_pos()
    # for k in linspace(0, 5, 1):
    self.arm.set_pos( (ma - (DELTA)))
    yield 0.005


    
class ArmLeft(Plan):
  def __init__(self,app,*arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.arm = self.app.arm_motor

  def onStop(self):
    pass

  def behavior(self):
    ma = self.arm.get_pos()
    self.arm.set_pos( (ma + (DELTA)))
    yield 0.005


# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# This plan handles all string motor turns.
# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

class StringRight(Plan):
  def __init__(self,app,*arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.string = self.app.string_motor

  def onStop(self):
    pass

  def behavior(self):
    ms = self.string.get_pos()
    # for k in linspace(0, 5, 1):
    self.string.set_pos( (ms - (DELTA)))
    yield 0.005
    
class StringLeft(Plan):
  def __init__(self,app,*arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.string = self.app.string_motor

  def onStop(self):
    pass

  def behavior(self):
    ms = self.string.get_pos()
    # for k in linspace(0, 5, 1):
    self.string.set_pos( (ms + (DELTA)))
    yield 0.005


# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# This plan handles all easel motor turns.
# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

class EaselRight(Plan):
  def __init__(self,app,*arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.easel = self.app.easel_motor

  def onStop(self):
    pass

  def behavior(self):
    ms = self.easel.get_pos()
    # for k in linspace(0, 5, 1):
    self.easel.set_pos( (ms - (DELTA)))
    yield 0.005
    
class EaselLeft(Plan):
  def __init__(self,app,*arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.easel = self.app.easel_motor

  def onStop(self):
    pass

  def behavior(self):
    ms = self.easel.get_pos()
    # for k in linspace(0, 5, 1):
    self.easel.set_pos( (ms + (DELTA)))
    yield 0.005