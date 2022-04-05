import numpy as np
from joy.plans import Plan
import os
import json

class record(Plan):
    def __init__(self, app):
        Plan.__init__(self,app)
        self.app = app
        