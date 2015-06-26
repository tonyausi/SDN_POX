#!/usr/bin/python

'''
Cloud project:
- Software Defined Networking (SDN) 
-- POX related events

Tony Lin
'''

from pox.core import core
#import pox.openflow.libopenflow_01 as of
from pox.lib.revent import *


class PoxInitReady (Event):
  """
  This is raised by Topo_controller to inform a listener that 
      1) Top level POX controller is ready
      2) underline switch controllers are ready
  """
  def __init__ (self, msg, content):
    Event.__init__(self)
    self.msg = msg
    self.content = content

class RatioConfig (Event):
  """
  This is raised by MnIf to inform underline load balancers that 
      1) new distribution rates need to be reconfigured
  """
  pass
    
