#!/usr/bin/python

'''
Cloud project:
- Software Defined Networking (SDN) 
-- External interface between POX top controller and other apps such as Mininet

Tony Lin
'''
import time
import os, sys
import logging
#import numpy as np
#import re

from pox.core import core
#import pox.openflow.libopenflow_01 as of
from pox.lib.revent import *
from pox.lib.ioworker import *
from pox.lib.ioworker.workers import *
    
sys.path.append('/home/mininet/Sdn_src/exp/src/')
from utils.proj_util import MemoryDict
from utils.constant_sys import *
from utils.proj_socket import PoxClient
from pox_src.controller import Topo_controller
from pox_src.poxEvent import RatioConfig



# ---------------------------------------------------------------------------
# Server Stuff
# ---------------------------------------------------------------------------

class ServerWorker (TCPServerWorker, RecocoIOWorker):
    pass

clients = set()

class NotifyWorker (RecocoIOWorker):
    def __init__ (self, *args, **kw):
        super(NotifyWorker, self).__init__(*args, **kw)
        self._connecting = True
        self.data = b''

    def _handle_close (self):
        log.info("Client disconnect")
        super(NotifyWorker, self)._handle_close()
        clients.discard(self)

    def _handle_connect (self):
        log.info("Client connect")
        super(NotifyWorker, self)._handle_connect()
        clients.add(self)

    def _handle_rx (self):
        self.data += self.read()
        if CMD_ESCAPE in self.data: # '\n' in received data
            # multiple commands
            while CMD_ESCAPE in self.data:
                msg,self.data = self.data.split(CMD_ESCAPE,1)
                print 'command with escaper: %s' %msg[:-1]
                f_processMnMsg(msg[:-1])  # need to confirm why -1 is needed to remove '\n'
        else: # single command
            print 'single line command'
            f_processMnMsg(self.data)
        self.data = b''

def f_processMnMsg(msg):
    print 'At poxExtIf receive: %s' %(msg)
    #print 'msg[0]=%s, msg[-2]=%s,msg[-1]=%s' %(msg[0],msg[-2],msg[-1])
    if '{' == msg[0] and '}' == msg[-1]: # dict for ratio
        # get ratio dict
        try:
            ratioSetupDict = eval(msg)
        except (SyntaxError,NameError) as e:
            print 'can not eval dict: %s due to error: %s, discard and wait for next message' %(msg,e)
            return False
        # send ratio to each load balancer
        core.PoxInternalIf.f_sendLbRatio(ratioSetupDict)        
        
class PoxInternalIf (EventMixin):
    _eventMixin_events = set([
        RatioConfig
    ])
    def __init__ (self):
        
        self.debug = False
        self.running = True
        self.starting_up = True
        self.components = {'PoxInternalIf':self}
        
    def f_sendLbRatio(self,ratioSetupDict):
        for key_name, ratio in ratioSetupDict.iteritems(): 
            print key_name
            if key_name in core.Topo_controller.switchDpidDict.keys():
                dpid = core.Topo_controller.switchDpidDict[key_name]
                #print 'before core.Topo_controller.switches[%s].LBWR.vlanRatio=%s' %(key_name,str(core.Topo_controller.switches[dpid].LBWR.vlanRatio))
                core.Topo_controller.switches[dpid].LBWR.vlanRatio = ratio
                #core.Topo_controller.switches[dpid].LBWR.wrMaxLen  = 20
                #print 'after =%s' %str(core.Topo_controller.switches[dpid].LBWR.vlanRatio)
        #self.raiseEvent(PoxInitReady(RatioConfig,str(ratioSetupDict)))
    
# ---------------------------------------------------------------------------
# Pox interface to Mininet
# ---------------------------------------------------------------------------
class MnIf (EventMixin):
#     _eventMixin_events = set([
#         RatioConfig
#     ])
    def __init__ (self):
        
        self.debug = False
        self.running = True
        self.starting_up = True
        self.components = {'MnIf':self}
        
        self.diable_items = None
        self.pox_st_machine_on = False
        self.pox_st_machine = None
        
        core.Topo_controller.addListeners(self)
        
        self.clisock = None
        self.clisockConnected = False
    
    def _handle_PoxInitReady (self, event):
        print 'msg:%s received at MnIf' %(event.msg)
        self.diable_items = event.content
        #print self.diable_items
        time.sleep(2)
        self.f_init_connect_mn()
        #self.f_start_pox_stateMachine()

    def f_init_connect_mn(self,msg=INIT_POX_DONE):
        while not self.clisock or not self.clisockConnected: # !!!!!!!!!!!!need to confirm if close status means None
            self.f_creat_mn_socket()
        # try to connect mn server
        response = None
        self.clisock.settimeout(2)  # set socket timeout 1 second for the following operation
        while response != CONN_CONFIRM:
            self.clisock.send(msg)
            try:
                response =  self.clisock.recv( 1024 )
            except socket.timeout:
                response=None 
            print 'wait for MN response'
        self.clisock.close()
        self.clisock = None
        print 'pox connected to MN'        
    def f_creat_mn_socket(self):
        try:
            self.clisock = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
            self.clisock.connect( ('', MN_PORT) )
            self.clisockConnected = True
#                self.clisock = socket.create_connection(('', self.port), 5,('0.0.0.0', 9001))
        except socket.error as e:
            logging.warn('Failed to create connection: %s', e.strerror)
            self.clisockConnected = False
        
            
#     def f_start_pox_stateMachine(self):
#         self.pox_st_machine_on = True
#         self.pox_st_machine =  PoxClient(self.diable_items)  
#         self.pox_st_machine.run()
            
def launch (port = POX_PORT,tf=b''):
    # Send full packets to controller
    core.openflow.miss_send_len = 0xffff
    """
    Starts the component Topo_controller
    """
    core.registerNew(Topo_controller,tf)
    #global SRC_DST_VLAN        
    SRC_DST_VLAN_obj = MemoryDict(mem_time=1000)
    core.register("SRC_DST_VLAN",SRC_DST_VLAN_obj)
    """
    Starts the component MnIf
    """
    core.registerNew(MnIf)
    core.registerNew(PoxInternalIf)    
    """
    Starts the interface component
    """
    global loop
    loop = RecocoIOLoop()
    #loop.more_debugging = True
    loop.start()

    w = ServerWorker(child_worker_type=NotifyWorker, port = int(port))
    loop.register_worker(w)
    