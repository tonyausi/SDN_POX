#!/usr/bin/python

'''
Cloud project:
- Software Defined Networking (SDN) 
-- Top level controller

Tony Lin
'''

from pox.core import core
import pox.openflow.libopenflow_01 as of
from pox.lib.revent import *
from pox.lib.util import dpid_to_str
from pox.lib.packet.ethernet import ethernet, ETHER_BROADCAST

import time
import os, sys, getopt
#import numpy as np
import re
import random
from collections import defaultdict

sys.path.append('/home/mininet/Sdn_src/exp/src/')
#print sys.path
#print os.environ["PYTHONPATH"]
import utils.read_topo as rt
#from utils.l2_learning_arp import LearningSwitch
from pox_src.customed_switchs import L3_project_switch
from utils.proj_util import MemoryDict, f_getTopoFile
from pox_src.stats_collector import handle_flowstats_received,handle_aggregateflowstats_received,handle_portstats_received,timer_func
from utils.proj_socket import PoxClient 
from utils.constant_sys import *
from pox_src.poxEvent import PoxInitReady

#from utils.proj_util import f_logNetBw

    
class Topo_controller (EventMixin):
    _eventMixin_events = set([
        PoxInitReady
    ])
    def __init__ (self,topfile_in):
        tf = f_getTopoFile(topfile_in)
        if tf:
            self.infor_obj = rt.TopoInfor(tf)
        else:
            self.infor_obj = rt.TopoInfor(TOPO_FILE)
        self.switches ={}
        # get dpid for all switches
        self.target_switch = self.infor_obj.dpidSwitchDict.keys()
        self.switchDpidDict = self.infor_obj.switchDpidDict
        self.switchNameList = self.infor_obj.switch_list
        #print self.switchNameList
        # get bwm-ng unwanted interface
        self.diable_items = ','.join(self.switchNameList)+','+SYS_ENTITY
        self.switchAllOn = False
        self.pox_st_machine = None
        self.pox_st_machine_on = False
        # Listen to dependencies
        #def startup ():
        #    core.openflow.addListeners(self, priority=0)
        #core.call_when_ready(startup, ('openflow'))
        core.openflow.addListeners(self)
        ''' event related '''
        self.initEventRaised = False

    def f_create_arp_control (self, event):
        fm = of.ofp_flow_mod()
        fm.priority = ARP_CONTROL_PRIORITY
        fm.match.dl_type = ethernet.ARP_TYPE
        fm.actions.append(of.ofp_action_output(port=of.OFPP_CONTROLLER))
        event.connection.send(fm)
        
    def f_create_switch_connection(self,event):
        dpid = event.dpid
        #print event
        sw = self.switches.get(dpid)
        if sw is None:
            '''get switch infor obj'''
            #print dpid, self.infor_obj.dpidSwitchDict
            sw_name = self.infor_obj.dpidSwitchDict[dpid]
            switchVlan_obj = self.infor_obj.nodeVlanDict[sw_name]
            # New switch
            sw = L3_project_switch(switchVlan_obj)
            sw.connect(event)
            
            #sw = LearningSwitch(event.connection, False)
            #sw.dpid = dpid
            
            self.switches[dpid] = sw            
               
    def f_checkAllUp(self):
        # get dpid for all switches turned on only
        on_switch = self.switches.keys()        
        left = set(self.target_switch)-set(on_switch)
        #print target_switch,on_switch, left
        if not left:
            self.switchAllOn = True
            if not self.initEventRaised:
                self.raiseEvent(PoxInitReady(INIT_POX_DONE,self.diable_items))
                self.initEventRaised = True
            #f_init_client_socket()
#             if not self.pox_st_machine_on: # run only once
#                 self.f_start_pox_stateMachine()
    
    def f_start_pox_stateMachine(self):
        self.pox_st_machine_on = True
        self.pox_st_machine =  PoxClient(self.diable_items)  
        self.pox_st_machine.run()
    
    def _handle_ConnectionUp (self, event):
        '''create OpenFlow controlled ARP mechanism
           do NOT flood before asking controller
        '''
        
        #global IP_MAC
        #if IP_MAC is None:
        #    IP_MAC = self.infor_obj.ip_mac_dict
        self.f_create_arp_control(event)
        '''Init connection between controller and OpenFlow switches'''
        self.f_create_switch_connection(event)
        '''check if all switchs are up and then inform mn to fire traffic'''
        self.f_checkAllUp()

#IP_MAC = None     
def launch (tf=b''):
    # Send full packets to controller
    core.openflow.miss_send_len = 0xffff
    """
    Starts the component
    """
#     t_controller = Topo_controller(tf)
#     print t_controller.infor_obj.switch_list
#     print 'tf=%s' %tf
    core.registerNew(Topo_controller,tf)  # register the class not the object t_controller
    #global SRC_DST_VLAN        
    SRC_DST_VLAN_obj = MemoryDict(mem_time=1000)
    core.register("SRC_DST_VLAN",SRC_DST_VLAN_obj)
    
    """ # tem disable
    from pox.lib.recoco import Timer

    # attach handlers to listeners
    core.openflow.addListenerByName("FlowStatsReceived",
                                    handle_flowstats_received)
    core.openflow.addListenerByName("AggregateFlowStatsReceived",
                                    handle_aggregateflowstats_received)
    core.openflow.addListenerByName("PortStatsReceived",
                                    handle_portstats_received)

    # timer set to execute every five seconds
    Timer(0.1, timer_func, recurring=True)
    """
