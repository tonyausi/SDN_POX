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
from pox.lib.recoco import Timer
from collections import defaultdict
from pox.openflow.discovery import Discovery
from pox.lib.util import dpid_to_str
from pox.lib.addresses import IPAddr
import time

import os, sys, getopt
import numpy as np
import re

#sys.path.append('/home/Sdn_src/exp/src/')
#print sys.path
#print os.environ["PYTHONPATH"]
import utils.read_topo as rt
TYPE_IP = 0x0800
TYPE_ARP= 0x0806

log = core.getLogger()

topo_file = "%s/Sdn_src/exp/src/topo_config/nat.csv" % os.environ[ 'HOME' ]  
 
'''
sys.path.append('../')
from utils import read_topo
'''

'''
def simulate(topo_fname):
    topo_data = read_topo.f_read_topo(topo_fname)
    #print topo_data
    vlan_set = read_topo.f_get_vlan_set(topo_data)

def main(argv):   
    try:
        opts, args = getopt.getopt(argv, "hd", ["help"])
    except getopt.GetoptError:
        print "custom <topo.csv>"
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            print "custom <topo.csv>"
            sys.exit()
        elif opt == '-h':
            global _debug
            _debug = 1
    try:
        topo_fname = args[0]
        simulate(topo_fname)
      
    except ValueError:
        print "can not open <topo.csv>"
        sys.exit(2)  
     
if __name__ == "__main__":
    main(sys.argv[1:])
'''
class L3_switch(object): # (EventMixin):
    def __init__ (self,switchVlan_obj):
        self.static_config_finished = False
        self.event      = None        
        self.connection = None
        self.ports = None  #???????????????
        self.dpid = None
        self._listeners = None
        self._connected_at = None
        self.switchVlan_obj = switchVlan_obj

    def connect (self, event):
        self.event = event
        connection = self.event.connection
        if self.dpid is None:
            self.dpid = connection.dpid
        assert self.dpid == connection.dpid
        self.connection = connection
        #self._listeners = self.listenTo(connection)
        #self._connected_at = time.time()
        print "Switch with dpid=%d is invoked by Connect %s" % (self.dpid,connection,)
        print self.switchVlan_obj.name
        print self.switchVlan_obj.match_dict
        print self.switchVlan_obj.nat_dict
        print self.switchVlan_obj.vlan_nat_options
        log.debug("Switch with dpid=%d is invoked by Connect %s" % (self.dpid,connection,))
        
        self.inject_match_flows()
        self.inject_nat_flows()

    def inject_match_flows (self):
        if self.switchVlan_obj.nat_enable:
            return
        match_dict = self.switchVlan_obj.match_dict
        cookie = 9
        for (src_ip,dst_ip),forword_port in match_dict.iteritems():
            #forword_port = match_dict[(src_ip,dst_ip)]
            print src_ip,dst_ip,forword_port
            '''set for IP'''
            self.inject_single_src_dst_nat_flow(cookie,src_ip,dst_ip,forword_port)
            cookie = cookie + 1
            '''set for arp'''
            self.inject_single_src_dst_nat_flow(cookie,src_ip,dst_ip,forword_port,
                                                dl_type=TYPE_ARP)
            cookie = cookie + 1
            '''
            msg = of.ofp_flow_mod()
            msg.match.dl_type = 0x800
            msg.match.nw_src=IPAddr(src_ip)
            msg.match.nw_dst=IPAddr(dst_ip)
            msg.actions.append(of.ofp_action_output(port = forword_port))
            msg.cookie = cookie
            #msg.data = self.event.ofp
            #msg.buffer_id = None # ????????????????????????self.event.ofp.buffer_id
            self.connection.send(msg)
            '''
                        
    def inject_nat_flows (self):
        if not self.switchVlan_obj.nat_enable:
            return
        nat_dict = self.switchVlan_obj.nat_dict
        match_dict = self.switchVlan_obj.match_dict
        cookie = 9
        for (src_ip,dst_ip),(new_src_ip,new_dst_ip) in nat_dict.iteritems():
            forword_port = match_dict[(src_ip,dst_ip)]
            print (src_ip,dst_ip),(new_src_ip,new_dst_ip),forword_port
            '''set for IP'''
            self.inject_single_src_dst_nat_flow(cookie,src_ip,dst_ip,forword_port,
                                                       new_src_ip,new_dst_ip)
            cookie = cookie +1
            '''set for arp'''
            self.inject_single_src_dst_nat_flow(cookie,src_ip,dst_ip,forword_port,
                                                       new_src_ip,new_dst_ip,
                                                       dl_type=TYPE_ARP)
            cookie = cookie +1
            
        '''need to implement load balance here !!!!!!!!!!!!!!!'''
        print self.switchVlan_obj.vlan_nat_src_pair
        print self.switchVlan_obj.vlan_ports
        self.test_single_nat(cookie)
        cookie = cookie +2
        
    def inject_single_src_dst_nat_flow(self,cookie,src_ip,dst_ip,forword_port,
                                                   new_src_ip=None,new_dst_ip=None,
                                                   dl_type = TYPE_IP):
        ''' inject single src_ip dst_ip / new_src_ip new_dst_ip nat with forwarding port'''
        msg = of.ofp_flow_mod()
        msg.match.dl_type = dl_type
        msg.match.nw_src=IPAddr(src_ip)
        msg.match.nw_dst=IPAddr(dst_ip)
        if new_src_ip:
            msg.actions.append(of.ofp_action_nw_addr.set_src(IPAddr(new_src_ip)))
        if new_dst_ip:
            msg.actions.append(of.ofp_action_nw_addr.set_dst(IPAddr(new_dst_ip)))
        msg.actions.append(of.ofp_action_output(port = forword_port))
        msg.cookie = cookie
        self.connection.send(msg)
    
    def test_single_nat(self,cookie):
        ''' test for 1st route '''
        vlan_keys  = self.switchVlan_obj.vlan_nat_options.keys()
        vlan_keys.sort()
        vlan_first = vlan_keys[0]
        print vlan_first
        (src_ip,dst_ip) = self.switchVlan_obj.vlan_nat_src_pair[vlan_first]
        (new_src_ip,new_dst_ip) = self.switchVlan_obj.vlan_nat_options[vlan_first]
        forword_port = self.switchVlan_obj.vlan_ports[vlan_first][rt.DST]
        print (src_ip,dst_ip),(new_src_ip,new_dst_ip),forword_port
        '''set for IP'''
        self.inject_single_src_dst_nat_flow(cookie,src_ip,dst_ip,forword_port,
                                                   new_src_ip,new_dst_ip)
        '''set for ARP'''
        self.inject_single_src_dst_nat_flow(cookie,src_ip,dst_ip,forword_port,
                                                   new_src_ip,new_dst_ip,
                                                   dl_type=TYPE_ARP)
        
    
class Topo_controller(object): #(EventMixin):
    def __init__ (self):
        self.infor_obj = rt.TopoInfor(topo_file)
        self.switches ={}
        # Listen to dependencies
        #def startup ():
        #    core.openflow.addListeners(self, priority=0)
        #core.call_when_ready(startup, ('openflow'))
        core.openflow.addListeners(self)

    def _handle_ConnectionUp (self, event):
        dpid = event.dpid
        print event
        sw = self.switches.get(dpid)
        if sw is None:
            '''get switch infor obj'''
            sw_name = self.infor_obj.dpidSwitchDict[dpid]
            switchVlan_obj = self.infor_obj.nodeVlanDict[sw_name]
            # New switch
            sw = L3_switch(switchVlan_obj)
            self.switches[dpid] = sw            
            sw.connect(event)
        

def launch ():
  """
  Starts the component
  """
  core.registerNew(Topo_controller)
