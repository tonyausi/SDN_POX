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
from pox.openflow.discovery import Discovery
from pox.lib.util import dpid_to_str
from pox.lib.packet.ethernet import ethernet, ETHER_BROADCAST
from pox.lib.packet.ipv4 import ipv4
from pox.lib.packet.arp import arp
from pox.lib.addresses import IPAddr, EthAddr, IP_ANY, IP_BROADCAST
#from pox.forwarding.l2_learning import LearningSwitch
from pox.misc.arp_helper import send_arp_reply

import time
import os, sys, getopt
import numpy as np
import re
import random
from collections import defaultdict

#sys.path.append('/home/Sdn_src/exp/src/')
#print sys.path
#print os.environ["PYTHONPATH"]
import utils.read_topo as rt
from utils.l2_learning_arp import LearningSwitch
TYPE_IP = 0x0800
TYPE_ARP= 0x0806
ARP_CONTROL_PRIORITY = 0x0007 # Pretty high
DUMMY_MAC = EthAddr('F0:F0:F0:F0:F0:F0')

log = core.getLogger()

topo_file = "%s/Sdn_src/exp/src/topo_config/nat.csv" % os.environ[ 'HOME' ]  
 
class L3_switch(EventMixin):  #   (object): # 
    def __init__ (self,switchVlan_obj):
        self.static_config_finished = False
        self.event      = None   
        self.packet_event = None     
        self.connection = None
        self.ports = None  #???????????????
        self.dpid = None
        self._listeners = None
        self._connected_at = None
        self.switchVlan_obj = switchVlan_obj
        self.vlans = self.switchVlan_obj.vlans
        self.vlan_choice = None
        self.dns_enable = self.switchVlan_obj.nat_enable
        self.vlt_enable = self.switchVlan_obj.vlt_enable
        if self.dns_enable:
            self.dns_choice_list = self.switchVlan_obj.vlan_nat_options.keys()
        self.arp_choice_done = False    
        self.counter = 0

    def connect (self, event):
        self.event = event
        connection = self.event.connection
        if self.dpid is None:
            self.dpid = connection.dpid
        assert self.dpid == connection.dpid
        self.connection = connection
        # We want to hear PacketIn messages, so we listen
        # to the connection
        self._listeners = self.listenTo(connection)
        self._connected_at = time.time()        
        #connection.addListeners(self)
        print "Switch with dpid=%d is invoked by Connect %s" % (self.dpid,connection,)
        '''
        print self.switchVlan_obj.name
        print self.switchVlan_obj.match_dict
        print self.switchVlan_obj.nat_dict
        print self.switchVlan_obj.vlan_nat_options
        '''
        log.debug("Switch with dpid=%d is invoked by Connect %s" % (self.dpid,connection,))
        
        self.inject_match_flows()
        self.inject_nat_flows()

    def inject_match_flows (self):
        if self.dns_enable:
            return
        match_dict = self.switchVlan_obj.match_dict
        cookie = 9
        for (src_ip,dst_ip),forword_port in match_dict.iteritems():
            #forword_port = match_dict[(src_ip,dst_ip)]
            #print src_ip,dst_ip,forword_port
            '''set for ICMP'''
            self.inject_single_match_flow(cookie,src_ip,dst_ip,forword_port)
            cookie = cookie + 1
            '''set for TCP'''
            self.inject_single_match_flow(cookie,src_ip,dst_ip,forword_port,nw_proto = ipv4.TCP_PROTOCOL)
            cookie = cookie + 1

                        
    def inject_nat_flows (self):
        if not self.dns_enable:
            return
        nat_dict = self.switchVlan_obj.nat_dict
        match_dict = self.switchVlan_obj.match_dict
        cookie = 9
        for (src_ip,dst_ip),(new_src_ip,new_dst_ip) in nat_dict.iteritems():
            forword_port = match_dict[(src_ip,dst_ip)]
            #print (src_ip,dst_ip),(new_src_ip,new_dst_ip),forword_port
            '''set for ICMP'''
            self.inject_single_match_flow(cookie,src_ip,dst_ip,forword_port,
                                                 new_src_ip,new_dst_ip)
            cookie = cookie +1
            '''set for TCP'''
            self.inject_single_match_flow(cookie,src_ip,dst_ip,forword_port,
                                          new_src_ip,new_dst_ip,nw_proto = ipv4.TCP_PROTOCOL)
            cookie = cookie + 1

        '''need to implement load balancing within _handle_PacketIn !!!!!!!!!!!!!!!'''

        #self.test_single_nat(cookie)
        cookie = cookie +2
        
    def inject_single_match_flow(self,cookie,src_ip,dst_ip,forword_port,
                                             new_src_ip=None,new_dst_ip=None,
                                             dl_type = TYPE_IP,
                                             nw_proto = ipv4.ICMP_PROTOCOL):
        ''' inject single src_ip dst_ip / new_src_ip new_dst_ip nat with forwarding port'''
        msg = of.ofp_flow_mod()
        msg.match.dl_type = dl_type
        msg.match.nw_proto = nw_proto
        #print src_ip
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
        self.vlan_choice = vlan_keys[-1]
        print self.vlan_choice
        (src_ip,dst_ip) = self.switchVlan_obj.vlan_nat_src_pair[self.vlan_choice]
        (new_src_ip,new_dst_ip) = self.switchVlan_obj.vlan_nat_options[self.vlan_choice]
        forword_eth_port = self.switchVlan_obj.vlan_ports[self.vlan_choice][rt.DST]
        forword_port = self.switchVlan_obj.ethPortMap[forword_eth_port]
        #print (src_ip,dst_ip),(new_src_ip,new_dst_ip),forword_port
        '''set for ICMP'''
        self.inject_single_match_flow(cookie,src_ip,dst_ip,forword_port,
                                             new_src_ip,new_dst_ip)
        '''set for TCP'''
        self.inject_single_match_flow(cookie,src_ip,dst_ip,forword_port,
                                             new_src_ip,new_dst_ip,nw_proto = ipv4.TCP_PROTOCOL)
        

    def _handle_PacketIn (self, event):
        #'''
        def drop ():
            if event.ofp.buffer_id is not None:
                # Kill the buffer
                msg = of.ofp_packet_out(data = event.ofp)
                self.con.send(msg)
            return None
        #'''
        packet = event.parsed
        
        icmpp = packet.find("icmp")
        tcpp = packet.find("tcp")
        ipp  = packet.find("ipv4")
        #print icmpp,tcpp,ipp  #, packet.payload.prototype
        if (not icmpp) and (not tcpp):  #       not ipp: #
            arpp = packet.find("arp")
            if not arpp: return None #drop ()
            print 'start ARP process'
            #print 'For ARP, event.dpid=', event.dpid
            #print arpp.prototype,'?=', arp.PROTO_TYPE_IP
            #print arpp.hwtype ,'?=',  arp.HW_TYPE_ETHERNET        
            #print 'self.dpid=', self.dpid
            self.packet_event = event
            self.f_arp_process(arpp)
            return
        #'''
        if icmpp:
            print 'start ICMP process'
            ipp = packet.find('ipv4')
            packet_in = event.ofp # The actual ofp_packet_in message.
            self.f_icmp_process(ipp,packet_in)    
        #'''  
        if tcpp:
            print 'start TCP process'
            ipp = packet.find('ipv4')
            packet_in = event.ofp # The actual ofp_packet_in message.
            self.f_tcp_process(ipp,packet_in)  
        #'''
            
    def f_arp_process(self,arpp):
        if arpp.opcode == arp.REQUEST:
            self.f_arp_reply(arpp)
        elif arpp.opcode == arp.REPLY:
            print "It's a reply; do something cool (Not supported yet)"
            #'''
            print 'arpp.next=',arpp.next
            print 'arpp.protosrc=',arpp.protosrc
            print 'arpp.protodst=',arpp.protodst
            #'''
            sys.exit()
        else:
            print "Some other ARP opcode, probably do something smart here"    
            
    def f_arp_reply(self,arpp):
        #print "It's a arp request"
        '''
        #print 'arpp.next=',arpp.next
        #print 'arpp.protosrc=',arpp.protosrc
        #print 'arpp.protodst=',arpp.protodst
        '''
        dstIP  = arpp.protodst.toStr()
        #print 'dstIP=',dstIP
        dstMac = self.f_get_arp_mac(dstIP)  
        #print 'dstMac=', dstMac
        if dstMac != None:
            arp_reply = arp()
            arp_reply.opcode = arp.REPLY
            arp_reply.hwdst = arpp.hwsrc
            arp_reply.protodst = arpp.protosrc
            arp_reply.protosrc = arpp.protodst
            arp_reply.hwsrc = dstMac
            ether = ethernet()
            ether.type = ethernet.ARP_TYPE
            ether.src = dstMac
            ether.dst = arpp.hwsrc            
            ether.payload = arp_reply
            #send this packet to the switch
            self.f_send_arp_reply_pkt(ether,self.packet_event.port) 
            #'''
        else:
            '''Send Openflow arp flood via Openflow flood port NOT ethernet flood !!!!!!!!'''
            self.f_send_arp_openflow_flood()          

    def f_get_arp_mac(self,dstIP):
        #print 'self.switchVlan_obj.name = ', self.switchVlan_obj.name
        #print 'self.vlan_choice=', self.vlan_choice
        #print 'self.switchVlan_obj.dstVIP[self.vlan_choice]=', self.switchVlan_obj.dstVIP[self.vlan_choice]
        if dstIP in self.switchVlan_obj.ip_mac_dict.keys():
            #print self.switchVlan_obj.ip_mac_dict[dstIP]
            return EthAddr(self.switchVlan_obj.ip_mac_dict[dstIP])
        elif self.vlan_choice and dstIP == self.switchVlan_obj.dstVIP[self.vlan_choice]:
            '''get physical dst IP string based on VLAN decision'''
            ip_str = self.switchVlan_obj.dstIP[self.vlan_choice]
            return EthAddr(self.switchVlan_obj.ip_mac_dict[ip_str])
        elif not self.vlan_choice:
            self.vlan_choice = self.f_LB_decision()
            self.arp_choice_done = True 
            self.counter = 0
            ip_str = self.switchVlan_obj.dstIP[self.vlan_choice]
            return EthAddr(self.switchVlan_obj.ip_mac_dict[ip_str])        
        else:
            return DUMMY_MAC #ETHER_BROADCAST

    def f_send_arp_reply_pkt(self,arp_ether,outport):
        msg = of.ofp_packet_out(in_port=of.OFPP_NONE)
        msg.data = arp_ether.pack()
        msg.actions.append(of.ofp_action_output(port = outport))
        #msg.buffer_id = <some buffer id, if any>
        self.connection.send(msg)
        
    def f_send_arp_openflow_flood(self):
        msg = of.ofp_packet_out()
        msg.actions.append(of.ofp_action_output(port = of.OFPP_FLOOD))
        msg.data = self.packet_event.ofp
        self.packet_event.connection.send(msg.pack()) 
    
    def f_icmp_process(self,ipp,packet_in): #  event_icmp):    #
        '''get Load balance decision'''
        self.vlan_choice = self.f_LB_decision()
        print 'self.vlan_choice =',self.vlan_choice 
        '''forward packet by changing src/dst ip 
        without installing matching rule'''
        self.f_dispatch_packet(ipp, packet_in)        
        
    def f_LB_decision(self):
        '''random pick'''
        #return random.choice(self.switchVlan_obj.vlan_nat_options.keys())
        '''evenly pick'''
        length = len(self.dns_choice_list)
        #self.counter = self.counter + 1
        return self.dns_choice_list[self.counter % length]
         
            
    def f_dispatch_packet(self,ipp,packet_in):
        (src_ip,dst_ip) = self.switchVlan_obj.vlan_nat_src_pair[self.vlan_choice]        
        
        if ipp.srcip == src_ip and ipp.dstip == dst_ip:
            (new_src_ip,new_dst_ip) = self.switchVlan_obj.vlan_nat_options[self.vlan_choice]
            forword_eth_port = self.switchVlan_obj.vlan_ports[self.vlan_choice][rt.DST]
            forword_port = self.switchVlan_obj.ethPortMap[forword_eth_port] 
            
            self.f_nat_forward_packet(packet_in,new_src_ip,new_dst_ip,forword_port)
        
    def f_nat_forward_packet(self,packet_in,new_src_ip,new_dst_ip,forword_port):    
        msg = of.ofp_packet_out(in_port=packet_in.in_port)               
        msg.data = packet_in
        if new_src_ip:
            msg.actions.append(of.ofp_action_nw_addr.set_src(IPAddr(new_src_ip)))
        if new_dst_ip:
            msg.actions.append(of.ofp_action_nw_addr.set_dst(IPAddr(new_dst_ip)))
        msg.actions.append(of.ofp_action_output(port = forword_port))
        #msg.buffer_id = <some buffer id, if any>
        msg.cookie = 100
        self.connection.send(msg)

    def f_tcp_process(self,ipp,packet_in): #  event_icmp):    #
        '''get Load balance decision'''
        self.vlan_choice = self.f_LB_decision()
        print 'self.vlan_choice =',self.vlan_choice 
        '''forward packet by changing src/dst ip 
        without installing matching rule'''
        self.f_dispatch_packet(ipp, packet_in)
    
    
class Topo_controller(object): #(EventMixin):
    def __init__ (self):
        self.infor_obj = rt.TopoInfor(topo_file)
        self.switches ={}
        # Listen to dependencies
        #def startup ():
        #    core.openflow.addListeners(self, priority=0)
        #core.call_when_ready(startup, ('openflow'))
        core.openflow.addListeners(self)

    def _create_arp_control (self, event):
        fm = of.ofp_flow_mod()
        fm.priority = ARP_CONTROL_PRIORITY
        fm.match.dl_type = ethernet.ARP_TYPE
        fm.actions.append(of.ofp_action_output(port=of.OFPP_CONTROLLER))
        event.connection.send(fm)
        
    def _create_switch_connection(self,event):
        dpid = event.dpid
        print event
        sw = self.switches.get(dpid)
        if sw is None:
            '''get switch infor obj'''
            sw_name = self.infor_obj.dpidSwitchDict[dpid]
            switchVlan_obj = self.infor_obj.nodeVlanDict[sw_name]
            # New switch
            sw = L3_switch(switchVlan_obj)
            sw.connect(event)
            
            #sw = LearningSwitch(event.connection, False)
            #sw.dpid = dpid
            
            self.switches[dpid] = sw            
            
        
    def _handle_ConnectionUp (self, event):
        '''create OpenFlow controlled ARP mechanism
           do NOT flood before asking controller
        '''
        self._create_arp_control(event)
        '''Init connection between controller and OpenFlow switches'''
        self._create_switch_connection(event)

def launch ():
  """
  Starts the component
  """
  core.registerNew(Topo_controller)
