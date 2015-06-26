#!/usr/bin/python

'''
Cloud project:
- Software Defined Networking (SDN) 
-- custom defined switches

Tony Lin
'''

from pox.core import core
import pox.openflow.libopenflow_01 as of
from pox.lib.revent import *
#from pox.lib.recoco import Timer
#from pox.openflow.discovery import Discovery
from pox.lib.util import dpid_to_str
from pox.lib.packet.ethernet import ethernet, ETHER_BROADCAST
from pox.lib.packet.ipv4 import ipv4
from pox.lib.packet.arp import arp
from pox.lib.addresses import IPAddr, EthAddr, IP_ANY, IP_BROADCAST
try: # old dir for Pyretic VM
    from pox.misc.arp_helper import send_arp_reply
except ImportError:
     # new dir for mininet VM
    from pox.proto.arp_helper import send_arp_reply

import time
import os, sys, getopt
#import numpy as np
import re
import random
from collections import defaultdict

#sys.path.append('/home/Sdn_src/exp/src/')
#print sys.path
#print os.environ["PYTHONPATH"]
import utils.read_topo as rt
from utils.proj_util import MemoryDict
from utils.proj_util import WeightedRandomPicker as wrp

#TYPE_IP = 0x0800
#TYPE_ARP= 0x0806
ARP_CONTROL_PRIORITY = 0x0007 # Pretty high
DUMMY_MAC = EthAddr('F0:F0:F0:F0:F0:F0')
V4PROTOCOLS = {'ICMP':ipv4.ICMP_PROTOCOL,'UDP':ipv4.UDP_PROTOCOL,'TCP':ipv4.TCP_PROTOCOL}

log = core.getLogger()

#topo_file = "%s/Sdn_src/exp/src/topo_config/nat.csv" % os.environ[ 'HOME' ]  
 
class L3_project_switch(EventMixin):  #   (object): # 
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
        self.dns_enable = self.switchVlan_obj.dns_enable
        self.vlt_enable = self.switchVlan_obj.vlt_enable
        self.strip_vlan = self.dns_enable or self.vlt_enable
        if self.dns_enable:
            # get non-zero vlan ID set
            self.dns_choice_list = list(self.vlans.difference(set([rt.VLANZERO])))     #  vlan_nat_options.keys()
            self.dns_choice_list.sort()
            self.LBWR = wrp(self.dns_choice_list,[0.5]*len(self.dns_choice_list),20*len(self.dns_choice_list))
            #self.LBWR = wrp(self.dns_choice_list,[0.6,0.0,0.1,0.3],100)
            #self.LBWR = wrp(self.dns_choice_list,[0.5,0.5,0.5,0.5],4)
            #print 'self.LBWR.vlanRatio = ', self.LBWR.vlanRatio
        self.arp_choice_done = False    
        self.counter = 0
        self.tcp_new_session = False
        self.flow_mem = {} # (srcip,dstvip, dstip) -> MemoryDict
        
        

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

    def inject_match_flows (self):
        match_dict = self.switchVlan_obj.match_dict        
        '''set for ICMP, UDP, TCP'''
        cookie = 9
        for v4protocol in V4PROTOCOLS.itervalues():            
            for (vlan_id,port_id),forward_port in match_dict.iteritems():
                self.inject_single_match_flow(cookie,vlan_id,port_id,forward_port,
                                              dl_type = ethernet.IP_TYPE,nw_proto = v4protocol)                                              
                cookie = cookie + 1
            if self.switchVlan_obj.vlt_enable and self.switchVlan_obj.multiDstIpMode:
                for (src_port_id,serverIp),[forward_port,vlan_id] in self.switchVlan_obj.match_srv_dict.iteritems():
                    #print src_port_id,serverIp, forward_port,vlan_id, v4protocol
                    self.inject_vtag_match_flow(cookie,src_port_id,serverIp,
                                                       forward_port,vlan_id,
                                                       nw_proto=v4protocol)
                    cookie = cookie + 1
                
    def inject_single_match_flow(self,cookie,vlan_id,port_id,forward_port,
                                             dl_type = ethernet.IP_TYPE,
                                             new_src_ip=None,new_dst_ip=None,                                             
                                             nw_proto = ipv4.ICMP_PROTOCOL):
        ''' inject single vlan_id,port_id with forwarding port'''
        msg = of.ofp_flow_mod()
        msg.match.in_port=port_id
        msg.match.dl_vlan=vlan_id
        if dl_type:
            msg.match.dl_type = dl_type
            msg.match.nw_proto = nw_proto
        #print vlan_id,port_id
        #print self.switchVlan_obj.name,forward_port, self.switchVlan_obj.ethPortMap
        target_forward_port = forward_port
        if self.switchVlan_obj.multiDstIpMode:
            if self.dns_enable: # NAT + stripVid
                msg.actions.append(of.ofp_action_strip_vlan())
                target_forward_port,new_srcIP = self.switchVlan_obj.match_edge_dict[(vlan_id,port_id)]
                msg.actions.append(of.ofp_action_nw_addr.set_src(IPAddr(new_srcIP)))
            elif self.vlt_enable:
                #print self.switchVlan_obj.name+'(vlan_id=%d,port_id=%d)-->forward_port=%d' %(vlan_id,port_id,forward_port)
                # SRC host --> DST server 
                msg.actions.append(of.ofp_action_strip_vlan())               
        else:                
            if self.strip_vlan:
                msg.actions.append(of.ofp_action_strip_vlan())                
        msg.actions.append(of.ofp_action_output(port = target_forward_port))
        msg.cookie = cookie
        self.connection.send(msg)
    
    def inject_vtag_match_flow(self,cookie,src_port_id,serverIp,
                                           forward_port,vid,
                                           dl_type = ethernet.IP_TYPE,
                                           nw_proto = ipv4.ICMP_PROTOCOL,
                                           new_src_ip=None,new_dst_ip=None,new_dst_mac=None,                                             
                                           ):
        msg = of.ofp_flow_mod()
        msg.match.in_port=src_port_id       
        if dl_type:
            msg.match.dl_type = dl_type
            msg.match.nw_proto = nw_proto
            msg.match.nw_src =serverIp
        target_forward_port = forward_port       
        '''set new src IP'''
        if new_src_ip:
            msg.actions.append(of.ofp_action_nw_addr.set_src(IPAddr(new_src_ip)))
        '''set new dst IP'''
        if new_dst_ip:
            msg.actions.append(of.ofp_action_nw_addr.set_dst(IPAddr(new_dst_ip)))
        if new_dst_mac:
            msg.actions.append(of.ofp_action_dl_addr.set_dst(EthAddr(new_dst_mac)))
        '''add VLAN ID'''
        if vid:
            msg.actions.append(of.ofp_action_set_vlan_vid(vlan_vid=vid))              
        msg.actions.append(of.ofp_action_output(port = forward_port))        
        msg.cookie = cookie
        self.connection.send(msg)
        
    '''depleted'''
    def test_single_nat(self,cookie):
        ''' test for 1st route '''
        vlan_keys  = self.switchVlan_obj.vlan_nat_options.keys()
        vlan_keys.sort()
        self.vlan_choice = vlan_keys[-1]
        #print self.vlan_choice
        (src_ip,dst_ip) = self.switchVlan_obj.vlan_nat_src_pair[self.vlan_choice]
        (new_src_ip,new_dst_ip) = self.switchVlan_obj.vlan_nat_options[self.vlan_choice]
        forword_eth_port = self.switchVlan_obj.vlan_ports[self.vlan_choice][rt.DST]
        forward_port = self.switchVlan_obj.ethPortMap[forword_eth_port]
        #print (src_ip,dst_ip),(new_src_ip,new_dst_ip),forward_port
        '''set for ICMP'''
        self.inject_single_match_flow(cookie,src_ip,dst_ip,forward_port,
                                      dl_type = ethernet.IP_TYPE,
                                      new_src_ip=new_src_ip,new_dst_ip=new_dst_ip)
        '''set for TCP'''
        self.inject_single_match_flow(cookie,src_ip,dst_ip,forward_port,
                                      dl_type = ethernet.IP_TYPE,
                                      new_src_ip=new_src_ip,new_dst_ip=new_dst_ip,nw_proto = ipv4.TCP_PROTOCOL)
        

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
        udpp = packet.find("udp")
        tcpp = packet.find("tcp")
        ipp  = packet.find("ipv4")
        #print icmpp,tcpp,ipp  #, packet.payload.prototype
        if (not icmpp) and (not tcpp) and (not udpp):  #       not ipp: #
            arpp = packet.find("arp")
            if not arpp: return None #drop ()
            #print 'start ARP process'
            #print 'For ARP, event.dpid=', event.dpid
            #print arpp.prototype,'?=', arp.PROTO_TYPE_IP
            #print arpp.hwtype ,'?=',  arp.HW_TYPE_ETHERNET        
            #print 'self.dpid=', self.dpid
            self.packet_event = event
            self.f_arp_process(arpp)
            return
        #'''
        if icmpp:
            #print 'start ICMP process'
            ipp = packet.find('ipv4')
            packet_in = event.ofp # The actual ofp_packet_in message.
            self.f_icmp_process(ipp,packet_in)    
        #'''  
        if tcpp:
            #print 'start TCP process'
            ipp = packet.find('ipv4')
            packet_in = event.ofp # The actual ofp_packet_in message.
            self.f_tcp_process(tcpp,ipp,packet_in)  
        #'''
        if udpp:
	    #print 'start UDP process'
	    ipp = packet.find('ipv4')
	    packet_in = event.ofp # The actual ofp_packet_in message.
	    self.f_udp_process(udpp,ipp,packet_in)  
        #'''
            
    def f_arp_process(self,arpp):
        if arpp.opcode == arp.REQUEST:
            self.f_arp_reply(arpp)
        elif arpp.opcode == arp.REPLY:
            print "It's a reply; do something cool (Not supported yet)"
            #'''
            #print 'arpp.next=',arpp.next
            #print 'arpp.protosrc=',arpp.protosrc
            #print 'arpp.protodst=',arpp.protodst
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
        srcIP  = arpp.protosrc.toStr()
        srcMac = arpp.hwsrc.toStr()
        #IP_MAC.update(srcIP = 
        dstIP  = arpp.protodst.toStr()
        #print 'dstIP=',dstIP
        dstMac = self.f_get_arp_mac(srcIP,dstIP)  
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

    def f_get_arp_mac(self,srcIP,dstIP):
        #print 'self.switchVlan_obj.name = ', self.switchVlan_obj.name
        #print 'self.vlan_choice=', self.vlan_choice
        #print 'self.switchVlan_obj.dstNatIP[self.vlan_choice]=', self.switchVlan_obj.dstNatIP[self.vlan_choice]
        #print 'need to find dstIP', dstIP
        if dstIP in self.switchVlan_obj.ip_mac_dict.keys():
            #print self.switchVlan_obj.ip_mac_dict[dstIP]
            return EthAddr(self.switchVlan_obj.ip_mac_dict[dstIP])
        elif self.vlan_choice and dstIP == self.switchVlan_obj.dstNatIP[self.vlan_choice]:
            '''get physical dst IP string based on VLAN decision'''
            ip_str = self.switchVlan_obj.dstIP[self.vlan_choice]
            return EthAddr(self.switchVlan_obj.ip_mac_dict[ip_str])
        elif not self.vlan_choice:
            if self.dns_enable:
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
        self.connection.send(msg.pack()) #self.connection.send(msg)
        
    def f_send_arp_openflow_flood(self):
        msg = of.ofp_packet_out()
        msg.actions.append(of.ofp_action_output(port = of.OFPP_FLOOD))
        msg.data = self.packet_event.ofp
        self.packet_event.connection.send(msg.pack()) 
    
    def f_icmp_process(self,ipp,packet_in): #  event_icmp):    #
        '''get Load balance decision'''
        if self.dns_enable:self.vlan_choice = self.f_LB_decision()
        #print 'At ICMP self.vlan_choice =',self.vlan_choice 
        '''forward packet by changing src/dst ip 
        without installing matching rule'''
        self.f_dispatch_packet(ipp, packet_in)   
    
    def f_udp_process(self,udpp,ipp,packet_in): 
        '''get Load balance decision'''
        if self.dns_enable:self.vlan_choice = self.f_LB_decision()
        #print 'At UDP self.vlan_choice =',self.vlan_choice 
        '''forward packet by changing src/dst ip 
        without installing matching rule'''
        self.f_dispatch_packet(ipp, packet_in)            
    
    def f_tcp_process(self,tcpp,ipp,packet_in): #  event_icmp):    #
        syn_flag = tcpp.SYN
        ack_flag = tcpp.ACK
        if not ack_flag and syn_flag:
            self.tcp_new_session = True
            '''get Load balance decision'''
            if self.dns_enable:self.vlan_choice = self.f_LB_decision() #(tcp_mode=True)
            #print 'At TCP self.vlan_choice =',self.vlan_choice 
        else:
            self.tcp_new_session = False
        '''forward packet by changing src/dst ip 
        without installing matching rule'''
        self.f_dispatch_packet(ipp, packet_in ,tcp_mode=True)
        '''
        # disable tcp_mode is also OK due to static new session control 
        self.f_dispatch_packet(ipp, packet_in)
        '''
    
    def f_LB_decision(self,tcp_mode=False):
        '''random pick'''
        #return random.choice(self.switchVlan_obj.vlan_nat_options.keys())
        '''evenly pick'''
        #print '!!!!!!!!!!!At LB current DPID=',self.dpid
        if self.dns_enable:
            length = len(self.dns_choice_list)
            self.counter = self.counter + 1
            #print self.dns_choice_list, self.counter
            if tcp_mode:
                self.counter = length - 1
            #print self.counter % length, self.dns_choice_list    
            #return self.dns_choice_list[self.counter % length] 
            return self.LBWR.vlanChoice          
        elif self.vlt_enable:
            return None            
            
    def f_dispatch_packet(self,ipp,packet_in,tcp_mode=False):
        if self.dns_enable:
            self.f_dispatch_packet_dns(ipp,packet_in,tcp_mode)                                                
        elif self.vlt_enable:
            self.f_dispatch_packet_vlt(ipp,packet_in,tcp_mode)
    
    def f_dispatch_packet_dns(self,ipp,packet_in,tcp_mode):
        if ipp.dstip.toStr() in self.switchVlan_obj.dstNatIP.values():
            new_dst_ip = self.switchVlan_obj.dstIP[self.vlan_choice]
            new_dst_mac= self.switchVlan_obj.dstMac[self.vlan_choice]
            forword_eth_port = self.switchVlan_obj.vlan_ports[self.vlan_choice][rt.DST]
            forward_port = self.switchVlan_obj.ethPortMap[forword_eth_port]
            ''' DNS + add VLAN ID '''
            self.f_nat_forward_packet(self.vlan_choice,packet_in,new_dst_ip,new_dst_mac,forward_port)
            match_tuple = (ipp.srcip.toStr(),new_dst_ip)
            value       = (self.vlan_choice,ipp.dstip.toStr())
            if not tcp_mode or self.tcp_new_session:
                ''' record ingress srcIP+dstIP to vlan ID mapping'''                                 
                core.SRC_DST_VLAN.add({match_tuple:value})
            if tcp_mode:
                key_outter = (ipp.srcip.toStr(),ipp.dstip.toStr())
                self.f_retire_flow_mem()
                if key_outter not in self.flow_mem.keys():
                    key_inner = match_tuple
                    self.flow_mem.update({key_outter:MemoryDict(input={key_inner:value})})
                    
    def f_dispatch_packet_vlt(self,ipp,packet_in,tcp_mode):
        
        if len(self.switchVlan_obj.IpVid) <=1:
            self.f_dispatch_packet_vlt_single(ipp,packet_in,tcp_mode)
            print 'process single mode here'
#         else:
#             self.f_dispatch_packet_vlt_multi(ipp,packet_in,tcp_mode)
        
    def f_dispatch_packet_vlt_single(self,ipp,packet_in,tcp_mode):
        match_tuple = (ipp.dstip.toStr(),ipp.srcip.toStr())       
        if tcp_mode:
            key_outter = match_tuple
            self.f_retire_flow_mem()
            if key_outter not in self.flow_mem.keys():
                '''refer to global memory'''
                try:
                    vid, forward_port, dstVip = self.f_retrieve_global_vlan_decision(match_tuple)
                    '''update own flow mem'''
                    key_inner = key_outter
                    value = (vid, forward_port, dstVip)
                    self.flow_mem.update({key_outter:MemoryDict(input={key_inner:value})})
                except:
                    '''find any available route with vid, forward_port, dstVip. TBD'''
                    return                    
            else:
                mem_dict_obj = self.flow_mem.get(key_outter)
                vid, forward_port, dstVip = mem_dict_obj.get(key_outter)
            #print "at f_dispatch_packet_vlt,vid=%d, forward_port=%d, dstVip=%s" % (vid, forward_port, dstVip) 
        else:                
            if match_tuple in core.SRC_DST_VLAN.keys:
                vid, forward_port, dstVip = self.f_retrieve_global_vlan_decision(match_tuple)                                    
            else:
                '''No match, TBD'''
                return
        self.f_add_vid_forward_packet(vid,packet_in,forward_port,dstVip)
    
    ''' depleted method replaced by initial flow match of vlan tagging for server-side switch'''
    def f_dispatch_packet_vlt_multi(self,ipp,packet_in,tcp_mode):
        match_tuple = (ipp.dstip.toStr(),ipp.srcip.toStr())
        if match_tuple not in core.SRC_DST_VLAN.keys:
            return        
        #print 'self.switchVlan_obj.name=%s' %(self.switchVlan_obj.name)
        #print match_tuple
        #print self.switchVlan_obj.IpVid
        backwardVlan = self.switchVlan_obj.IpVid[ipp.srcip.toStr()]
        #print 'backworkdVlan=%d' %(backwardVlan)
        backwardPort = self.switchVlan_obj.vlan_ports[backwardVlan][rt.SRC]
        pysicalPort = self.switchVlan_obj.ethPortMap[backwardPort]
        #print 'backwardPort =%d ,pysicalPort=%d' %(backwardPort,pysicalPort)
        #print ipp.dstip.toStr()
        #vid, forward_port, dstVip = self.f_retrieve_global_vlan_decision(match_tuple)
        #self.f_add_vid_forward_packet(backwardVlan,packet_in,pysicalPort,dstVip)
        self.f_add_vid_forward_packet(backwardVlan,packet_in,pysicalPort)
    
    def f_retire_flow_mem(self):
        c = len(self.flow_mem)
        self.flow_mem = {k:v for k,v in self.flow_mem.items() 
                         if not v.is_expired}
        if len(self.flow_mem) != c:
            log.debug("Expired %i flows", c-len(self.flow_mem))
        
    def f_retrieve_global_vlan_decision(self,match_tuple): 
        vid,dstVip = core.SRC_DST_VLAN.get(match_tuple)
        #print 'at vlt, dstVip=', dstVip
        forword_eth_port = self.switchVlan_obj.vlan_ports[vid][rt.SRC]
        forward_port = self.switchVlan_obj.ethPortMap[forword_eth_port]
        return vid, forward_port, dstVip 
                                 
    def f_nat_forward_packet(self,vid,packet_in,new_dst_ip,new_dst_mac,forward_port,new_src_ip=None):    
        msg = of.ofp_packet_out(in_port=packet_in.in_port)               
        msg.data = packet_in
        '''set new src IP'''
        if new_src_ip:
            msg.actions.append(of.ofp_action_nw_addr.set_src(IPAddr(new_src_ip)))
        '''set new dst IP'''
        if new_dst_ip:
            msg.actions.append(of.ofp_action_nw_addr.set_dst(IPAddr(new_dst_ip)))
        if new_dst_mac:
            msg.actions.append(of.ofp_action_dl_addr.set_dst(EthAddr(new_dst_mac)))
        '''add VLAN ID'''
        if vid:
            msg.actions.append(of.ofp_action_set_vlan_vid(vlan_vid=vid))    
          
        msg.actions.append(of.ofp_action_output(port = forward_port))
        #msg.buffer_id = <some buffer id, if any>
        msg.cookie = 100
        self.connection.send(msg)
    
    def f_add_vid_forward_packet(self,vid,packet_in,forward_port,new_srcIP=None):
        msg = of.ofp_packet_out(in_port=packet_in.in_port)               
        msg.data = packet_in
        '''change DST IP'''
        if new_srcIP:
            msg.actions.append(of.ofp_action_nw_addr.set_src(IPAddr(new_srcIP)))
        '''add VLAN ID'''
        if vid:
            msg.actions.append(of.ofp_action_set_vlan_vid(vlan_vid=vid))
        msg.actions.append(of.ofp_action_output(port = forward_port))
        #msg.buffer_id = <some buffer id, if any>
        msg.cookie = 200
        self.connection.send(msg)
