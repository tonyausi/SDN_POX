#!/usr/bin/python

'''
Cloud project:
- read topology csv file 
Tony Lin
'''

import os, sys, getopt
import numpy as np
import re
from collections import defaultdict as dd

SPECIAL_DELIMITER   = ['',';','$']
IP_DELIMITER        = ['/']
FULLIP              = 'ip/maskLength'
IPONLY              = 'ip'
NULLSTR = '' # empty string 
NULLLIST = [] # empty list
SRC = 'transmitter'
DST = 'receiver'
VLANZERO = 0
HEX      = 16
SERVER_TYPE = 'server'
SWITCH_TYPE = 'switch'
HOST_TYPE   = 'host'
VID         = 'vlanid'
LINKOPTS    = 'linkopts'
SRC_NAME    = 'src_name'
DST_NAME    = 'dst_name'
SRC_TYPE    = 'src_type'
DST_TYPE    = 'dst_type'
SRC_ID      = 'src_id'
DST_ID      = 'dst_id'
SRC_MAC     = 'src_mac'
DST_MAC     = 'dst_mac'
SRC_PORT    = 'src_port'
DST_PORT    = 'dst_port'
SRC_VID     = 'src_vids'
SRC_VIP     = 'src_vip'
DST_VID     = 'dst_vids'
ROW_FORMAT = {'names': (VID, LINKOPTS,
                        SRC_NAME, DST_NAME,
                        SRC_TYPE, DST_TYPE,
                        SRC_ID  , DST_ID,
                        SRC_MAC , DST_MAC,
                        SRC_PORT, DST_PORT,
                        SRC_VID , DST_VID),
              'formats': ('i4', 'S40',
                          'S4', 'S4',
                          'S6', 'S6',
                          'S60','S40',
                          'S20','S20',
                          'i4', 'i4',
                          'S40','S40')}

def f_read_topo(topo_fname,topoformat=ROW_FORMAT):
    # Reading the csv file.
    topo_data = np.loadtxt(topo_fname, dtype=topoformat,
                           comments='#',delimiter=',')
##    print topo_data
#     print topo_data[0][1]
##    print topo_data[VID]
    return topo_data

def f_get_item_set(topo_data):
    src_list = list(set(topo_data[SRC_NAME]))
    dst_list = list(set(topo_data[DST_NAME]))
    item_list = list(set(src_list+dst_list))
    return item_list  

def f_get_vlan_set(topo_data):
    return list(set(topo_data[VID]))  

def f_parse_item_set(topo_data):

    host_list=[]
    switch_list=[]
    server_list=[]
    vid_raw_list  = topo_data[VID]
    linkops_raw_list = topo_data[LINKOPTS]
    type_raw_list = list(topo_data[SRC_TYPE])+list(topo_data[DST_TYPE])
    name_raw_list = list(topo_data[SRC_NAME])+list(topo_data[DST_NAME])
    id_raw_list   = list(topo_data[SRC_ID])+list(topo_data[DST_ID])
    port_raw_list = list(topo_data[SRC_PORT])+list(topo_data[DST_PORT])
    #print name_raw_list[0]

    for idx, type_data in enumerate(type_raw_list):
        target = name_raw_list[idx]
        if type_data == SWITCH_TYPE:
            if target not in switch_list:
                switch_list.append(name_raw_list[idx])
        elif type_data == HOST_TYPE:
            if target not in host_list:
                host_list.append(name_raw_list[idx])
        elif type_data == SERVER_TYPE:
            if target not in server_list:
                server_list.append(name_raw_list[idx])
        else:
            print "unsupported type: %s" %type_data
            sys.exit()
     
#     print topo_data[SRC_TYPE]==SWITCH_TYPE
    return switch_list,host_list,server_list

def f_get_linkops(raw_linkops):
    if raw_linkops: # is not None or raw_linkops is not EOF:
        opts = re.sub('[%s]' % ''.join(SPECIAL_DELIMITER), ',', raw_linkops)
        try:
            return eval(opts)
        except ValueError:
            print "can not eval dictionary %s" %opts
            sys.exit(2)
    else:
        return {}        
def f_parse_ip_only(ipAndMasklength):
    return re.split('[%s]' % ''.join(IP_DELIMITER), ipAndMasklength)[0]
    
def f_parse_ips(id_infor,parse_mode=FULLIP):
    if id_infor:
        ips = re.split('[%s]' % ''.join(SPECIAL_DELIMITER), id_infor)
        ips_len = len(ips)
        #mac_add = None
        virtual_ip = None
        virtual_dst_ip = None
        if ips_len == 1:
            physical_ip = ips[0]
        elif ips_len == 2:
            physical_ip = ips[0]
            mac_add     = ips[1]
        elif ips_len == 3:
            physical_ip = ips[0]
            virtual_ip = ips[1]
            virtual_dst_ip = ips[2]
        #elif ips_len == 4:
        #    physical_ip = ips[0]
        #    mac_add     = ips[1]
        #    virtual_ip = ips[2]
        #    virtual_dst_ip = ips[3]      
        else:
            print "only support 1 ~ 3 ips"
            sys.exit()
        if parse_mode == IPONLY:
            physical_ip = f_parse_ip_only(physical_ip)
            if virtual_ip!=None: virtual_ip  = f_parse_ip_only(virtual_ip)
            if virtual_dst_ip!=None: virtual_dst_ip = f_parse_ip_only(virtual_dst_ip)
        #return physical_ip, mac_add, virtual_ip, virtual_dst_ip
        return physical_ip, virtual_ip, virtual_dst_ip
    else:
        print "empty id_infor"
        return None, None, None
    
def f_parse_vids(vid_str):
    if vid_str != NULLSTR:
        vidstr_list = re.sub('[%s]' % ''.join(SPECIAL_DELIMITER), ',', vid_str)
        try:
            vid_list = eval(vidstr_list)
        except ValueError:
            print "can not eval vid string: %s" %opts
            sys.exit(2)
        return vid_list
    else:
        return []
    
def f_convertDpid(dpid_in,dpidLen):
    "Derive dpid from short string '1'-->'000000000001' "
    try:
        dpid = int( re.findall( r'\d+', dpid_in )[ 0 ] )
        dpid = hex( dpid )[ 2: ]
        dpid = '0' * ( dpidLen - len( dpid ) ) + dpid
        return dpid
    except IndexError:
        raise Exception( 'Unable to derive default datapath ID - '
                         'please either specify a dpid or use a '
                         'canonical switch name such as s23.' )
            
class VlanRoute(object):
    def __init__(self,name):
        ''' init.
        
        '''
        self.vlanIndex = name
        self.srcSwitch = None
        self.dstSwitch = None
        self.srcIP     = None
        self.srcMac    = None
        self.srcVIP    = None        
        self.dstIP     = None
        self.dstMac    = None
        self.dstVIP    = None
        self.route     =[]
        self.ports     ={}
        self.types     ={}
        self.ids       ={}
    def get_tail(self):
        if self.route:
            return self.route[-1]
        else:
            return None
    def get_head(self):
        if self.route:
            return self.route[0]
        else:
            return None
    def sort_route(self,src_idx,dst_idx):
        '''
        assume src_idx > dst_idx such that existing src is after dst
        a,b,c,dst,d,(e,src),f,g,h
        reorder route list to make 
        a,b,c,(e,src),dst,d,f,g,h
        '''
        total_len = len(self.route)
        
        insert_head = dst_idx + 2
        insert_tail = src_idx + 1 
        src_part = self.route[insert_head:insert_tail]
        #print insert_head,insert_tail,'src_part=',src_part
        self.route[insert_head:insert_tail]=[]
        self.route[dst_idx:dst_idx]=src_part
        
    def update_infor(self,direction,name,port,type,id,mac): 
            self.ports[name] = port            
            self.types[name] = type
            self.ids[name] = [f_parse_ips(id)] # id
            if type == HOST_TYPE and direction==SRC:
                self.srcIP,self.srcVIP,self.dstVIP = f_parse_ips(id,IPONLY) # f_parse_ips(id)#
                self.srcMac = mac
            if (type == HOST_TYPE or type == SERVER_TYPE) and direction==DST:
                self.dstIP, srcVIP, dstVIP = f_parse_ips(id,IPONLY)#f_parse_ip_only(id)  # id #  
                self.dstMac = mac                   
           
    def add_pair(self,src_name,src_port,src_type,src_id,src_mac,
                      dst_name,dst_port,dst_type,dst_id,dst_mac):
        if src_name == dst_name:
            raise
        if (src_name not in self.route) and (dst_name not in self.route):
            ''' append to the end first '''
            self.route.append(src_name)
            self.route.append(dst_name)
            self.update_infor(SRC,src_name,src_port,src_type,src_id,src_mac)
            self.update_infor(DST,dst_name,dst_port,dst_type,dst_id,dst_mac)            

        elif (src_name in self.route) and (dst_name not in self.route):
            src_idx = self.route.index(src_name)
            ''' add dst at the end of src '''
            dst_idx = src_idx+1
            cur_len = len(self.route)
            if dst_idx < cur_len:
                self.route[dst_idx] = dst_name
            else:
                self.route.append(dst_name)
            self.update_infor(DST,dst_name,dst_port,dst_type,dst_id,dst_mac)
            
        elif (src_name not in self.route) and (dst_name in self.route):
            dst_idx = self.route.index(dst_name)
            '''preposition src_name before dst_name'''
            self.route[dst_idx:dst_idx]=[src_name]
            self.update_infor(SRC,src_name,src_port,src_type,src_id,src_mac)
            
        elif (src_name in self.route) and (dst_name in self.route):
            src_idx = self.route.index(src_name)
            dst_idx = self.route.index(dst_name)
            #print src_idx,dst_idx
            if src_idx > dst_idx:
                self.sort_route(src_idx, dst_idx)
                
        self.update_switch()
        
    def update_switch(self):
        head_type = self.types[self.get_head()]
        tail_type = self.types[self.get_tail()]
        if head_type == HOST_TYPE:
            self.srcSwitch = self.route[1]
        else:
            self.srcSwitch = self.get_head()
        if tail_type == HOST_TYPE or tail_type == SERVER_TYPE:    
            self.dstSwitch = self.route[-2]
        else:
            self.dstSwitch = self.get_tail()
            
class NodeVlan(object):
    def __init__(self,name,type=None):
        ''' init.
        
        '''
        self.name = name
        self.type = type
        self.id   = None
        self.vlans = set()
        self.neighbours = set()
        self.vlan_ports = dd(lambda:dd(lambda:None)) #{} 
        self.connections = dd(lambda:dd(lambda:None)) #{} 
        ''' only applied to edge switch with NAT function'''
        self.nat_enable = False
        ''' only applied to switch connected to end switch/host'''
        self.vlt_enable = False 
        self.srcIP     = {}
        self.srcMac    = {}
        self.srcVIP    = {}        
        self.dstIP     = {}
        self.dstMac    = {}
        self.dstVIP    = {}
        self.match_dict = {}
        self.nat_dict   = {}
        self.vlan_nat_options = {}
        self.vlan_nat_src_pair = {}
        self.ip_mac_dict= {}
        #self.virtual_ip = None
        #self.transmitter_ip = None
        #self.receiver_ip = {}   
        
    def update_node(self,vlan_id, port, type, id, vidlist,
                    neighbour_name,neighbour_port,
                    neighbour_type,neighbour_id,neighbour_vidlist,
                    neibhbour_status):
        
        #neighbour_port_dict = {neighbour_name:neighbour_port}
        #obj_port_dict       = {neibhbour_status:port}
        ''' update DPID or IP property '''
        if self.id == None:
            self.id = id     
        ''' update type property '''  
        if self.type == None:
            self.type = type
        ''' update vlan related dicts'''        
##        if vlan_id not in self.vlans:
##            self.vlan_ports[vlan_id]={neibhbour_status:port}
##            self.connections[vlan_id]={neighbour_name:neighbour_port}
##        else:
##            self.vlan_ports[vlan_id][neibhbour_status]=port
##            self.connections[vlan_id][neighbour_name]=neighbour_port
        self.vlan_ports[vlan_id][neibhbour_status]=port
        self.connections[vlan_id][neighbour_name]=neighbour_port
        #print vlan_id,self.vlans, self.vlan_ports    
        '''update host-->switch infor for switch'''
        if neighbour_type == HOST_TYPE and neibhbour_status == SRC:
            self.nat_enable = True
            for vid in neighbour_vidlist:
                self.vlan_ports[vid][neibhbour_status]=port
                self.connections[vid][neighbour_name]=neighbour_port
        '''update switch-->server/host infor for switch'''
        if ((neighbour_type == SERVER_TYPE or neighbour_type == HOST_TYPE) 
            and neibhbour_status == DST):
            self.vlt_enable = True
            for vid in neighbour_vidlist:
                self.vlan_ports[vid][neibhbour_status]=port
                self.connections[vid][neighbour_name]=neighbour_port                
        '''update vlan and neighbours set'''
        self.vlans.add(vlan_id)
        self.neighbours.add(neighbour_name)    

    def generat_flow(self):
        if self.type != SWITCH_TYPE:
            return
        for vid in self.vlans:
            if vid == VLANZERO:
                continue
            if self.nat_enable:
                '''Add SRCip --> DSTip match'''
                '''load balancer will choose SRC-->DST nat'''
                ''' #??????????????????????????????????????????????????????
                match_tuple = (self.srcIP[vid],self.dstVIP[vid])
                out_port    = self.vlan_ports[vid][DST]
                self.match_dict[match_tuple] = out_port
                self.nat_dict[match_tuple] = nat_tuple
                '''
                nat_tuple = (self.srcVIP[vid],self.dstIP[vid])                              
                self.vlan_nat_options[vid] = nat_tuple
                sourc_pair_tuple = (self.srcIP[vid],self.dstVIP[vid])
                self.vlan_nat_src_pair[vid] = sourc_pair_tuple
                '''Add DSTip --> SRCip match'''
                match_tuple = (self.dstIP[vid],self.srcVIP[vid])
                out_port    = self.vlan_ports[vid][SRC]
                self.match_dict[match_tuple] = out_port
                nat_tuple = (self.dstVIP[vid],self.srcIP[vid])
                self.nat_dict[match_tuple] = nat_tuple
            else:
                '''Add SRCip --> DSTip match'''
                match_tuple = (self.srcVIP[vid],self.dstIP[vid])
                out_port    = self.vlan_ports[vid][DST]
                self.match_dict[match_tuple] = out_port
                '''Add DSTip --> SRCip match'''
                match_tuple = (self.dstIP[vid],self.srcVIP[vid])
                out_port    = self.vlan_ports[vid][SRC]
                self.match_dict[match_tuple] = out_port
                
    def generat_arp(self):  
        if self.type != SWITCH_TYPE:
            return
        for vid in self.vlans:
            if vid == VLANZERO:
                continue
            for ip in [self.srcIP[vid],self.srcVIP[vid]]:  
                if ((ip not in self.ip_mac_dict.keys()) and ip != None):
                    self.ip_mac_dict[ip] = self.srcMac[vid]
            '''exclude DST VIP - MAC mapping (TBD by load balancer)'''
            if ((self.dstIP[vid] not in self.ip_mac_dict.keys()) 
                and self.dstIP[vid] != None):
                    self.ip_mac_dict[self.dstIP[vid]] = self.dstMac[vid]
                         
                    
class TopoInfor(object):
    def __init__(self,fname,debug=False):
        ''' init.
        
        '''
        self.debug = debug
        self.vlanRouteDict={}
        self.nodeVlanDict={}
        self._swList=[]
        self.switchDpidDict={}
        self.dpidSwitchDict={}
        self.host_list=[]
        self.switch_list=[]
        self.server_list=[]
        self.topo_data = f_read_topo(fname)
        #self.switch_list,self.host_list,self.server_list = f_parse_item_set(self.topo_data)
        #print self.topo_data
        #'''
        self.node_set  = f_get_item_set(self.topo_data)
        self.switch_vlan_set = f_get_vlan_set(self.topo_data)
        self.switch_vlan_set.remove(VLANZERO)
        print self.node_set,self.switch_vlan_set
        '''Init VlanRoute and NodeVlan objs'''
        self.init_objs()
        #print self.vlanRouteDict
        '''parse each line of topo_data'''  
        self.parse_data()
        '''Creat match table for NodeVlan objs for each switch'''
        self.creat_switchs_match()
        
        if not self.debug:
            return
        #'''
        for i in self.switch_vlan_set:
            print '###############'
            print self.vlanRouteDict[i].route, self.vlanRouteDict[i].srcSwitch, self.vlanRouteDict[i].dstSwitch
            print self.vlanRouteDict[i].srcIP, self.vlanRouteDict[i].srcVIP, self.vlanRouteDict[i].dstIP, self.vlanRouteDict[i].dstVIP
            print self.vlanRouteDict[i].ids
        
        for i in self.node_set:
            print '@@@@@@@@@@@@@@@'
            print self.nodeVlanDict[i].name, self.nodeVlanDict[i].vlan_ports.items(), self.nodeVlanDict[i].vlans
            #print self.nodeVlanDict[i].srcIP, self.nodeVlanDict[i].srcVIP
            #print self.nodeVlanDict[i].dstIP, self.nodeVlanDict[i].dstVIP
            if self.nodeVlanDict[i].type==SWITCH_TYPE:
                print self.nodeVlanDict[i].match_dict
                print self.nodeVlanDict[i].nat_dict
                print self.nodeVlanDict[i].vlan_nat_options
                print self.nodeVlanDict[i].dstVIP
        print '++++++++++++++'
        print self.dpidSwitchDict
        print self.switchDpidDict  
        #'''  
        
    def init_objs(self):
        # init VlanRoute objs
        for vlan_id in self.switch_vlan_set:
            self.vlanRouteDict[vlan_id]=VlanRoute(vlan_id)
        # init NodeVlan objs
        for node_id in self.node_set:
            self.nodeVlanDict[node_id]=NodeVlan(node_id)        
        
    def parse_data(self):   
        for idx, row_data in enumerate(self.topo_data):
            vlan_id  = row_data[VID]
            #print 'vlan_id=',vlan_id
            src_vidlist = f_parse_vids(row_data[SRC_VID]) #row_data[SRC_VID]
            dst_vidlist = f_parse_vids(row_data[DST_VID]) #row_data[DST_VID]
            ''''src para'''
            src_name = row_data[SRC_NAME]
            src_port = row_data[SRC_PORT]
            src_type = row_data[SRC_TYPE]
            src_id   = row_data[SRC_ID]
            src_mac  = row_data[SRC_MAC]
            ''''dst para'''
            dst_name = row_data[DST_NAME]
            dst_port = row_data[DST_PORT]
            dst_type = row_data[DST_TYPE]
            dst_id   = row_data[DST_ID] 
            dst_mac  = row_data[DST_MAC] 
            '''update switch dpid dicts '''
            self.updat_switch_dpid_dicts(src_name,src_type,src_id,
                                         dst_name,dst_type,dst_id)
            '''update VlanRoute objs'''
            if vlan_id != VLANZERO:
                self.set_vlanRoute(vlan_id,src_name,src_port,src_type,src_id,src_mac,
                                   dst_name,dst_port,dst_type,dst_id,dst_mac)
            elif src_vidlist != NULLLIST:
                #print 'src_name=', src_name
                for srcvlan_id in src_vidlist:
                    self.set_vlanRoute(srcvlan_id,src_name,src_port,src_type,src_id,src_mac,
                                       dst_name,dst_port,dst_type,dst_id,dst_mac)
            elif dst_vidlist != NULLLIST:                
                for dstvlan_id in dst_vidlist:
                    self.set_vlanRoute(dstvlan_id,src_name,src_port,src_type,src_id,src_mac,
                                       dst_name,dst_port,dst_type,dst_id,dst_mac)
            '''update nodes objs'''
            '''SRC node'''        
            src_node = self.nodeVlanDict[src_name]
            src_node.update_node(vlan_id, src_port,src_type,src_id,src_vidlist,
                                 dst_name,dst_port,dst_type,dst_id,dst_vidlist,DST)
            '''DST node'''        
            dst_node = self.nodeVlanDict[dst_name]
            dst_node.update_node(vlan_id, dst_port,dst_type,dst_id,dst_vidlist,
                                 src_name,src_port,src_type,src_id,src_vidlist,SRC)   
                 
    def updat_switch_dpid_dicts(self,src_name,src_type,src_id,
                                     dst_name,dst_type,dst_id):
        if src_type==SWITCH_TYPE and src_name not in self._swList:
            src_id_value = self.get_dpid_value(src_id)
            self.switchDpidDict[src_name] = src_id_value
            self.dpidSwitchDict[src_id_value]   = src_name
            self._swList.append(src_name)
        if dst_type==SWITCH_TYPE and dst_name not in self._swList:
            dst_id_value = self.get_dpid_value(dst_id)
            self.switchDpidDict[dst_name] = dst_id_value
            self.dpidSwitchDict[dst_id_value]   = dst_name
            self._swList.append(dst_name)    
 
    def get_dpid_value(self,dpid_str):
        try:
            value = int(dpid_str,HEX)
            return value
        except ValueError:
            print "can not convert dpid %s into int" %(dpid_str)
            sys.exit(2)
 
    def set_vlanRoute(self,vlan_id,src_name,src_port,src_type,src_id,src_mac,
                                   dst_name,dst_port,dst_type,dst_id,dst_mac):  
        target_vlanRoute = self.vlanRouteDict[vlan_id]
        target_vlanRoute.add_pair(src_name,src_port,src_type,src_id,src_mac,
                                  dst_name,dst_port,dst_type,dst_id,dst_mac)                 

    def creat_switchs_match(self):
        '''
        '''
        for node in self.node_set:
            node_obj = self.nodeVlanDict[node]
            if node_obj.type != SWITCH_TYPE:
                continue
            '''get IPs for each vlan involved'''
            self.update_switch_ips(node_obj)
            '''creat match rule and forward port based on IPs'''
            node_obj.generat_flow()
            '''creat arp ip-mac tabel'''
            node_obj.generat_arp()
            
    def update_switch_ips(self,switch_obj):
        for vlan in switch_obj.vlans:
            if vlan == VLANZERO:
                continue
            vlanRoute_obj = self.vlanRouteDict[vlan]
            switch_obj.srcIP[vlan]  = vlanRoute_obj.srcIP
            switch_obj.srcMac[vlan]  = vlanRoute_obj.srcMac
            switch_obj.srcVIP[vlan] = vlanRoute_obj.srcVIP        
            switch_obj.dstIP[vlan]  = vlanRoute_obj.dstIP
            switch_obj.dstMac[vlan]  = vlanRoute_obj.dstMac
            switch_obj.dstVIP[vlan] = vlanRoute_obj.dstVIP
        
                        
        
            
if __name__ == "__main__":
##    obj_vlan = VlanRoute(1)
##    obj_vlan.add_pair('7', 7,SWITCH_TYPE,7, '8', 8,SERVER_TYPE,'192.168.100.201/24')
##    print obj_vlan.route
##    obj_vlan.add_pair('5', 5,SWITCH_TYPE,5, '6', 6,SWITCH_TYPE,6)
##    print obj_vlan.route
##    obj_vlan.add_pair('1', 1,HOST_TYPE,'192.168.100.1/24', '2', 2,SWITCH_TYPE,2)
##    print obj_vlan.route
##    obj_vlan.add_pair('3', 3,SWITCH_TYPE,3, '4', 4,SWITCH_TYPE,4)
##    print obj_vlan.route
##    #
##    obj_vlan.add_pair('2', 2,SWITCH_TYPE,2, '3', 3,SWITCH_TYPE,3)
##    print obj_vlan.route
##    obj_vlan.add_pair('4', 4,SWITCH_TYPE,4, '5', 5,SWITCH_TYPE,5)
##    print obj_vlan.route
##    obj_vlan.add_pair('6', 6,SWITCH_TYPE,6, '7', 7,SWITCH_TYPE,7)
##    print obj_vlan.route
##    print obj_vlan.ports
##    print obj_vlan.types
##    print obj_vlan.ids
##    print obj_vlan.srcSwitch,obj_vlan.dstSwitch

    infor_obj = TopoInfor('nat.csv',debug=True)
    
                    
            
            
            
        
