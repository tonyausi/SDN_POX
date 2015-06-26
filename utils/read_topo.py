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
# import all constants
from utils.constant_read_topo import *
                                            
def f_sort_num_list(inList):  # sort by appended number
    #print inList
    return sorted(inList, key=lambda x: [int(y) for y in re.findall('(\d+)',x)])   

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

def f_parse_link_set(topo_data):
    link_list = []
    linkNames = set(topo_data[LINK_NAME])
    if NULL_STR in linkNames:
        linkNames.remove(NULL_STR)
    #print linkNames
    for link_name in linkNames:
        #link_name = re.split('[%s]' % ''.join(RES_PARA_DELIMITER), linkPara)[-1]
        if link_name and link_name != NULL_STR:
            link_list.append(link_name)    
    return f_sort_num_list(link_list)

def f_get_linkops(raw_linkops):
    if raw_linkops: # is not None or raw_linkops is not EOF:
        opts = re.sub('[%s]' % ''.join(SPECIAL_DELIMITER), ',', raw_linkops)
        try:
            opts_dict = eval(opts)
            # set linux Hierarchical Token Bucket to enable using one physical link 
            # to simulate several slower links and to send different kinds of traffic 
            # on different simulated links.
            opts_dict.update({'use_htb': True}) 
            return opts_dict
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
            #print vidstr_list
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
            
def f_parse_res_para(res_para):
    '''cap,pmax,beta,utiRatio:for journal 1 and 2 '''
    para_list = re.split('[%s]' % ''.join(RES_PARA_DELIMITER), res_para)
    out = []
    for p in para_list:
        if not p or p==NULL_STR:
            out.append(None)
        else:
            out.append(p)
    return out
            

class VlanRoute(object):
    def __init__(self,name):
        ''' init.
        
        '''
        self.vlanIndex = name
        self.srcSwitch = None
        self.dstSwitch = None
        self.srcIP     = None # depleted parameter to be removed
        self.srcMac    = None
        self.dstNatIP    = None
        self.srcVIP    = None # depleted parameter to be removed       
        self.dstIP     = None
        self.dstVIP    = None # TBD, may not be needed
        self.dstMac    = None        
        self.route     =[]
        self.ports     ={}
        self.types     ={}
        self.ids       ={}
        self.resSet    = set()
        self.hostSwSet = set()
        self.serverSwSet = set()
        self.noneResSet = set()
        self.linkSet   = set()
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
                self.srcIP,self.srcVIP,self.dstNatIP = f_parse_ips(id,IPONLY) # f_parse_ips(id)#
                self.srcMac = mac
            if (type == HOST_TYPE or type == SERVER_TYPE) and direction==DST:
                self.dstIP, self.dstVIP, srtNatIP = f_parse_ips(id,IPONLY)#f_parse_ip_only(id)  # id #  
                self.dstMac = mac                   
           
    def add_pair(self,src_name,src_port,src_type,src_id,src_mac,
                      dst_name,dst_port,dst_type,dst_id,dst_mac,
                      src_res_para, dst_res_para, link_res_para,link_name):
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
        self.update_res_para(src_name,src_type,dst_name,dst_type,
                             src_res_para, dst_res_para, link_res_para,link_name)       
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
    def update_res_para(self,src_name,src_type,dst_name,dst_type,
                        src_res_para, dst_res_para, link_res_para,link_name):
        '''update node res set'''
        self.update_node_set(src_name,src_type,src_res_para,dst_name,dst_type,dst_res_para)
        '''update link res set'''
        if link_name and link_name != NULL_STR:
            self.resSet.add(link_name)
            self.linkSet.add(link_name)
            
#         ''' add RES related para'''
#         s_paras = re.split('[%s]' % ''.join(RES_PARA_DELIMITER), src_res_para)
#         
#         '''TB stuffed'''
#         d_paras = re.split('[%s]' % ''.join(RES_PARA_DELIMITER), dst_res_para)
# 
#         '''TB stuffed'''
#         l_paras = re.split('[%s]' % ''.join(RES_PARA_DELIMITER), link_res_para)
#         if link_res_para:
#             print l_paras                       
#             '''TB stuffed'''
    def update_node_set(self,src_name,src_type,src_res_para,dst_name,dst_type,dst_res_para):
        if src_type != HOST_TYPE:
            self.resSet.add(src_name)
            if dst_type == HOST_TYPE:  # neighbour is host
                self.hostSwSet.add(src_name)
                self.noneResSet.add(src_name)                
            if dst_type == SERVER_TYPE:
                self.serverSwSet.add(src_name)
                if not src_res_para:
                    self.noneResSet.add(src_name)
        if dst_type != HOST_TYPE:
            self.resSet.add(dst_name)
            if src_type == HOST_TYPE:  # neighbour is host
                self.hostSwSet.add(dst_name)
                self.noneResSet.add(dst_name)
            if src_type == SERVER_TYPE:
                self.serverSwSet.add(dst_name)
                if not dst_res_para:
                    self.noneResSet.add(dst_name)        
        self.resSet = self.resSet- self.noneResSet  


class Resource(object):
    def __init__(self,name,type=None):        
        '''init res related paras'''
        self.name = name
        self.type = type
        self.capcity = None
        self.pmax    = None
        self.beta    = None
        self.utiRatio= None
        self.resDelay   = None  
        self.resParaUpdateDone = False    
        self.vlans = set()
        self.resMeasName = None  
        
    def f_updateResPara(self,res_para):
        if not res_para or res_para == NULL_STR or self.resParaUpdateDone:
            return
        paras = f_parse_res_para(res_para)
        if not reduce(lambda a,b: a and b, paras): # at least one item is None or 0
            return
        try:
            paras_float = [float(x) for x in paras]
        except ValueError:
            print 'can not convert to float for ', paras
            return
        if len(paras) == J1_PARA_LEN:
            [self.capcity,self.pmax,self.beta] = paras
            self.resParaUpdateDone = True
        elif len(paras) == J2_PARA_LEN:
            [self.capcity,self.pmax,self.beta,self.utiRatio] = paras
            self.resParaUpdateDone = True
        elif len(paras) == J2_PARA_DELAY_LEN:
            [self.capcity,self.pmax,self.beta,self.utiRatio,self.resDelay] = paras
            self.resParaUpdateDone = True
        else:
            raise ('do not support number of paras other than 3 or 4')
    def f_addMeasName(self):   
        self.resMeasName = self.name
                
        
class LinkVlan(Resource):
    def __init__(self,name,type=LINK_TYPE):
        ''' init.        
        '''
        super(LinkVlan, self).__init__(name,type)
        
    def update_link(self,vlan_id,link_res_para,src_type,src_name,src_port,dst_type,dst_name,dst_port):
        ''' '''
        self.vlans.add(vlan_id)
        self.f_updateResPara(link_res_para)
        self.f_addMeasName(src_type,src_name,src_port,dst_type,dst_name,dst_port)
    def f_addMeasName(self,src_type,src_name,src_port,dst_type,dst_name,dst_port): 
        if src_type not in [HOST_TYPE, SERVER_TYPE]:  
            self.resMeasName = src_name+'-eth'+str(src_port) 
        elif dst_type not in [HOST_TYPE, SERVER_TYPE]:  
            self.resMeasName = dst_name+'-eth'+str(dst_port)      
            
         
class NodeVlan(Resource):
    def __init__(self,name,type=None):
        ''' init.
        
        '''
        #self.name = name
        #self.type = type
        super(NodeVlan, self).__init__(name,type)
        self.id   = None
        #self.vlans = set()
        self.neighbours = set()
        self.vlan_ports = dd(lambda:dd(lambda:None)) # {[vlan][neighbourStatus]:forwardPort} 
        self.ethSet = set()
        self.ethPortMap = {}
        self.connections = dd(lambda:dd(lambda:None)) #{} 
        ''' only applied to edge switch with NAT function'''
        self.dns_enable = False
        ''' only applied to switch connected to end switch/host'''
        self.vlt_enable = False         
        self.srcIP     = {} # depleted parameter to be removed   
        self.srcMac    = {}
        self.srcVIP    = {} # depleted parameter to be removed       
        self.dstIP     = {}                          # {vlan:dstIP}
        self.dstMac    = {}
        self.dstNatIP    = {}
        self.match_dict = {}
        self.match_edge_dict={} # {[vid,inputPort]:[forwardPort,natIp] for host side switch
        self.nat_dict   = {}
        self.vlan_nat_options = {}
        self.vlan_nat_src_pair = {}
        self.ip_mac_dict= {}
        #self.virtual_ip = None
        #self.transmitter_ip = None
        #self.receiver_ip = {}
        self.IpVid = {} # {dstIP: vlanid}
        self.multiDstIpMode = False # multiple Vid-DstIP one-by-one mapping mode
        self.match_srv_dict = {}    #{(port_id,serverIp):vid}

        
    def update_node(self,vlan_id, port, type, id, vidlist, res_para,
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
        ''' update ethernet port number'''
        self.ethSet.add(port)
        ''' update vlan related dicts'''        
        self.vlan_ports[vlan_id][neibhbour_status]=port
        self.connections[vlan_id][neighbour_name]=neighbour_port
        #print vlan_id,self.vlans, self.vlan_ports    
        '''update host-->switch infor for switch'''
        if neighbour_type == HOST_TYPE and neibhbour_status == SRC:
            self.dns_enable = True
            for vid in neighbour_vidlist:
                self.vlan_ports[vid][neibhbour_status]=port
                self.connections[vid][neighbour_name]=neighbour_port
        '''update switch-->server/host infor for switch'''
        if ((neighbour_type == SERVER_TYPE or neighbour_type == HOST_TYPE) 
            and neibhbour_status == DST):
            #print self.name
            self.vlt_enable = True
            for vid in neighbour_vidlist:
                self.vlan_ports[vid][neibhbour_status]=port
                self.connections[vid][neighbour_name]=neighbour_port                
        '''update vlan and neighbours set'''
        self.vlans.add(vlan_id)
        self.neighbours.add(neighbour_name)
        '''update res para'''
        self.f_updateResPara(res_para) 
        '''add bwm measure name'''
        self.f_addMeasName(neighbour_type,neighbour_name,neighbour_port)
    def f_addMeasName(self,neighbour_type,neighbour_name,neighbour_port):
        if self.type == SWITCH_TYPE:     
            self.resMeasName = self.name
        elif self.type in [HOST_TYPE,SERVER_TYPE] and neighbour_type == SWITCH_TYPE:
            self.resMeasName = neighbour_name+'-eth'+str(neighbour_port) 

    

    def generat_vlan_flow(self):
        if self.type != SWITCH_TYPE:
            return        
        '''create ethernet to openVswitch port_ID map'''
        self.creat_eth_portId_map()
        ''' single mode vlan flow generation'''
        self.generat_vlan_flow_single()
        ''' for multiple vlan-dstIp setup only''' 
        #if (len(self.IpVid) == len(self.vlans)-1 and
        #   self.dns_enable): # each vlan associates with a unique dst ip address,-1 to exclude vlan0 
        if (len(self.IpVid) == len(self.vlans)-1): # each vlan associates with a unique dst ip address,-1 to exclude vlan0
            self.multiDstIpMode = True              
        self.generat_edge_vlan_flow()
        self.generat_srv_vlan0_flow()  
    def creat_eth_portId_map(self):
        '''For open Vswitch, port_ID starts from 1 but ethernet port may not based on the config. 
           Ref: http://openvswitch.org/cgi-bin/ovsman.cgi?page=utilities%2Fovs-ofctl.8
           cmd: sudo ovs-ofctl show [switch_name]
        '''
        if not self.ethSet:
            print 'No ethernet port is configured for switch %s' %self.name
            return        
        eth_list = list(self.ethSet)
        eth_list.sort()
        for idx in range(len(eth_list)):
            port_id = idx + 1 # port_id starts from 1 not 0
            eth_port= eth_list[idx]
            self.ethPortMap[eth_port]=port_id                  
    def generat_vlan_flow_single(self):
        '''create match_dict:{(vid,inputPort):forwardPort} for single mode'''
        for vid in self.vlans:
            if vid == VLANZERO:
                continue
            # real port ID may not be the ethernet port number
            src_port_id = self.map_eth_portId(self.vlan_ports[vid][SRC])
            dst_port_id = self.map_eth_portId(self.vlan_ports[vid][DST])
            
            if self.dns_enable:  
                '''for switch connecting host'''
                '''add per vlan DSTport --> SRCport match'''
                match_tuple = (vid,dst_port_id)
                out_port    = src_port_id
                self.match_dict[match_tuple] = out_port            
            elif self.vlt_enable: 
                '''for switch connecting switch'''
                '''add per vlan SRCport --> DSTport match'''
                match_tuple = (vid,src_port_id)
                out_port    = dst_port_id
                self.match_dict[match_tuple] = out_port            
            else:
                '''for MUX switch'''
                '''add per vlan DSTport --> SRCport match'''
                match_tuple = (vid,dst_port_id)
                out_port    = src_port_id
                self.match_dict[match_tuple] = out_port
                '''add per vlan SRCport --> DSTport match'''
                match_tuple = (vid,src_port_id)
                out_port    = dst_port_id
                self.match_dict[match_tuple] = out_port        
    def generat_edge_vlan_flow(self):
        '''only for edge switch connected to hosts (need additional NAT for traffic back from server)'''
        if (not self.dns_enable) or (not self.multiDstIpMode):
            return        
        for vid in self.vlans:
            if vid == VLANZERO:
                continue  
            # real port ID may not be the ethernet port number
            src_port_id = self.map_eth_portId(self.vlan_ports[vid][SRC])
            dst_port_id = self.map_eth_portId(self.vlan_ports[vid][DST])
            '''for switch connecting host'''
            '''add per vlan DSTport --> SRCport,nat_ip match'''
            match_tuple = (vid,dst_port_id)
            out_port    = src_port_id
            vip         = self.dstNatIP[vid]
            self.match_edge_dict[match_tuple] = [out_port, vip]                          
    def generat_srv_vlan0_flow(self):
        '''only for switch connected to servers (need to add vlan tag for traffic back from server)'''
        if (not self.vlt_enable) or (not self.multiDstIpMode):
            return
        # real port ID may not be the ethernet port number
        #print self.name, self.vlan_ports
        #print self.vlan_ports[VLANZERO][DST]
        src_port_id = self.map_eth_portId(self.vlan_ports[VLANZERO][DST])
        for vid in self.vlans:
            if vid == VLANZERO:
                continue
            serverIp = self.dstIP[vid]
            # real port ID may not be the ethernet port number
            out_port = self.map_eth_portId(self.vlan_ports[vid][SRC])
            match_tuple = (src_port_id,serverIp)
            self.match_srv_dict[match_tuple] = [out_port, vid]
    def map_eth_portId(self,ethPort): 
        ''' switch ethPort --> openVswitch port_ID'''
        if self.ethSet:
            return self.ethPortMap[ethPort]
        else:
            return ethPort   
    

                            
    '''depleted method for virtual src IP'''
    def generat_flow(self):
        if self.type != SWITCH_TYPE:
            return
        '''create ethernet to openVswitch port_ID map'''
        self.creat_eth_portId_map()
        for vid in self.vlans:
            if vid == VLANZERO:
                continue
            if self.dns_enable:
                '''Add SRCip --> DSTip match'''
                '''load balancer will choose SRC-->DST nat'''
                ''' #??????????????????????????????????????????????????????
                match_tuple = (self.srcIP[vid],self.dstNatIP[vid])
                out_port    = self.vlan_ports[vid][DST]
                self.match_dict[match_tuple] = out_port
                self.nat_dict[match_tuple] = nat_tuple
                '''
                nat_tuple = (self.srcVIP[vid],self.dstIP[vid])                              
                self.vlan_nat_options[vid] = nat_tuple
                sourc_pair_tuple = (self.srcIP[vid],self.dstNatIP[vid])
                self.vlan_nat_src_pair[vid] = sourc_pair_tuple
                '''Add DSTip --> SRCip match'''
                match_tuple = (self.dstIP[vid],self.srcVIP[vid])
                out_port    = self.map_eth_portId(self.vlan_ports[vid][SRC])
                self.match_dict[match_tuple] = out_port
                nat_tuple = (self.dstNatIP[vid],self.srcIP[vid])
                self.nat_dict[match_tuple] = nat_tuple
            else:
                '''Add SRCip --> DSTip match'''
                match_tuple = (self.srcVIP[vid],self.dstIP[vid])
                out_port    = self.map_eth_portId(self.vlan_ports[vid][DST])
                self.match_dict[match_tuple] = out_port
                '''Add DSTip --> SRCip match'''
                match_tuple = (self.dstIP[vid],self.srcVIP[vid])
                out_port    = self.map_eth_portId(self.vlan_ports[vid][SRC])
                self.match_dict[match_tuple] = out_port
                


    '''depleted method'''        
    def generat_arp_table(self):  
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

'''depleted class to be remoed'''
class ResMem(object):
    def __int__(self):
        self.capMem  = {}
        self.pmaxMem = {}
        self.betaMem = {}
        self.utiRatioMem = {}
        self.nameKeys = set()
        self.hostMeasPort = {}
        self.serverMeasPort = {}
        
    def f_rea_para(self,srcName, srcPara, srcType, srcPort,
                        dstName, dstPara, dstType, dstPort,
                       linkName, linkPara ):
        if srcName not in self.nameKeys:
            srcParaList = f_parse_res_para(srcPara)
            
        if linkName not in self.nameKeys:
            linkParaLIst = f_parse_res_para(linkPara)  
    def f_add_para(self,resName,paraData,resType,resPort,neighborName,neighborType): 
        ''' get server aggregate input bw measure eth port'''
        if neighborType == SERVER_TYPE:  
            self.serverMeasPort[neighborName] = resName+'-eth'+str(resPort)
        ''' get host aggregate output bw measure eth port'''
        if neighborType == HOST_TYPE:  
            self.hostMeasPort[neighborName] = resName+'-eth'+str(resPort)           
        ''' add para '''
        ParaList = f_parse_res_para(paraData)
                           
                    
class TopoInfor(object):
    def __init__(self,fname,debug=False):
        ''' init.
        
        '''
        self.debug = debug
        self.topo_data = f_read_topo(fname)
        '''POX setup related'''
        self.init_pox_set()
        '''RES para related'''
        self.init_res()
        
        '''Init VlanRoute and NodeVlan objs'''
        self.init_objs()
        #print self.vlanRouteDict
        '''parse each line of topo_data'''  
        self.parse_data()
        '''Create match table for NodeVlan objs for each switch'''
        self.creat_switchs_match()
        '''Generat traffic distribution related paras'''
        self.get_res_paras()
        
        if not self.debug:
            return
        '''
        for i in self.switch_vlan_set:
            print '###############'
            print self.vlanRouteDict[i].route, self.vlanRouteDict[i].srcSwitch, self.vlanRouteDict[i].dstSwitch
            print self.vlanRouteDict[i].srcIP, self.vlanRouteDict[i].srcVIP, self.vlanRouteDict[i].dstIP, self.vlanRouteDict[i].dstNatIP
            print self.vlanRouteDict[i].ids, self.vlanRouteDict[i].resSet, self.vlanRouteDict[i].hostSwSet
        '''
        print '*************'
        print self.sortedResList, len(self.sortedResList)
        print self.userRouteMat
        print self.reRouteMat
        print np.shape(self.reRouteMat)
        self.f_saveArray(self.reRouteMat)
        print self.link_list
        print 'self.resServer_num=',self.resServer_num,'self.resMux_num=',self.resMux_num,'self.resLink_num=',self.resLink_num
        print 'self.resCapacity=',self.resCapacity
        print 'self.resPmax=',self.resPmax
        print 'self.resBeta=',self.resBeta
        print 'self.resUtilRatio=',self.resUtilRatio
        print 'self.resMeasKeys=',self.resMeasKeys
        print 'self.hostMeasKeys=',self.hostMeasKeys
        print 'self.resDelay=',np.shape(self.resDelay)
        
        
        resDict = dict(self.nodeVlanDict.items() + self.linkVlanDict.items())
        for res in self.sortedResList+self.host_list:
            print 'mmmmmmmmmmmmmmmmmmmmm', res
            print resDict[res].vlans
            print resDict[res].name, resDict[res].resMeasName, resDict[res].capcity,resDict[res].pmax,resDict[res].beta,resDict[res].utiRatio
        
        '''
        for i in self.node_set:
            print '@@@@@@@@@@@@@@@'
            print self.nodeVlanDict[i].name, self.nodeVlanDict[i].vlan_ports.items(), self.nodeVlanDict[i].vlans
            print self.nodeVlanDict[i].srcIP, self.nodeVlanDict[i].srcVIP
            print self.nodeVlanDict[i].dstIP, self.nodeVlanDict[i].dstNatIP
            if self.nodeVlanDict[i].type==SWITCH_TYPE:
                print self.nodeVlanDict[i].match_dict
                print self.nodeVlanDict[i].ip_mac_dict
                print self.nodeVlanDict[i].nat_dict
                print self.nodeVlanDict[i].vlan_nat_options
                print self.nodeVlanDict[i].dstNatIP
                print self.nodeVlanDict[i].multiDstIpMode
                if self.nodeVlanDict[i].multiDstIpMode:
                    print self.nodeVlanDict[i].match_edge_dict
                    if self.nodeVlanDict[i].vlt_enable :
                        print self.nodeVlanDict[i].match_srv_dict
            print self.nodeVlanDict[i].IpVid
        print '++++++++++++++'
        print self.dpidSwitchDict
        print self.switchDpidDict  
        '''  
    def init_pox_set(self):
        self.vlanRouteDict={}
        self.nodeVlanDict={}
        self._swList=[]
        self.switchDpidDict={}
        self.dpidSwitchDict={}
        self.host_list=[]
        self.switch_list=[]
        self.server_list=[]
        self.ip_mac_dict    = {}        
        self.switch_list,self.host_list,self.server_list = f_parse_item_set(self.topo_data)
        self.node_set  = f_get_item_set(self.topo_data)
        self.switch_vlan_set = f_get_vlan_set(self.topo_data)
        self.switch_vlan_set.remove(VLANZERO)
        print self.node_set,self.switch_vlan_set
    def init_res(self):
        self.linkVlanDict={}
        self.link_list = f_parse_link_set(self.topo_data)        
        self.sysResSet = set()
        self.sysLinkSet = set()        
        self.sortedResList = []
        '''RES output'''
        self.resServer_num = None
        self.resMux_num   = None
        self.resLink_num  = None
        self.userRouteMat = None # RE matrix for user/route mapping
        self.reRouteMat   = None # RM matrix for res/route mapping
        
        self.resCapacity  = []
        self.resPmax      = []
        self.resBeta      = []
        self.resUtilRatio = []
        self.resDelay     = []
        self.resMeasKeys  = []
        self.hostMeasKeys = []      
        
    def init_objs(self):
        # init VlanRoute objs
        for vlan_id in self.switch_vlan_set:
            self.vlanRouteDict[vlan_id]=VlanRoute(vlan_id)
        # init NodeVlan objs
        for node_id in self.node_set:
            self.nodeVlanDict[node_id]=NodeVlan(node_id)
        if self.link_list:
            for link_name in self.link_list:
                self.linkVlanDict[link_name]=LinkVlan(link_name)
                            
    def parse_data(self):   
        for idx, row_data in enumerate(self.topo_data):
            vlan_id  = row_data[VID]
            #print 'vlan_id=',vlan_id
            src_vidlist = f_parse_vids(row_data[SRC_VID]) #row_data[SRC_VID]
            dst_vidlist = f_parse_vids(row_data[DST_VID]) #row_data[DST_VID]
            '''src para'''
            src_name = row_data[SRC_NAME]
            src_port = row_data[SRC_PORT]
            src_type = row_data[SRC_TYPE]
            src_id   = row_data[SRC_ID]
            src_mac  = row_data[SRC_MAC]
            '''dst para'''
            dst_name = row_data[DST_NAME]
            dst_port = row_data[DST_PORT]
            dst_type = row_data[DST_TYPE]
            dst_id   = row_data[DST_ID] 
            dst_mac  = row_data[DST_MAC] 
            '''RES para'''
            try:
                src_res_para  = row_data[SRC_PARA]
                dst_res_para  = row_data[DST_PARA]
                link_res_para = row_data[LINK_PARA]
                t_name        = row_data[LINK_NAME]
                link_name     = t_name
            except:
                src_res_para  = dst_res_para = link_res_para = link_name = None
            '''update switch dpid dicts '''
            self.update_switch_dpid_dicts(src_name,src_type,src_id,
                                          dst_name,dst_type,dst_id)
            '''update VlanRoute objs'''
            if vlan_id != VLANZERO:
                self.set_vlanRoute(vlan_id,src_name,src_port,src_type,src_id,src_mac,
                                   dst_name,dst_port,dst_type,dst_id,dst_mac,
                                   src_res_para, dst_res_para, link_res_para,link_name)
            elif src_vidlist != NULLLIST:
                #print 'src_name=', src_name
                for srcvlan_id in src_vidlist:
                    self.set_vlanRoute(srcvlan_id,src_name,src_port,src_type,src_id,src_mac,
                                       dst_name,dst_port,dst_type,dst_id,dst_mac,
                                       src_res_para, dst_res_para, link_res_para,link_name)                    
                # update IP-MAC table for switch connecting HOST
                self.update_ip_mac_table(src_id,src_mac)                       
            elif dst_vidlist != NULLLIST:                
                for dstvlan_id in dst_vidlist:
                    self.set_vlanRoute(dstvlan_id,src_name,src_port,src_type,src_id,src_mac,
                                       dst_name,dst_port,dst_type,dst_id,dst_mac,
                                       src_res_para, dst_res_para, link_res_para,link_name)
                # update IP-MAC table for switch connecting SERVER
                self.update_ip_mac_table(dst_id,dst_mac)    
            '''update nodes objs'''
            '''SRC node'''        
            src_node = self.nodeVlanDict[src_name]
            src_node.update_node(vlan_id, src_port,src_type,src_id,src_vidlist,src_res_para,
                                 dst_name,dst_port,dst_type,dst_id,dst_vidlist,DST)
            '''DST node'''        
            dst_node = self.nodeVlanDict[dst_name]
            dst_node.update_node(vlan_id, dst_port,dst_type,dst_id,dst_vidlist,dst_res_para,
                                 src_name,src_port,src_type,src_id,src_vidlist,SRC)
            '''update link objs'''
            if link_name in self.linkVlanDict.keys():
                link_obj = self.linkVlanDict[link_name]
                link_obj.update_link(vlan_id,link_res_para,
                                     src_type,src_name,src_port,
                                     dst_type,dst_name,dst_port)
                           
    def update_ip_mac_table(self,id,mac):
        srcIP,srcVIP,dstNatIP = f_parse_ips(id,IPONLY)
        if srcIP:
            self.ip_mac_dict.update({srcIP:mac})
        if srcVIP:
            self.ip_mac_dict.update({srcVIP:mac})        
                     
    def update_switch_dpid_dicts(self,src_name,src_type,src_id,
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
            value = int(dpid_str) #int(dpid_str,HEX), event.dpid readback is not HEX
            return value
        except ValueError:
            print "can not convert dpid %s into int" %(dpid_str)
            sys.exit(2)
 
    def set_vlanRoute(self,vlan_id,src_name,src_port,src_type,src_id,src_mac,
                                   dst_name,dst_port,dst_type,dst_id,dst_mac,
                                   src_res_para, dst_res_para, link_res_para,link_name):  
        target_vlanRoute = self.vlanRouteDict[vlan_id]
        target_vlanRoute.add_pair(src_name,src_port,src_type,src_id,src_mac,
                                  dst_name,dst_port,dst_type,dst_id,dst_mac,
                                  src_res_para, dst_res_para, link_res_para,link_name)                 

    def creat_switchs_match(self):
        '''
        '''
        for node in self.node_set:
            node_obj = self.nodeVlanDict[node]
            if node_obj.type != SWITCH_TYPE:
                continue
            '''get IPs for each vlan involved'''
            self.update_switch_ips(node_obj)
            #'''creat match rule and forward port based on IPs'''
            #node_obj.generat_flow()
            '''creat match rule and forward port based on vlan ID and input port'''
            node_obj.generat_vlan_flow()
            '''creat arp ip-mac tabel'''
            #node_obj.generat_arp_table()
            node_obj.ip_mac_dict = self.ip_mac_dict
            
    def update_switch_ips(self,switch_obj):
        for vlan in switch_obj.vlans:
            if vlan == VLANZERO:
                continue
            vlanRoute_obj = self.vlanRouteDict[vlan]
            switch_obj.srcIP[vlan]  = vlanRoute_obj.srcIP    # depleted, to be removed 
            switch_obj.srcMac[vlan]  = vlanRoute_obj.srcMac  # depleted, to be removed 
            switch_obj.srcVIP[vlan] = vlanRoute_obj.srcVIP   # depleted, to be removed      
            switch_obj.dstIP[vlan]  = vlanRoute_obj.dstIP
            switch_obj.dstMac[vlan]  = vlanRoute_obj.dstMac
            switch_obj.dstNatIP[vlan] = vlanRoute_obj.dstNatIP
            switch_obj.IpVid[vlanRoute_obj.dstIP] = vlan
            #switch_obj.IpVid[vlanRoute_obj.srcIP] = vlan            
 
    def get_res_paras(self):
        '''get res set'''
        self.get_res_set()
        '''get RE mat'''
        self.get_user_route_mat()
        '''get RM mat'''
        self.get_res_route_map()
        '''update real meas port'''
        self.f_upateMeasEthPort()
        ''' get RES related setup '''
        self.f_get_res_setup()
    def get_res_set(self):
        for vlan_id in self.switch_vlan_set:
            self.sysResSet=self.sysResSet.union(self.vlanRouteDict[vlan_id].resSet)
            self.sysLinkSet=self.sysLinkSet.union(self.vlanRouteDict[vlan_id].linkSet)
    def get_user_route_mat(self):
        #self.host_list
        re_mat=[]
        for user_name in f_sort_num_list(self.host_list):
            rt_row = []
            for route in sorted(list(self.switch_vlan_set)):
                if user_name in self.vlanRouteDict[route].route:
                    rt_row.append(1)
                    self.nodeVlanDict[user_name].vlans.add(route)
                else:
                    rt_row.append(0)
            re_mat.append(rt_row)
        self.userRouteMat = np.array(re_mat)
    def get_res_route_map(self):
        serverList = f_sort_num_list(list(self.sysResSet.intersection(set(self.server_list))))
        muxList    = f_sort_num_list(list(self.sysResSet.intersection(set(self.switch_list))))
        linkList   = f_sort_num_list(list(self.sysLinkSet))
        self.resServer_num = len(serverList)
        self.resMux_num    = len(muxList)
        self.resLink_num   = len(linkList)
        #print serverList,muxList,linkList
        self.sortedResList = serverList + muxList + linkList
        resDict = dict(self.nodeVlanDict.items() + self.linkVlanDict.items())
        rm_mat=[]
        for re_name in self.sortedResList:
            rt_row = []
            for route in sorted(list(self.switch_vlan_set)):
                if re_name in self.vlanRouteDict[route].resSet:
                    rt_row.append(1)
                    if resDict[re_name].type == SERVER_TYPE:
                        resDict[re_name].vlans.add(route)
                else:
                    rt_row.append(0)
            rm_mat.append(rt_row)
        self.reRouteMat = np.array(rm_mat)
    def f_upateMeasEthPort(self):      
        for linkName in self.link_list:
            resObj = self.linkVlanDict[linkName]
            self.f_updateMeasEthPortObj(resObj)
            #print resObj.resMeasName
        for hostName in self.host_list:
            resObj = self.nodeVlanDict[hostName]
            self.f_updateMeasEthPortObj(resObj)
            #print resObj.resMeasName
    def f_updateMeasEthPortObj(self,resObj):
        [swName,swPort] = re.split('-eth',resObj.resMeasName)
        swObj   = self.nodeVlanDict[swName]
        truePort = swObj.map_eth_portId(int(swPort))
        resObj.resMeasName = swName+'-eth'+str(truePort)
    def f_get_res_setup(self):  
        resDict = dict(self.nodeVlanDict.items() + self.linkVlanDict.items())
        for resName in self.sortedResList:
            resObj = resDict[resName]
            self.resCapacity.append(float(resObj.capcity))
            self.resPmax.append(float(resObj.pmax))
            self.resBeta.append(float(resObj.beta))
            self.resUtilRatio.append(float(resObj.utiRatio) if resObj.utiRatio else None)
            self.resDelay.append(float(resObj.resDelay) if resObj.resDelay else None)
            self.resMeasKeys.append(resObj.resMeasName)
        for hostName in self.host_list:
            self.hostMeasKeys.append(resDict[hostName].resMeasName)
    def f_saveArray(self,array_in,format=np.int,fname='array.csv'):  
        t_array = np.array(array_in,dtype=format,)
        try:
            np.savetxt(fname, t_array, fmt='%d',delimiter=",")
            #np.savetxt(fname, t_array, dtype=format,fmt='%i',delimiter=",")
        except:
            print 'can not save input array to a csv file'
               
   
            
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

    #infor_obj = TopoInfor('nat.csv',debug=False)
    #infor_obj = TopoInfor('nat1.csv',debug=True)
    infor_obj = TopoInfor('usnet11_delay.csv',debug=True)
    
                    
            
            
            
        
