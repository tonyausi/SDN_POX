#!/usr/bin/python

'''
Cloud project:
- Software Defined Networking (SDN) 
-- Top level module for customed topology

Tony Lin
'''

from mininet.topo import Topo
from mininet.net import Mininet
from mininet.node import CPULimitedHost
from mininet.link import TCLink
from mininet.util import irange,dumpNodeConnections,netParse
from mininet.log import setLogLevel
from mininet.cli import CLI
from mininet.node import ( Host, CPULimitedHost, Controller, OVSController,
                           NOX, RemoteController, UserSwitch, OVSKernelSwitch,
                           OVSLegacyKernelSwitch )

import os, sys, getopt, subprocess
import numpy as np
import re
from collections import defaultdict as dd

sys.path.append('/home/mininet/Sdn_src/exp/src/')
from utils import read_topo
from utils.constant_read_topo import *
#import utils.read_topo as read_topo
from utils.proj_util import f_logNetBw, f_getTopoFile, f_getRawList, f_getTrafficDict, f_getIncreList,f_get2DIncreList, f_getScale,f_rescaleTrafficDict,f_getPara
from utils.constant_sys import * 
from utils.proj_mn_process import f_generate_traffic
from utils.proj_socket import MnServer
import utils.paras as PARAS

DPID_LEN    = 16

#tem hard constants
#TEM_hostTraffic_dict = [{'h1':'3m','h2':'7m'},{'h1':'4m','h2':'8m'},{'h1':'5m','h2':'9m'}] #[{'h1':'8m','h2':'4m'}]
TEM_hostTraffic_dict = [{'h1':'0.3m','h2':'0.7m'}]   

class customTopo(Topo):    
    "Customised topology ."

    def __init__(self, topo_fname, **opts):
        """topo_fname: topology file name in csv format
           hconf: host configuration options
           lconf: link configuration options"""

        super(customTopo, self).__init__(**opts)
        self.dpidLen = DPID_LEN
        self.topo_fname = topo_fname
        self.topo_data = read_topo.f_read_topo(self.topo_fname)
        self.item_list = read_topo.f_get_item_set(self.topo_data)
        print self.item_list
        '''copy a list '''
        self.tem_list = self.item_list+[]
        # build topo
        self._node=[]
        self._physical_ip = None
        self._virtual_ip  = None
        self.hostPortIpDict=dd(lambda:dd(lambda:[]))
        self.serverPortIpDict=dd(lambda:dd(lambda:[]))
        self.hostVdstIpDict={}
        self.f_build_topo()    

    def f_add_end_node(self,name,id,mac):
        '''virtual ips are not used in this module '''
        self._physical_ip, self._virtual_ip, virtual_dst_ip = read_topo.f_parse_ips(id)
        '''Add host or server node'''
        if name in self.tem_list:            
            #print name, physical_ip
            mac_add = mac #'AA:BB:CC:DD:EE:FF'
            self._node = self.addHost(name,ip=self._physical_ip,mac=mac_add)
            if virtual_dst_ip:
                self.hostVdstIpDict[name] = virtual_dst_ip
            self.tem_list.remove(name)
    def f_record_host_ips(self,name,port):
        if name not in self.hostPortIpDict.keys():
            self.hostPortIpDict[name][port].append(self._physical_ip)
            self.hostPortIpDict[name][port].append(self._virtual_ip)
        elif self._virtual_ip:
            self.hostPortIpDict[name][port].append(self._virtual_ip)
    def f_record_server_ips(self,name,port): # add multiple ip for servers, might be merged with f_record_host_ips
        if name not in self.serverPortIpDict.keys():
            self.serverPortIpDict[name][port].append(self._physical_ip)           
        elif self._physical_ip not in self.serverPortIpDict[name][port]:
                self.serverPortIpDict[name][port].append(self._physical_ip)
                       
    def f_add_node(self,name,type,id,mac,port):        
        if type == SWITCH_TYPE:
            if name in self.tem_list:
                id_str = read_topo.f_convertDpid(id,self.dpidLen)
                #id_str = '0000000000000021'
                #print id_str
                self._node = self.addSwitch(name,dpid=id_str) #id)
                self.tem_list.remove(name)
        elif type == SERVER_TYPE:
            self.f_add_end_node(name,id,mac) 
            self.f_record_server_ips(name,port)
        elif type == HOST_TYPE:
            self.f_add_end_node(name,id,mac) 
            self.f_record_host_ips(name,port)
                             
          
    def f_build_topo(self):
        for idx, row_data in enumerate(self.topo_data):
            #print idx, type_data
            link_ops = read_topo.f_get_linkops(row_data[LINKOPTS])
            ##print link_ops
            '''get port infor'''
            src_port = row_data[SRC_PORT]
            dst_port = row_data[DST_PORT]
            ''' add src node '''
            src_name = row_data[SRC_NAME]
            src_type = row_data[SRC_TYPE]
            src_id   = row_data[SRC_ID]
            src_mac  = row_data[SRC_MAC]
            self.f_add_node(src_name, src_type, src_id, src_mac, src_port)
            ''' add dst node '''
            dst_name = row_data[DST_NAME]
            dst_type = row_data[DST_TYPE]
            dst_id   = row_data[DST_ID]
            dst_mac  = row_data[DST_MAC]
            self.f_add_node(dst_name, dst_type, dst_id, dst_mac, dst_port)
            ''' add link '''
            self.addLink(src_name,dst_name,src_port,dst_port,**link_ops)
        
def f_setup_nodes(net,topo,switch_list,host_list,server_list):
    for host in net.hosts:
        '''set multiple IP address on the host interface'''
        if host.name in topo.hostPortIpDict.keys():
            #print 'host.name=',host.name
            for port in topo.hostPortIpDict[host.name].keys():
                adds = topo.hostPortIpDict[host.name][port]
                for idx in range(1,len(adds)):
                    host.cmd( 'ifconfig %s-eth%d:%d %s up' %(host.name,port,idx,adds[idx]) )  
        '''set server'''
        #'''
        if host.name in server_list:
            '''set multiple IP address on the server interface''' 
            if host.name in topo.serverPortIpDict.keys():
                for port in topo.serverPortIpDict[host.name].keys():
                    adds = topo.serverPortIpDict[host.name][port]
                    for idx in range(1,len(adds)):
                        host.cmd( 'ifconfig %s-eth%d:%d %s up' %(host.name,port,idx,adds[idx]) )
            #print 'server.name=',host.name
            host.cmd( 'python -m SimpleHTTPServer 80 &' )  
            # add iperf UDP server
            #host.cmd( 'iperf -s -u &') 
            # add iperf TCP server
            #host.cmd( 'iperf -s -w 32m &')                              
        #'''
    print "Dumping host connections"
    dumpNodeConnections(net.hosts)
    #print "Testing network connectivity"
    #net.pingAll()
    for switch_inst in net.switches:
        print "switch %s has dpid=%s" %(switch_inst.name,switch_inst.dpid)      
   
def f_test_topo(topo_fname,mat_fname,traffic_in,weight_raw_in=[],cli_mode=False):
    tp = f_getTopoFile(topo_fname)
    print tp
    if not tp:
        raise Exception( 'Unable to find topology file: %s' ) %tp 
    
    mat_f = f_getTopoFile(mat_fname,file_appendix='.mat')
    print 'mat file: ', mat_f
        
    topo = customTopo(tp)
    PARAS.MININET_PARAS[TOPO] = topo
    #print topo.hostPortIpDict
    switch_list,host_list,server_list = read_topo.f_parse_item_set(topo.topo_data)
    # add global mininet parameters
    PARAS.MININET_PARAS[SWITCH_LIST] = switch_list
    PARAS.MININET_PARAS[HOST_LIST]   = host_list
    PARAS.MININET_PARAS[SERVER_LIST] = server_list
    #print host_list, switch_list, server_list
    '''
    net = Mininet(topo=topo,
                  host=CPULimitedHost, link=TCLink)
    '''
    '''
    net = Mininet(topo=topo, switch=UserSwitch, 
                  host=CPULimitedHost, link=TCLink)
    '''
    '''
    net = Mininet(topo=topo, switch=UserSwitch, 
                  host=CPULimitedHost, controller=RemoteController, link=TCLink)
    '''
    #'''
    net = Mininet(topo=topo, 
                  host=CPULimitedHost, controller=RemoteController, link=TCLink)
    #'''
    net.start()
    PARAS.MININET_PARAS[NET] = net
    ''' setup host,server and switch'''
    f_setup_nodes(net,topo,switch_list,host_list,server_list)
    
    if not cli_mode:
        mn_st_machine = MnServer(topo_fname,net,topo,server_list,switch_list,traffic_in,weight_raw_in)
        mn_st_machine.io_mat_f = mat_f
        final = mn_st_machine.run()
    
#     status = f_init_server_socket()
#     
#     if status:
#     
#    f_generate_traffic(net,topo,{'h1':'8m'},server_list,period=1000)
#     
#         diable_items = ','.join(switch_list)+','+SYS_ENTITY
#         #print diable_items    
#         f_logNetBw(NETBW_FILE,diable_items)
          
    CLI( net )
    net.stop()

def main(argv):
    os.system('sudo mn -c')
    os.system('sudo killall iperf')
    os.system('sudo killall bwm-ng')
    mat_fname = b''
    mat_mode  = False
    cli_mode = False
    ping_mode= False
    delay_th = None
    weight_raw_in  = None
    traffic_in = TEM_hostTraffic_dict   
    trafficScale = 1.0
    try:
        # http://www.tutorialspoint.com/python/python_command_line_arguments.htm
        opts, args = getopt.getopt(argv[1:], "hbcpm:w:i:I:D:s:d:", ["help","mat"])
    except getopt.GetoptError:
        print "need to review args"
        sys.exit(2)
    #print 'opts=',opts, ' args=',args    
    for opt, arg in opts:        
        if opt in ("-h", "--help"):
            print "custom <topo.csv>"
            sys.exit()
        elif opt == '-b':
            global _debug
            _debug = 1
        elif opt == '-c':
            cli_mode = True          
        elif opt == '-p':
            ping_mode = True
            
        elif opt in ("-m", "--mat"):
            mat_fname = arg
            mat_mode = True
        elif opt in ("-w", "--weight"):
            #print opt, arg
            weight_raw_in = f_getRawList(arg,'w')
        elif opt in ("-i", "--input"):
            traffic_raw_in = f_getRawList(arg)
            traffic_in     = f_getTrafficDict(traffic_raw_in)
        elif opt in ("-I", "--Input"):
            traffic_raw_in = f_getIncreList(arg)
            traffic_in     = f_getTrafficDict(traffic_raw_in)
        elif opt in ("-D"):
            traffic_raw_in,m_rowNum,m_colNum,traffic_s,traf_step = f_get2DIncreList(arg)
            traffic_in     = f_getTrafficDict(traffic_raw_in)
            PARAS.ACTION_PARAS[IN_TRAFFIC_ROW] = m_rowNum
            PARAS.ACTION_PARAS[IN_TRAFFIC_COL] = m_colNum
            PARAS.ACTION_PARAS[IN_TRAFFIC_SLIST] = traffic_s
            PARAS.ACTION_PARAS[IN_TRAFFIC_STEP] = traf_step
        elif opt in ("-s"):
            (PARAS.ACTION_PARAS[TRAFFIC_SCALE],
             PARAS.ACTION_PARAS[OPW_SCALE]) = f_getScale(arg)
            trafficScale = PARAS.ACTION_PARAS[TRAFFIC_SCALE]
        elif opt in ("-d"):
            delay_th   = f_getPara(arg)
            
            
    # add global paras        
    PARAS.ACTION_PARAS[CLI_MODE] = cli_mode
    PARAS.ACTION_PARAS[MAT_MODE] = mat_mode
    PARAS.ACTION_PARAS[PING_ACTION] = ping_mode
    PARAS.ACTION_PARAS[DELAY_TH] = delay_th
            
    try:
        topo_fname = argv[0]   
    except ValueError:
        print "can not open <topo.csv>"
        sys.exit(2)
    #print traffic_in,weight_raw_in
    #raw_input("check inputs")
    #print 'input mat file is: ', mat_fname 
    if trafficScale != 1.0:
        scaledTraffic = f_rescaleTrafficDict(traffic_in,trafficScale) 
    else:
        scaledTraffic = traffic_in    
    f_test_topo(topo_fname,mat_fname,scaledTraffic,weight_raw_in,cli_mode)
     
        
if __name__ == "__main__":
    main(sys.argv[1:])         
##   setLogLevel('info')

##topos = { 'custom': ( lambda: CustomTopo() ) }
