#!/usr/bin/python

'''
Cloud project:
- socket processing for MN and POX 
Tony Lin
'''

import os,sys
#import re
#import time
import socket
import select
import logging
#sys.path.append('/home/mininet/Sdn_src/exp/src/')
from utils.constant_sys import * 
from utils.proj_mn_process import f_generate_traffic, f_kill_iperf, Traffic_process#,f_send_pox_inform
from utils.proj_util import f_logNetBw,f_logNetBw_W

#tem hard constants
from mininet.cli import CLI
from utils.read_bwm import BwmCsvParser
import time
#MN_PORT = 23100
TEM_ratio = {'E1':[0.25,0.25,0.25,0.25],'E2':[1.0,0.0,0.0,0.0]}


class MnServer:
    def __init__(self,topoFileName,net,topo,server_list,switch_list,hostTraffic_dict,weight_raw_in,
                 period=10000000,port=MN_PORT):
        '''input'''
        self.topoFileName = topoFileName
        self.net  = net 
        self.topo = topo
        self.hostTraffic_dict_list = hostTraffic_dict
        self.weight_raw_in         = weight_raw_in
        self.server_list = server_list
        self.switch_list = switch_list
        self.iperfPeriod = period
        self.port = port
        self.io_mat_f = b''
        '''logging related'''
        self.diable_items = ','.join(self.switch_list)+','+SYS_ENTITY
        '''iperf PID'''
        self.pidHistDict = None
        '''status'''
        self.workFinish = False 
        self.dataCnt = 0
        '''process obj'''
        self.trafficProcessor = None
        # start server socket
        self.f_init_server_socket()            
    def f_init_server_socket(self):
        try:
            self.srvsock = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
            result = self.srvsock.connect_ex(("", self.port))
            print 'socket result=',result
#             if(result == 0) :
#                 self.srvsock.shutdown(socket.SHUT_RDWR)
#                 self.srvsock.close()
#                 self.srvsock = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
            self.srvsock.setsockopt( socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 )
            #print 'socket ok here 1'
            if (result == 0) :
                self.srvsock.bind( ('localhost', 0) )
            else:
                self.srvsock.bind( ("", self.port) )
            #print 'socket ok here 2'
            self.srvsock.listen( 5 )
            self.descriptors = [self.srvsock]
        except socket.error as err:
            print "socket creation failed at client with error %s" %(err) 
            return False
        print 'ChatServer started on port %s' % self.port
        return True
        
    def accept_new_connection( self ):       
        
        ''' receive new connection '''
        newsock, (remhost, remport) = self.srvsock.accept()
        self.descriptors.append( newsock )
        print 'remhost:%s, remport:%d' %(remhost, remport)
        # send connection ack
        newsock.send(CONN_CONFIRM)
        
    def run( self ):
        '''loop for MN and POX communications and processes'''
        while 1:
            # Await an event on a readable socket descriptor
            #print self.descriptors
            (sread, swrite, sexc) = select.select( self.descriptors, [], [] )
            # Iterate through the tagged read descriptors
            for sock in sread:
                self.f_mnSocketProcess(sock)
            if self.workFinish:
                return True
    def f_mnSocketProcess(self,sock):
        # Received a connect to the server (listening) socket
        # iperf ok here        
        if sock == self.srvsock:           
            self.accept_new_connection()
            #raw_input("Press enter to continue") #os.system("pause")
            #CLI( self.net )
        else:            
            # Received something on a client socket
            str_in = sock.recv(1024)
            #print 'at MN receive %s' %str_in            
            # Check to see if the peer socket closed
            if str_in == '':
                self.f_closeSocket(sock)
            elif str_in == INIT_POX_DONE:
                self.trafficProcessor = Traffic_process(self.topoFileName,self.net,self.topo,self.server_list,
                                                        self.switch_list,self.hostTraffic_dict_list,self.weight_raw_in,
                                                        self.io_mat_f,self.iperfPeriod)
                self.trafficProcessor.run() # start() #.f_run()
                #raw_input("Press enter to continue") #os.system("pause")
                sys.exit()
                CLI( self.net )
                # notify POX traffic ready
                f_send_pox_inform(msg=MSG_TRAFFIC_FIRED) # sock.send(MSG_TRAFFIC_FIRED)
            elif str_in == MSG_SENDNEWRATIO:
                f_send_pox_inform(msg=str(TEM_ratio)) 
            elif str_in == MSG_NEXTTRAFFIC:
                # loop to next traffic
                if self.dataCnt < len(self.hostTraffic_dict_list):
                    self.pidHistDict = f_generate_traffic(self.net,self.topo,self.hostTraffic_dict_list[self.dataCnt],
                                                          self.server_list,self.iperfPeriod,self.pidHistDict)
                    self.dataCnt += 1
                else:
                    f_send_pox_inform(MSG_NOTRAFFIC)                    
            elif str_in == MSG_ALLDONE:
                f_kill_iperf(self.net,self.server_list)
                self.workFinish = True
            else: # TBD
                host,port = sock.getpeername()
                newstr = '[%s:%s] %s' % (host, port, str_in)

    def f_closeSocket(self,sock):
        #host,port = sock.getpeername()
        sock.close
        self.descriptors.remove(sock)
        
def f_send_pox_inform(msg,port=POX_PORT):
    ''' send information to POX in a fire-and-forget way'''
    try:
        pox_sock = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
        pox_sock.connect( ('', int(port)) )
    except (socket.error,ValueError) as e:
        logging.warn('Failed to create connection: %s', e.strerror)
        return False
    pox_sock.send(msg)
    #response = pox_sock.recv(100)
    pox_sock.close()
    return True

'''depleted class (not working due to multi-threading problem for POX, to be removed)'''    
class PoxClient:
    def __init__(self,diable_items,port=MN_PORT):
        '''input'''
        self.diable_items = diable_items
        self.port = port
        
        self.descriptors = []
        self.clisock = None
        self.f_init_connect_mn(msg=INIT_POX_DONE)
        '''status'''
        self.workFinish = False
    def f_init_connect_mn(self,msg=INIT_POX_DONE):
        if not self.clisock: # !!!!!!!!!!!!need to confirm if close status means None
            try:
                self.clisock = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
                self.clisock.connect( ('', self.port) )
#                self.clisock = socket.create_connection(('', self.port), 5,('0.0.0.0', 9001))
            except socket.error as e:
                logging.warn('Failed to create connection: %s', e.strerror)
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
        self.clisock.settimeout(None)  # set no timeout 
        if self.clisock not in self.descriptors:        
            self.descriptors.append(self.clisock)
        print 'pox connected to MN'
        
    def run( self ):
        '''loop for MN and POX communications and processes'''
        while 1:
            if self.workFinish:
                return True
            # Await an event on a readable socket descriptor
            (sread, swrite, sexc) = select.select( self.descriptors, [], [] )
            # Iterate through the tagged read descriptors
            for sock in sread:
                self.f_poxSocketProcess(sock)  
            print 'loop in while'    
                                
    def f_poxSocketProcess(self,sock):
        print 'in f_poxSocketProcess'
        if sock == self.clisock:
            print 'in self.clisock Process'
            # Received something from a server
            str_in = sock.recv(100)
            print 'at POX receive %s' %str_in
            # Check to see if the peer socket active
            if str_in == '': #Not active
                self.f_closeSocket(sock)               # close first 
                self.f_init_connect_mn(MSG_RECONNREQ)  # then try to re-connect
            elif str_in == MSG_TRAFFIC_FIRED:  # mn fired traffic
                # begin measure system bandwidth
                print 'start measure traffic'
                f_logNetBw(NETBW_FILE,self.diable_items)
                # obtain bandwidth measurement output
                
                # start traffic distribution calculation
                
                # config new traffic distribution ratio for each edge router
                
                # send calculation finish message and require next data traffic
                sock.send(MSG_NEXTTRAFFIC)
            elif str_in == MSG_NOTRAFFIC:
                # process final results
                
                # send MSG_ALLDONE
                sock.send(MSG_ALLDONE)
                # close socket
                self.f_closeSocket(sock)
                self.workFinish = True
            else:
                print 'message: %s not supported' %(str_in)

    def f_closeSocket(self,sock):
        sock.close
        self.descriptors.remove(sock)

def f_init_server_socket_old(port=MN_PORT,req_str=INIT_POX_DONE,sendStr=MSG_ACK,buffsize=1024):
    try:
        sock = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
        #host = socket.gethostname()
        #print 'new server name:%s' %(host)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #print port
        sock.bind( ('', port) )
        #sock.listen(1) # !!!only for SOCK_STREAM
    except socket.error as err:
        print "socket creation failed at server with error %s" %(err)
        return False
    msg = None    
    while msg!=req_str:
        msg, (addr,cport) = sock.recvfrom( buffsize )
        print msg, addr, cport
    sock.sendto(sendStr, (addr,cport))
    sock.close()
    return True
        
def f_init_client_socket_old(port=MN_PORT,send_str=INIT_POX_DONE,req_response=MSG_ACK,buffsize=1024):
    try: 
        sock = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
    except socket.error as err:
        print "socket creation failed at client with error %s" %(err) 
        return False 
    msg = None
    sock.settimeout(1)  # set socket timeout 1 second for the following operation
    while msg != req_response:
        sock.sendto( send_str, ('', port) )        
        try:
            msg =  sock.recv( buffsize )
        except socket.timeout:
            msg=None
            
    print 'server said %s' %(msg)   
    sock.close()
    print "handshake with server finished"
    return True          
    
if __name__ == "__main__":
    ssocket = f_init_server_socket()
    csocket = f_init_client_socket()
        
