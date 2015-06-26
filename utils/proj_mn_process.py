#!/usr/bin/python

'''
Cloud project:
- Software Defined Networking (SDN) 
-- project related process

Tony Lin
'''
import re
import os
import time
import socket
import numpy as np
from threading import Thread

from utils.constant_sys import * 
from utils.proj_util import f_logNetBw,f_logNetBw_W, f_getTopoFile, f_load_mat, f_scaleTraffic, f_parse_ping_out
##from utils.proj_socket import f_send_pox_inform 
from utils.read_topo import TopoInfor
from utils.read_bwm import BwmCsvParser
from alg.alg_global import projSolver
from alg.alg_miqp import miqpSolver
from alg.alg_miqp_delay import miqpSolverDelay
from utils.proj_analyser import ResultsRecorder_P1
import utils.paras as PARAS
#tem hard constants
from mininet.cli import CLI
TEM_ratio = {'J1':[0.0,0.25,0.25,0.0,0.25,0.25],'J2':[0.0,1.0,0.0,0.0,0.0,0.0]}

def f_generate_traffic(net,topo,hostTraffic_dict,server_list,period=None,pidHistDict=None):
    #print 'host_list=',PARAS.MININET_PARAS[HOST_LIST]
    pidHistDict_out = {}
    if not pidHistDict: 
        # clear previous iperf process if exists    
        os.system( 'sudo killall iperf > /dev/null 2>&1' )
    for host in net.hosts:    
        if host.name not in server_list:
            # add iperf UDP client
            #print host.name
            vip = re.split('/',topo.hostVdstIpDict[host.name])[0]
            traffic = hostTraffic_dict[host.name]
#             print 'vip=',vip,'traffic=',traffic
            if vip:
                if pidHistDict and host.name in pidHistDict.keys():
                    killPid = pidHistDict[host.name]
                    killMsg = host.cmdPrint ( 'sudo kill -9 %s > /dev/null 2>&1' %(killPid))
                    #print '\nAt %s, killMsg=%s' %(host.name,killMsg)
#                 print 'setup iperf udp client at %s with %s' %(host.name,vip)   
                #print 'iperf -c %s -u -b %s -t %d &' %(vip,traffic,period)                
                if period:        
                    msg=host.cmdPrint( 'iperf -c %s -u -b %s -t %d & echo $!' %(vip,traffic,period))
                    #host.cmd( 'iperf -c %s -u -b 0.8m -t %d &' %(vip,period))
                else:
                    msg=host.cmdPrint( 'iperf -c %s -u -b %s & echo $!' %(vip,traffic))
                #iperf_pid = os.environ['ppp']
                #os.system( 'unset ppp' )
                try:
                    #print int(msg)
                    #[int(s) for s in tt.split() if s.isdigit()]
                    pidHistDict_out[host.name] = int(msg.split()[0])
                except:
                    print 'can not transfer msg=%s into PID ' %msg
                    return None
    return pidHistDict_out               

def f_ping_process(net,topo,host_list,cdf_log_en,
                   pingNum=1000,preload=6,interval=0.1):
    #host_list = PARAS.MININET_PARAS[HOST_LIST]
    ping_all = {}
    ping_all_single = {}
    singlePings = []
    for host in net.hosts:
        if host.name not in host_list:
            continue
        vip = re.split('/',topo.hostVdstIpDict[host.name])[0]
        if vip:
            if cdf_log_en: # log out each single ping out for CDF plot
                msg = host.cmdPrint( 'ping -c %d %s -i %2.1f -l %d' %(pingNum,vip,interval,preload))
            else:
                msg = host.cmdPrint( 'ping -c %d %s -i %2.1f -l %d -q' %(pingNum,vip,interval,preload))
            #print 'ping msg: ',msg
            try:
                #print 'cdf_log_en=',cdf_log_en
                ping_all[host.name],singlePings = f_parse_ping_out(msg,cdf_log_en)
                #print cdf_log_en#,ping_all[host.name]
                #print 'singlePings=',singlePings
                if cdf_log_en and singlePings:
                    ping_all_single[host.name]=singlePings
            except:
                print 'ping out logging not successful'
                fid = open(ERROR_LOG, 'a')
                fid.write('ping out error due to:\n%s' %msg)
                fid.close()
                return False
            #raw_input("Press enter to continue")
    return ping_all,ping_all_single      

'''depleted method, to be removed '''                    
def f_kill_iperf(net,server_list):
    for host in net.hosts:    
        if host.name not in server_list:
            host.cmd( 'killall iperf 2>&1 /dev/null' )

def f_send_pox_inform_mn(msg,port=POX_PORT):
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
            
class Traffic_process(): #(Thread):
    def __init__(self,topoFileName,net,topo,
                 server_list,switch_list,hostTraffic_dict,weight_raw_in,
                 mat_f,iperfPeriod=100):
        '''input'''
        self.topoFileName = topoFileName
        self.net  = net 
        self.topo = topo
        self.hostTraffic_dict_list = hostTraffic_dict
        self.weight_raw_in         = weight_raw_in
        self.host_list = PARAS.MININET_PARAS[HOST_LIST]
        #print self.hostTraffic_dict_list, self.weight_raw_in
        self.server_list = server_list
        self.switch_list = switch_list
        self.iperfPeriod = iperfPeriod
        self.mat_f = mat_f
        '''init member'''
        self.init_m()
        '''init topo RES para'''
        self.init_topRes()
        '''logging related'''
        self.diable_items = ','.join(self.switch_list)+','+SYS_ENTITY
        '''iperf PID'''
        self.pidHistDict = None
        '''status'''
        self.workFinish = False 
        self.dataCnt = 0
        '''bwm csv reader'''
        self.bwmParserObj = None
        ''' start logging bwm-ng'''
        self.f_startBwm()
    def init_m(self):
        self.m_firstopen = 1 # init log indicator
        self.m_trueTrafficDict = {}
        try:
            self.m_cdf_log_en = True #PARAS.ACTION_PARAS[CDF_EN]
        except:
            self.m_cdf_log_en = False                
        if (IN_TRAFFIC_ROW in PARAS.ACTION_PARAS.keys() 
            and IN_TRAFFIC_COL in PARAS.ACTION_PARAS.keys()):
            self.m_2D_en = True
            self.m_rowNum = PARAS.ACTION_PARAS[IN_TRAFFIC_ROW]
            self.m_colNum = PARAS.ACTION_PARAS[IN_TRAFFIC_COL]
            self.m_rowCnt = 0
            self.m_colCnt = 0
            self.m_totalPowMat = np.zeros((self.m_rowNum,self.m_colNum))
        else:
            self.m_2D_en = False
            self.m_rowNum = 0
            self.m_colNum = 0
        try:
            self.m_inTrafficScaleInv = 1.0/float(PARAS.ACTION_PARAS[TRAFFIC_SCALE])
        except:
            self.m_inTrafficScaleInv = 1.0
            
    def init_topRes(self):
        ''' '''
        tf = f_getTopoFile(self.topoFileName)
        if tf:
            infor_obj = TopoInfor(tf)
        else:
            infor_obj = TopoInfor(TOPO_FILE)
        self.host_list   = infor_obj.host_list
        self.switch_list = infor_obj.switch_list
        self.server_list = infor_obj.server_list
        self.link_list   = infor_obj.link_list              
        self.sortedResList = infor_obj.sortedResList
        self.traffic_in  = None
        self.m_solver    = None
        ''' RES related'''
        
        self.resCapacity  = infor_obj.resCapacity     # C
        self.resPmax      = infor_obj.resPmax         # PMAX
        self.resBeta      = infor_obj.resBeta         # Beta
        self.resServer_num = infor_obj.resServer_num  # I
        self.resMux_num   = infor_obj.resMux_num      # H        
        self.resLink_num  = infor_obj.resLink_num     # L
        self.userRouteMat = infor_obj.userRouteMat    # EM matrix for user/route mapping
        self.reRouteMat   = infor_obj.reRouteMat      # RM matrix for res/route mapping        
        
        self.resUtilRatio = infor_obj.resUtilRatio    # RES_ratio
        
        self.resDelay     = infor_obj.resDelay
        
        self.resMeasKeys  = infor_obj.resMeasKeys
        self.hostMeasKeys = infor_obj.hostMeasKeys
        self.hostSwitch = [keys.split('-')[0] for keys in self.hostMeasKeys]
        self.ratioDict = {}
        
        self.init_matPara()
        '''
        D,C,PMAX,Beta,I,H,L,EM,RM,
        Weight,Threshold,Iteration,NormMode,Outloop,
        GameMode,RES_ratio,constraint_mode,
        i_fileName=o_file
        '''
    def init_matPara(self):
        if not self.mat_f:
            if PARAS.ACTION_PARAS[MAT_MODE]:
                raise ('can not file mat file with mat mode on')
            return
        print 'self.mat_f=', self.mat_f
        matIn = f_load_mat(self.mat_f)
        if not self.weight_raw_in:
            self.weight_raw_in = [matIn['Weight']]  # use mat file weight if no weight input        
        self.Threshold = matIn['Threshold'].flatten()[0]
        self.Iteration = matIn['Iteration']
        self.NormMode = []
        self.Outloop = 0
        self.GameMode = matIn['Mode'][0]
        self.constraint_mode = 'lag'
        #o_file = 'fmin_iteration_out_sdn.txt'
        if 0:
            self.debug_inputPara()
    def debug_inputPara(self):
        print self.resCapacity  # C
        print self.resPmax # PMAX
        print self.resBeta # Beta
        print self.resServer_num # I
        print self.resMux_num # H        
        print self.resLink_num # L
        print self.userRouteMat # EM matrix for user/route mapping
        print self.reRouteMat # RM matrix for res/route mapping        
        
        print self.resUtilRatio # RES_ratio
        print self.resDelay     # delay for RES
        
        raw_input("check input paras")
       
    def f_startBwm(self,measurePeriod=200):
        ''' start logging bwm-ng'''
        f_logNetBw_W (NETBW_FILE,self.diable_items,measurePeriod)
    
    def run(self):
        for self.Weight in self.weight_raw_in:
            self.dataCnt = 0
            for trafficIdx in range(len(self.hostTraffic_dict_list)):
                print self.hostTraffic_dict_list[trafficIdx]
                self.m_trueTrafficDict = self.hostTraffic_dict_list[trafficIdx]
                self.pidHistDict = f_generate_traffic(self.net,self.topo,self.hostTraffic_dict_list[trafficIdx],
                                                      self.server_list,self.iperfPeriod,self.pidHistDict)
    #             print self.pidHistDict                
                # parse out user input traffic
                time.sleep(2)
    #            raw_input("Press enter to continue")
                self.f_preTrafficCapture(trafficIdx)  
    #             print 'self.traffic_in=', self.traffic_in         
                self.f_runSolver(trafficIdx) 
                # wait and capture results
    #             print 'start long sleep'
                time.sleep(2.5)  
                time.sleep(2.6)          
                # 
                self.f_postAveTrafficMeasure()                
                # debug 
                #CLI( self.net )                               
                self.m_recorder.i_cntIdx = self.dataCnt
                # try to run ping emulation
                self.f_pingEmulation() 
                # analyse and log results               
                self.m_recorder.f_analyse()
                if self.m_firstopen:
                    self.m_firstopen = 0
                self.dataCnt += 1
                #raw_input("Finish 1 input config")
            #raw_input("Finish all inputs for 1 weights setup")
    def f_preTrafficCapture(self,trafficIdx):
        # initial bwg-ng parser
        if trafficIdx==0:
            self.bwmParserObj = BwmCsvParser(NETBW_FILE)
            self.f_read_multiInput()
        else:
            self.bwmParserObj.f_readNew()
            self.f_read_multiInput()
    #        raw_input("Press enter to continue")
        self.traffic_in = f_scaleTraffic(self.bwmParserObj.f_getMapOut(self.hostMeasKeys),
                                         J1TRAFFIC_SCALE*self.m_inTrafficScaleInv)  # D in solver, scale down 1e-7
    def f_read_multiInput(self):
        for cnt in range(5):
            time.sleep(1)
            self.bwmParserObj.f_readNew(readMode='a')
    ''' debug code '''        
    def f_run_test(self):
        # start fire traffic
        self.pidHistDict = f_generate_traffic(self.net,self.topo,self.hostTraffic_dict_list[0],
                                              self.server_list,self.iperfPeriod,self.pidHistDict)
        self.dataCnt += 1
##        f_send_pox_inform(msg=MSG_TRAFFIC_FIRED) # sock.send(MSG_TRAFFIC_FIRED) 
        #print self.pidHistDict
        raw_input("Press enter to continue") #os.system("pause")
        self.bwmParserObj = BwmCsvParser(NETBW_FILE)
        #print self.bwmParserObj.pars_status , self.bwmParserObj.memDepth , self.bwmParserObj.start_intName
        print self.bwmParserObj.f_getMapOut(self.resMeasKeys)
        self.traffic_in = f_scaleTraffic(self.bwmParserObj.f_getMapOut(self.hostMeasKeys),
                                         J1TRAFFIC_SCALE*self.m_inTrafficScaleInv)  # D in solver, scale down 1e-7
        print self.traffic_in
        self.f_initSolver()
        self.f_getRatioDict(self.m_solver.o_ratioList)
        #print self.ratioDict
#        f_send_pox_inform_mn(msg=str(TEM_ratio)) 
        f_send_pox_inform_mn(msg=str(self.ratioDict)) 
#        raw_input("FMIN slover") #os.system("pause")
        #############################################
        time.sleep(0.81)
        self.bwmParserObj.f_readNew()
        rawMeasureData = f_scaleTraffic(self.bwmParserObj.f_getMapOut(self.resMeasKeys),
                                        J1TRAFFIC_SCALE*self.m_inTrafficScaleInv) # D in solver, scale down 1e-7
        self.m_recorder.bwMeasure = [RECORD_ONCE,rawMeasureData]
        for t in range(3):
            time.sleep(0.8)
            self.bwmParserObj.f_readNew()
            rawMeasureData = f_scaleTraffic(self.bwmParserObj.f_getMapOut(self.resMeasKeys),
                                        J1TRAFFIC_SCALE*self.m_inTrafficScaleInv) # D in solver, scale down 1e-7
            print rawMeasureData
            self.m_recorder.bwMeasure = [RECORD_MULTI,rawMeasureData]
        self.m_recorder.f_analyse()
        #############################################
        #print self.bwmParserObj.pars_status , self.bwmParserObj.memDepth , self.bwmParserObj.start_intName
        print self.bwmParserObj.f_getMapOut(self.resMeasKeys)
        print self.bwmParserObj.f_getMapOut(self.hostMeasKeys)
        raw_input("Finish recorder") #os.system("pause")
        self.bwmParserObj.f_readNew()
        #print self.bwmParserObj.pars_status , self.bwmParserObj.memDepth , self.bwmParserObj.start_intName
        print self.bwmParserObj.f_getMapOut(self.resMeasKeys)
        print self.bwmParserObj.f_getMapOut(self.hostMeasKeys)
        self.pidHistDict = f_generate_traffic(self.net,self.topo,self.hostTraffic_dict_list[0],
                                              self.server_list,self.iperfPeriod,self.pidHistDict)
        #print self.pidHistDict.pars_status, 
        raw_input("Press enter to continue") #os.system("pause")
        CLI( self.net )
    #def f_scaleTraffic(self,):
    def f_initSolver(self):
        if not self.mat_f:
            return
#         self.m_solver = projSolver(self.traffic_in,self.resCapacity,self.resPmax,self.resBeta,
#                       self.resServer_num,self.resMux_num,self.resLink_num,self.userRouteMat,self.reRouteMat,
#                       self.Weight,self.Threshold,self.Iteration,self.NormMode,self.Outloop,
#                       self.GameMode,self.resUtilRatio,self.constraint_mode,
#                       i_fileName=SOLVER_OUT)
        ''' disable RES ratio for MIQP solver '''
        self.resUtilRatio = None 
        delayTh = PARAS.ACTION_PARAS[DELAY_TH]
        if delayTh:
            self.m_solver = miqpSolverDelay(self.traffic_in,self.resCapacity,self.resPmax,self.resBeta,
                      self.resServer_num,self.resMux_num,self.resLink_num,self.userRouteMat,self.reRouteMat,
                      self.Weight,self.Threshold,self.Iteration,self.NormMode,self.Outloop,
                      self.GameMode,self.resUtilRatio,self.constraint_mode,self.resDelay,delayTh,
                      i_sdiag = None, normScale=18.0,
                      i_fileName=SOLVER_OUT,LB_mode='global')
        else:
            self.m_solver = miqpSolver(self.traffic_in,self.resCapacity,self.resPmax,self.resBeta,
                      self.resServer_num,self.resMux_num,self.resLink_num,self.userRouteMat,self.reRouteMat,
                      self.Weight,self.Threshold,self.Iteration,self.NormMode,self.Outloop,
                      self.GameMode,self.resUtilRatio,self.constraint_mode,
                      i_sdiag = None, normScale=18.0,
                      i_fileName=SOLVER_OUT,LB_mode='global')
        
        
        self.m_recorder = ResultsRecorder_P1(self.traffic_in,self.resCapacity,self.resPmax,self.resBeta,
                      self.resServer_num,self.resMux_num,self.resLink_num,self.userRouteMat,self.reRouteMat,
                      self.Weight,self.Threshold,self.Iteration,self.NormMode,self.Outloop,
                      self.GameMode,self.resUtilRatio,self.constraint_mode,plot_en=1)
        self.m_recorder.m_firstopen = self.m_firstopen
        self.m_recorder.m_firstopen_sim = self.m_firstopen
        self.m_recorder.m_firstopen_ping = self.m_firstopen
        self.m_recorder.m_tPowfirstopen = self.m_firstopen
        
        #self.m_recorder.
        self.m_recorder.m_title_on_sim = 0
    def f_runSolver(self,trafficIdx):
        if trafficIdx==0:
            # initial solver and recorder
            self.f_initSolver()
        else:
            # set input para for solver and recorder
            traffic_array_in = np.array(self.traffic_in)
            self.m_solver.i_traffic_in = traffic_array_in
            self.m_solver.f_recalculate()
            self.m_recorder.i_traffic_in = traffic_array_in
        # get route ratio output list: self.ratioDict    
        self.f_getRatioDict(self.m_solver.o_ratioList)
        print 'self.ratioDict=',self.ratioDict
        # send ratio to pox controller
        f_send_pox_inform_mn(msg=str(self.ratioDict))        
    def f_postAveTrafficMeasure(self):
        for captureIdx in range(10):
            # read bwg-ng output
            self.bwmParserObj.f_readNew()
            # parse res bandwidth
            rawMeasureData = f_scaleTraffic(self.bwmParserObj.f_getMapOut(self.resMeasKeys),
                                            J1TRAFFIC_SCALE*self.m_inTrafficScaleInv) # D in solver, scale down 1e-7
            traffic_in = f_scaleTraffic(self.bwmParserObj.f_getMapOut(self.hostMeasKeys),
                                        J1TRAFFIC_SCALE*self.m_inTrafficScaleInv)
            print captureIdx
            #print rawMeasureData
            #print traffic_in
            if captureIdx == 0: # first time
                self.m_recorder.bwMeasure = [RECORD_ONCE,rawMeasureData]
            else:               # append results
                self.m_recorder.bwMeasure = [RECORD_MULTI,rawMeasureData]
            # wait for another results
            time.sleep(0.81)    
    def f_pingEmulation(self):
        if not PARAS.ACTION_PARAS[PING_ACTION]:
            return
        ''' run ping process '''
        out = False
        while not out:
            #print 'self.m_cdf_log_en=',self.m_cdf_log_en
            out,single_out = f_ping_process(self.net, self.topo,self.host_list,
                                            self.m_cdf_log_en) #,pingNum=60)  #
        self.m_recorder.pingMeasure = out
        self.m_recorder.m_trueTrafficDict = self.m_trueTrafficDict
        self.m_recorder.pingMeasureSingle = single_out
        print 'ping log finished'
            
    def f_getRatioDict(self,ratioList):
        self.ratioDict = {}
        assert(len(ratioList) == len(self.hostSwitch))
        for k,v in enumerate(self.hostSwitch): 
            self.ratioDict[v] = ratioList[k]
        
                
        # notify POX traffic ready
               
