#!/usr/bin/python

'''
Cloud project:
- constant for system operation 
Tony Lin
'''
import os

HOME = '%s' % os.environ[ 'HOME' ]
NETBW_DIR   = HOME + '/projlog'
NETBW_NAME  = 'netbw'
NETBW_FILE  = NETBW_DIR + '/' + NETBW_NAME
SCRIPT = HOME + "/Sdn_src/exp/src/script/"
BWMLOG = SCRIPT+"bwmlog.sh"
BWMLOGW = SCRIPT+"bwmlog_w.sh"

SYS_ENTITY = 'eth0,eth1,lo,ovs-system'
DIR_DELIMITER = ['/']

'''Mininet related'''
SWITCH_LIST = 'switch_list'
HOST_LIST   = 'host_list'
SERVER_LIST = 'server_list'
TOPO        = 'topology'
NET         = 'mininetwork'

'''POX related'''
ARP_CONTROL_PRIORITY = 0x0007 # Pretty high
TOPO_DIR = '%s/Sdn_src/exp/src/topo_config/' % os.environ[ 'HOME' ] 
#TOPO_FILE = "%s/Sdn_src/exp/src/topo_config/nat.csv" % os.environ[ 'HOME' ] 
TOPO_FILE = "%s/Sdn_src/exp/src/topo_config/nat1.csv" % os.environ[ 'HOME' ] 

''' socket related constants'''
CMD_ESCAPE   = '\n'                    # command line end escape. used for multiple commands in multiple lines 
CONN_CONFIRM = 'connection confirmed'  # server confirm connection from client
INIT_POX_DONE = 'pox finish init'      # pox init finished from pox client
MSG_RECONNREQ = 'require reconnection' # pox issue reconnection request
MSG_ACK      = 'got it'
MSG_ALLDONE  = 'all done'
MSG_TRAFFIC_FIRED = 'traffic fired'
MSG_NEXTTRAFFIC = 'calculation for current done, what is next?'
MSG_NOTRAFFIC = 'no more traffic'
MSG_SENDNEWRATIO = 'send new ratio'
MN_PORT = 23100
POX_PORT = 23101

''' solver related'''
OUT_DIR = "%s/Sdn_src/exp/src/log" % os.environ[ 'HOME' ]
SOLVER_OUT = OUT_DIR + "/fmin_iteration_out.txt"
J1TRAFFIC_SCALE = 1.0e-6

''' emulation results related'''
ERROR_LOG   = OUT_DIR + "/error.log"
ERROR_CSV   = OUT_DIR + "/error.csv"
TRAFFIC_OUT = OUT_DIR + "/traffic_log_out_detail.txt"
SIM_OUT     = OUT_DIR + "/traffic_log_out_sim.txt"
PING_OUT    = OUT_DIR + "/log_out_ping.txt"
POW2D_OUT   = OUT_DIR + "/traffic_log_out_pow2D.txt"
PING_ITEMS  = 8     # transmitted,received,lossRatio,time,min,avg,max,mdev
PINT_MAT_OUT_PREFIX= OUT_DIR + "/ping/log_out_pingSingle"
DUMMY_NUM  = -999
RECORD_ONCE = 0
RECORD_MULTI = 1

''' Operation/Action related '''
TRAFFIC_SCALE = 'mininet traffic scale'
OPW_SCALE     = 'solver power objective denominator scale'
CLI_MODE = 'command line mode'
MAT_MODE = 'mat parameter mode'
PING_ACTION = 'ping emulation indicator'
IN_TRAFFIC_ROW = 'row number for input traffic'
IN_TRAFFIC_COL = 'column number for input traffic'
IN_TRAFFIC_SLIST = 'input 2D traffic start list'
IN_TRAFFIC_STEP  = 'input 2D traffic step'
DELAY_TH = 'route delay threshold as constraints'
