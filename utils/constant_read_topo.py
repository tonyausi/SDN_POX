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
LINK_TYPE   = 'link'
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
# new para for CAP,PMAX,BETA,RES_RATIO
# RES_CPA     = 'S'         # RES capacity
# PMAX        = 'PMAX'      # RES maximal power
# BETA        = 'Beta'      # Ratio of idle power
# RES_RATIO   = 'res_ratio' # RES utilisation ratio
RES_PARA_DELIMITER = [':',';']
SRC_PARA    = 'SRC_PARA'  # [RES_CPA:PMAX:BETA:RES_RATIO]
DST_PARA    = 'DST_PARA'  # [RES_CPA:PMAX:BETA:RES_RATIO]
#NODE_PARA_LEN = 3
LINK_PARA   = 'LINK_PARA' # [RES_CPA:PMAX:BETA:RES_RATIO]
LINK_NAME   = 'LINK_NAME' # link name
NULL_STR    = b''
J1_PARA_LEN = 3           # Jouranl 1 RES para length
J2_PARA_LEN = 4           # Jouranl 2 RES para length
J2_PARA_DELAY_LEN = 5     # with utilisation and delay paras
ROW_FORMAT = {'names': (VID, LINKOPTS,
                        SRC_NAME, DST_NAME,
                        SRC_TYPE, DST_TYPE,
                        SRC_ID  , DST_ID,
                        SRC_MAC , DST_MAC,
                        SRC_PORT, DST_PORT,
                        SRC_VID , DST_VID,
                        SRC_PARA , DST_PARA, LINK_PARA,LINK_NAME),
              'formats': ('i4', 'S40',
                          'S4', 'S4',
                          'S6', 'S6',
                          'S60','S40',
                          'S20','S20',
                          'i4', 'i4',
                          'S40','S40',
                          'S40' ,'S40','S40','S4')}

USER_ROUT_MAT  = 'EM'     # user-route mapping matrix
RES_ROUT_MAT   = 'RM'     # res_route mapping matrix

# bwm-ng csv file format
TIME          = 'timestamp'
INT           = 'interface'   
RO_BYTE       = 'bytes_out/s'
RI_BYTE       = 'bytes_in/s'
RA_BYTE       = 'bytes_total/s'    
I_BYTE        = 'bytes_in'
O_BYTE        = 'bytes_out'            
RO_PACKET     = 'packets_out/s'
RI_PACKET     = 'packets_in/s'
RA_PACKET     = 'packets_total/s'  
I_PACKET      = 'packets_in'
O_PACKET      = 'packets_out'
RO_ERR        = 'errors_out/s'
RI_ERR        = 'errors_in/s'
I_ERR         = 'errors_in'
O_ERR         = 'errors_out'      
BWM_CSVFORMAT = {'names':  (TIME,INT,                     # unix timestamp     interface
                            RO_BYTE,RI_BYTE,RA_BYTE,      # bytes_out/s     bytes_in/s     bytes_total/s
                            I_BYTE,O_BYTE,                # bytes_in     bytes_out
                            RO_PACKET,RI_PACKET,RA_PACKET,# packets_out/s     packets_in/s     packets_total/s 
                            I_PACKET,O_PACKET,            # packets_in     packets_out
                            RO_ERR,RI_ERR,                # errors_out/s     errors_in/s
                            I_ERR,O_ERR),                 # errors_in     errors_out
                 'formats':('i4','S10',                   # i4-->32-bit signed integer, f8-->64-bit floating-point number
                            'f8','f8','f8',
                            'i4','i4',
                            'f8','f8','f8',
                            'i4','i4',
                            'f8','f8',
                            'i4','i4')}
BWM_CSVDATA = [RO_BYTE,RI_BYTE,RA_BYTE,
               I_BYTE,O_BYTE,
               RO_PACKET,RI_PACKET,RA_PACKET,
               I_PACKET,O_PACKET,
               RO_ERR,RI_ERR,
               I_ERR,O_ERR]
BWM_CSVDATA_RBYTE = [RO_BYTE,RI_BYTE,RA_BYTE]
BWMCSV_DELIMITER = 'total'
PACKET_OVERHEAD = 1500.0 / 1470.0 # 1.028571429 # iperf packet overhead