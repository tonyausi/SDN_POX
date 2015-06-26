#!/usr/bin/python

'''
Cloud project:
- Software Defined Networking (SDN) 
-- global parameters

Tony Lin
'''

''' Mininet related parameters
keys: TOPO <-- mininet topology obj
      NET  <-- mininet net obj
      SWITCH_LIST <-- list of switches
      HOST_LIST   <-- list of hosts
      SERVER_LIST <-- list of servers

'''
MININET_PARAS = {} 
''' Operation related parameters
keys: CLI_MODE <-- True: mininet command line operation , False: mininet-controller socket operation
      PING_ACTION <-- True: enable ping emulation (not for CLI_MODE=True)

'''
ACTION_PARAS  = {}