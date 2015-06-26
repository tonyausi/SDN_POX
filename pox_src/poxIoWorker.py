#!/usr/bin/python

'''
Cloud project:
- socket processing for POX 
  -- mainly based on pox.lib.ioworker
                     pox.lib.recoco
Tony Lin
'''

"""
Run the server as:
 lib.ioworker.notify_demo:server

Clients can be run in several ways...

To just listen for notifications and show them as log messages:
 lib.ioworker.notify_demo:client --server=127.0.0.1 --name=SirSpam

To send a notification and quit, append --msg="Spam eggs spam".
"""

from pox.lib.ioworker import *
from pox.lib.ioworker.workers import *
from pox.core import core

sys.path.append('/home/mininet/Sdn_src/exp/src/')
from utils.constant_sys import * 

log = core.getLogger()

# ---------------------------------------------------------------------------
# Server Stuff
# ---------------------------------------------------------------------------

class ServerWorker (TCPServerWorker, RecocoIOWorker):
    pass

clients = set()

class NotifyWorker (RecocoIOWorker):
    def __init__ (self, *args, **kw):
        super(NotifyWorker, self).__init__(*args, **kw)
        self._connecting = True
        self.data = b''

    def _handle_close (self):
        log.info("Client disconnect")
        super(NotifyWorker, self)._handle_close()
        clients.discard(self)

    def _handle_connect (self):
        log.info("Client connect")
        super(NotifyWorker, self)._handle_connect()
        clients.add(self)

    def _handle_rx (self):
        self.data += self.read()
        if CMD_ESCAPE in self.data: # '\n' in received data
            # multiple commands
            while CMD_ESCAPE in self.data:
                msg,self.data = self.data.split('\n',1)
                f_processMnMsg(msg)
        else: # single command
            f_processMnMsg(msg)

def f_processMnMsg(msg):
    if '{' == msg[0] and '}' == msg[-1]: # dict for ratio
        # get ratio dict
        try:
            ratioSetupDict = eval(msg)
        except SyntaxError:
            print 'can not eval dict: %s, discard and wait for next message' %msg
            pass
        # send ratio to each load balancer
        f_sendLbRatio(ratioSetupDict)
        
def f_sendLbRatio(ratioSetupDict):
    for key, ratio in d.iteritems(): 
        print key
        


def PoxServer (port = 23101):
    global loop
    loop = RecocoIOLoop()
    #loop.more_debugging = True
    loop.start()

    w = ServerWorker(child_worker_type=NotifyWorker, port = int(port))
    loop.register_worker(w)