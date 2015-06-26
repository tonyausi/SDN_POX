
import os,sys
import re
import time
import random
import numpy as np
import collections # used for dict sorting
#import shutil # High-level file operations,used to delete files in a dir
import scipy.io

from utils.constant_sys import * 

random.seed(0)
#FLOW_IDLE_TIMEOUT = 10
FLOW_MEMORY_TIMEOUT = 10 # 60 * 5
DPIDLEN = 12

def f_logNetBw(file_in,nameExcluded):
    '''Create empty file if not exit'''
    try:
        print file_in
        file = open(file_in,'w')   # Trying to create a new file or open one
        file.close()
        cmd_chmod  = "sudo chmod a+rwx %s" %(file_in)
        # no block, it start a sub process.
        print 'file open is ok'
#         p = subprocess.Popen(cmd_chmod, shell=True, 
#                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#         # and you can block util the cmd execute finish
#         p.wait()
        os.system(cmd_chmod)
    except:
        print('Something went wrong! Can\'t create/open %s' %file_in)
        sys.exit(0) # quit Python
        
    '''log out'''
    cmd_log = "/bin/bash %s %s %s %d %d" %(BWMLOG,file_in,nameExcluded,500,5)
    print cmd_log
    #os.system("pwd;uname -a")
    os.system(cmd_log)
    #p = subprocess.Popen(cmd_log, shell=True, 
    #                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def f_logNetBw_W(file_in,nameExcluded,mode='w',period=500):
    '''get target log dir'''
    target_dir,file_name = f_getTargetDirFile (file_in)
    '''create new dir if not exist'''
    if not os.path.exists(target_dir): os.makedirs(target_dir)
    '''delete all files in a dir'''
    if mode=='w': 
        f_deleteFilesInDir(target_dir)
    '''Create empty file if not exit'''
    try:
        file = open(file_in,'w')   # Trying to create a new file or open one
        file.close()
    except:
        print('Something went wrong! Can\'t create/open %s' %file_in)
        sys.exit(0) # quit Python
    '''log out'''
    cmd_log = "/bin/bash %s %s %s %s &" %(BWMLOGW,file_in,nameExcluded,period)
    #os.system("pwd;uname -a")
    try:
        os.system(cmd_log)
    except:
        print ('script command (%s) NOT working ' %cmd_log )
        sys.exit(0)
    #print cmd_log    

def f_getTargetDirFile (file_in):  
    if not file_in:
        return NETBW_DIR, NETBW_NAME
    '''get log dir'''
    depth_item = re.split('[%s]' % ''.join(DIR_DELIMITER),file_in)
    file_name = depth_item[-1]
    dir_len = len(depth_item)
    if dir_len == 0:
        print('No input file with input %s' %file_in)
        return None,None
    elif dir_len == 1: # not specify dir
        target_dir = NETBW_DIR
    else:
        lastLen = len(file_name) + 1 # +1 means '/'
        target_dir = file_in[:-lastLen]
    if not os.path.isdir(target_dir): os.makedirs(target_dir) 
    return target_dir,file_name           
def f_deleteFilesInDir (targetDir):    
    #shutil.rmtree(target_dir)
    #filelist = [ f for f in os.listdir(".") if f.endswith(".bak") ]
    filelist = [ f for f in os.listdir(targetDir) ]
    #os.system('sudo chmod a+rwx %s' %(targetDir))
    #print filelist
    try:
        for f in filelist: os.remove(targetDir+'/'+f)    
    except OSError as e:
        print 'can not delete files in %s due to error: %s' %(targetDir,e)

def f_getTopoFile(f_name,file_appendix='.csv'):
    '''Try to find a topology csv file'''
    try:
        if os.path.isfile(f_name):
            return f_name
        elif os.path.isfile(f_name+file_appendix):
            return f_name+file_appendix
        elif os.path.isfile(TOPO_DIR+f_name):
            return TOPO_DIR+f_name
        elif os.path.isfile(TOPO_DIR+f_name+file_appendix):
            return TOPO_DIR+f_name+file_appendix
        else:
            return None
    except:
        return None      

def f_load_mat(matfile):
    try:
        mat = scipy.io.loadmat(matfile)
        return mat
    except IOError:
        print 'fail to load mat file: %s and return None' %matfile
        return None  
    
def f_scaleTraffic(targ,scale):  
    try:
        return np.array(targ) * scale
    except:
        print 'can not scale input targ'
        return targ  
    
def f_getRawList(w_in,var='traffic_set'):   
    mat_w = f_getTopoFile(w_in,file_appendix='.mat')
    #print 'mat_w=',mat_w
    if mat_w:
        #print 'weight mat file: ', mat_w 
        matInDict = f_load_mat(mat_w)
        #print 'matInDict=',matInDict
        try:
            if matInDict and var=='traffic_set':
                w_list = f_trafficMatProcess(matInDict,var)
            elif matInDict and ('w' in var or 'W' in var):
                #print 'process weight mat'
                w_list = f_weightMatProcess(matInDict)
            else:
                w_list = None  
        except:
            w_list = None                  
    elif w_in:
        try:
            w_list = eval(w_in)
        except:
            w_list = None
    else:
        w_list = None
    return w_list
def f_trafficMatProcess(matInDict,var):
    t_list = []
    try:
        matIn = matInDict[var]
        for row in range(len(matIn)):
            t = matIn[row,:]
            t_list.append(t)
    except:
        print 'can not parse %s for traffic' %str(matInDict)
    return t_list
def f_weightMatProcess(matInDict):
    #print matInDict.keys()
    w_list = None
    for key in matInDict.keys():
        if key in ['Weight','weight','w','W']:
            try:
                w_list = np.array(matInDict['Weight'])
                w_shape = np.shape(w_list)
                print w_shape,len(w_shape),w_list
                if len(w_shape)==3:
                    w_l = []
                    for w_idx in range(w_shape[2]):
                        w_l.append(w_list[:,:,w_idx])
                    w_list = w_l
            except:
                w_list = None
    return w_list

def f_getTrafficDict(d_list,unit='m'):
    out = []
    for testIdx in range(len(d_list)):
        trafficDict = {}
        trafficList = d_list[testIdx]
        #reorder = range(len(trafficList))[::-1]
        #print reorder
        for hostIdx in range(len(trafficList)): # reorder:
            key = 'h'+str(hostIdx+1)
            trafficDict[key] = str(trafficList[hostIdx])+unit
        out.append(trafficDict)
        #out.append(collections.OrderedDict(sorted(trafficDict.items())))
    return out
def f_rescaleTrafficDict(tDictList,scale):
    out = []
    for testIdx in range(len(tDictList)):
        trafficDict = {}
        sDict = tDictList[testIdx]
        for hostKey in sDict.keys():
            #print hostKey,sDict[hostKey][0:-1]
            value = eval(sDict[hostKey][0:-1])
            unit  = sDict[hostKey][-1]
            scaled = scale * value
            trafficDict[hostKey] = str(scaled)+unit
            print hostKey,trafficDict[hostKey]
        out.append(trafficDict)
    return out
        

def f_getIncreList(dataSetStr):
    traffic_list = []
    try:
        traffic_s,traffic_e,traf_step = eval(dataSetStr)
        traffic_s = np.array(traffic_s)
        traffic_e = np.array(traffic_e)
        #print traffic_s,traffic_e,traf_step
        if len(traffic_s)!=len(traffic_e):
            print 'start traffic:%s mis-alignes with end traffic:%s' %(str(traffic_s),str(traffic_e))
            return None
        m_iter_num = int(np.ceil(min(traffic_e-traffic_s) / traf_step))
        #print m_iter_num
        for i in range(m_iter_num):
            newTraffic = traffic_s + float(i)*traf_step
            traffic_list.append(newTraffic)
    except:
        print 'invalid input = %s' %dataSetStr
    return traffic_list

def f_get2DIncreList(dataSetStr):
    traffic_list = []
    m_rowNum = 0
    m_colNum = 0
    traffic_s = []
    traf_step = 0
    try:
        traffic_s,traffic_e,traf_step = eval(dataSetStr)
        traffic_s = np.array(traffic_s)
        traffic_e = np.array(traffic_e)
        assert(len(traffic_s)==2) # for 2 input only
        assert(len(traffic_e)==2) # for 2 input only
        m_rowNum   = int(round((traffic_e[0]-traffic_s[0]) / traf_step)) # silly numpy, has to use round
        m_colNum   = int(round((traffic_e[1]-traffic_s[1]) / traf_step))
        #print traffic_e,traffic_s, m_rowNum,m_colNum
        for row in range(m_rowNum):
            for col in range(m_colNum):
                t_row = traffic_s[0]+row * traf_step
                t_col = traffic_s[1]+col * traf_step
                traffic_list.append(np.array([t_row,t_col]))
        assert(len(traffic_list)==(m_rowNum*m_colNum))                              
    except:
        print 'invalid input = %s' %dataSetStr
    return traffic_list,m_rowNum,m_colNum,traffic_s,traf_step

def f_parse_ping_out(pingStr,cdf_outen=False):
    singlePings = []
    ''' parse ping output of last 2 lines'''
    lines = pingStr.split('\n')
    #print lines[-1]," + ",lines[-2]," + ",lines[-3]
    if "packets transmitted" not in lines[-3] or "rtt min/avg/max/mdev" not in lines[-2]:
        raise Exception('ping out put incomplete')
    rrtStr=lines[-2].split('=')[-1].split(',')[0]#.split(' ')
    unit = rrtStr.split(' ')[-1]
    #print unit#print rrtStr
    # [-+]?\d*\.\d+|\d+ <-- extracting float/int number
    rrt_listRaw = [re.findall('\d*\.\d+|\d+', s) for s in rrtStr.split('/')]
    # flatten a list
    rrt_list = [item for sublist in rrt_listRaw for item in sublist]
    #print rrt_list
    state_listRaw = [re.findall('\d*\.\d+|\d+', s) for s in lines[-3].split(',')]    
    state_list = [item for sublist in state_listRaw for item in sublist]
    #print state_list
    if len(rrt_list)!=4 or len(state_list)!=4:
        raise Exception('incomplete packet line:%s and/or rrt line:%s' %(lines[-3],lines[-2]))
    if cdf_outen:
        singlePings = f_parse_ping_out_single(lines[0:-5])
    return(state_list+rrt_list+[unit]),singlePings
def f_parse_ping_out_single(lines):
    #print lines
    lineStart = f_getPingStartLine(lines)
    #print 'lineStart=',lineStart
    out = [float(line.split('=')[-1].split(' ')[0]) for line in lines[lineStart:]]
    #print 'out=',out
    return out
def f_getPingStartLine(lines):
    cnt=0
    #print 'len(lines)=',len(lines)
    while cnt < len(lines):
        #print 'lines[cnt]=',lines[cnt]
        if ('bytes from' not in lines[cnt] 
            or 'icmp_seq=' not in lines[cnt] 
            or 'time=' not in lines[cnt]):
            #print 'cnt=',cnt
            cnt+=1
        else:
            return cnt    
    #print 'cnt=',cnt
    raise Exception('can not find the first line')

def f_getScale(dataStr):
    trafficScale = 1.0
    objWeightScale = 1.0
    try:
        #opts = re.sub('[%s]' % ''.join(['[',']']), '', dataStr)
        opts = re.sub('[%s]' % ''.join([' ',';']), ',', dataStr)
        #print opts#,len(opts)
        if ('[' not in opts) and (']' not in opts):
            raw = eval('['+opts+']') # eval(opts)
        else:
            raw = eval(opts)
        #print raw#,len(raw)
        if len(raw)==1:
            trafficScale=raw[0]
        elif len(raw)==2:
            [trafficScale,objWeightScale] = raw
    except:
        print 'can not convert option: %s' %dataStr 
    return trafficScale,objWeightScale

def f_getPara(Para):
    para   = None
    try:
        para = eval(Para)
    except:
        raise ('can not evaluate input Para=%s' %DelayPara)
    return para

class MemoryEntry (object):
    """
    Record for flows we are balancing

    Table entries in the switch "remember" flows for a period of time, but
    rather than set their expirations to some long value (potentially leading
    to lots of rules for dead connections), we let them expire from the
    switch relatively quickly and remember them here in the controller for
    longer.

    Another tactic would be to increase the timeouts on the switch and use
    the Nicira extension witch can match packets with FIN set to remove them
    when the connection closes.
    """
    def __init__ (self, input, mem_time=FLOW_MEMORY_TIMEOUT):
        #self.server = server
        self.content = input
        #self.client_port = client_port
        self.mem_time = mem_time
        self.refresh()

    def refresh (self):
        self.timeout = time.time() + self.mem_time
        
    def adjust (self, adjust_time):
        self.timeout = self.timeout + adjust_time

    @property
    def is_expired (self):
        return time.time() > self.timeout
    
    @property
    def time_to_live (self):
        if not self.is_expired:
            return self.timeout - time.time()
        else:
            return -1

class MemoryDict (MemoryEntry):
    def __init__ (self, input={}, mem_time=FLOW_MEMORY_TIMEOUT):
        MemoryEntry.__init__ (self, input, mem_time)
        self._check_input_dict()
        
    def _check_input_dict (self):
        if isinstance(self.content,dict):
            self.dict_keys = self.content.keys()
            self.dict_values = self.content.values()
        else:
            print 'input is not dictionary'
            sys.exit()
            
    def add(self,new_dict):
        if isinstance(new_dict,dict):
            self.content.update(new_dict)
            self.dict_keys = self.content.keys()
            self.dict_values = self.content.values()
            
    def get(self,key):
        return self.content.get(key)
            
    @property        
    def keys(self):
        return self.dict_keys
    
    @property    
    def values(self): 
        return self.dict_values

class MemoryPacket (MemoryEntry):
    def __init__ (self, input, input_port, mem_time=FLOW_MEMORY_TIMEOUT):
        MemoryEntry.__init__ (self, input, mem_time)
        self.input_port = input_port
        
    @property    
    def hw_adds(self):
        ethp = self.input
        return ethp.src, ethp.dst
    @property
    def nw_adds(self):
        ethp = self.input
        ipp = ethp.find('ipv4')
        if ipp:
            return ipp.srcip,ipp.dstip
        else:
            return None, None
    @property
    def tcp_ports(self):
        ethp = self.input
        tcpp = ethp.find('tcp')
        if tcpp:
            return tcpp.dstport,tcpp.srcport
        else:
            return None, None
                   
def getDpid( name, dpidLen=DPIDLEN ):
    "Derive dpid from switch name, s1 -> 1"
    try:
        dpid = int( re.findall( r'\d+', name )[ -1 ] )
        dpid = hex( dpid )[ 2: ] # remove 0x        
        dpid = '0' * ( dpidLen - len( dpid ) ) + dpid
        return dpid
    except IndexError:
        raise Exception( 'Unable to derive default datapath ID - '
		     'please either specify a dpid or use a '
		     'canonical switch name such as s23.' )

WR_MAXLEN = 1000
class WeightedRandomPicker (object):
    '''
    Apply weighted random algorithm to choose vlan tagging
    |  10%  |  10%  |     20%     |            40%             |
    |<----->|<----->|<----------->|<-------------------------->|
    |1111111222222223333333333333344444444444444444444444444444| # accessed by random interleaver
    |<---------------------------------------------------------|
    |                        100%                              |
    
    see also Weighted random generation in Python at http://eli.thegreenplace.net/2010/01/22/weighted-random-generation-in-python
    '''
    def __init__ (self, vlanlist, vlanRatioIn=None, MaxLength=WR_MAXLEN):
        self.vlanlist  = vlanlist
        self._vlanRatio = vlanRatioIn
        self.m_vlanSeq = [] # vlan choice sequence with distribution on vlanRatioIn
        self.m_vlanNum = []
        self.m_shuffleSeq = []
        self.m_Cnt = 0
        self._wrMaxLen = MaxLength
        self._vlanChoice = None
        if vlanRatioIn:
            assert(len(vlanlist)==len(vlanRatioIn)), "vlanRatioIn: %r dim not match" % vlanRatioIn
        elif len(vlanlist)>0:
            self._vlanRatio = [1./len(vlanlist)] * len(vlanlist)
        else:
            raise RuntimeError("vlanlist is empty")
        self._generate_shuffleSeq()
        self._generate_wrSeq()
        
    def _generate_shuffleSeq(self):
        self.m_shuffleSeq = range(self._wrMaxLen)
        random.shuffle(self.m_shuffleSeq)
        
    def _generate_wrSeq(self):
        #self.m_vlanSeq = []
        # make sure ratio add up to 1
        sumRatio = sum(self._vlanRatio)
        self._vlanRatio = [x/sumRatio for x in self._vlanRatio]       
        self.m_vlanNum     = [int(x * self._wrMaxLen) for x in self._vlanRatio]
        # add missing roundup to the maximal ratio
        maxIdx = self.m_vlanNum.index(max(self.m_vlanNum))
        self.m_vlanNum[maxIdx] += self._wrMaxLen - sum(self.m_vlanNum)
        assert(sum(self.m_vlanNum)==self._wrMaxLen)
        
        m_vlanSeq = []
        for vidCnt in range(len(self.vlanlist)):
            vid = self.vlanlist[vidCnt]
            num = self.m_vlanNum[vidCnt]
            t_seq = [vid] * num
            #self.m_vlanSeq = self.m_vlanSeq + t_seq
            m_vlanSeq = m_vlanSeq + t_seq
        self.m_vlanSeq = m_vlanSeq
    @property
    def vlanChoice(self):
        """return random vlan choice with predefined distribution"""
        idx = self.m_shuffleSeq[self.m_Cnt]
        self._vlanChoice = self.m_vlanSeq[idx]
        self.m_Cnt = (self.m_Cnt + 1) % self._wrMaxLen
        return self._vlanChoice
    
    @property
    def vlanRatio(self):
        return self._vlanRatio
    @vlanRatio.setter
    def vlanRatio(self, value):
        self._vlanRatio = value
        assert(len(self.vlanlist)==len(value)), "InputVlanRatio= %r dim not match self.vlanlist= %r" % (value,self.vlanlist)
        self._generate_wrSeq()
        
    @property
    def wrMaxLen(self):
        return self._wrMaxLen
    @wrMaxLen.setter
    def wrMaxLen(self, value):
        self._wrMaxLen = value
        self._generate_shuffleSeq()
        self._generate_wrSeq()
        self.m_Cnt = self.m_Cnt % self._wrMaxLen 



if __name__ == "__main__":
    #print getDpid ("s1e45")
    #print getDpid ("s23")
    #print getDpid ("SGW6")
    a=MemoryDict({'a':1,'b':2},mem_time=0.01)
    print a.time_to_live
    time.sleep(1)
    print a.time_to_live
    a.adjust(1)
    print a.time_to_live, a.keys, a.values
    print "@@@@@@@@@@@@@@@@@@@@@@@@@"
    b = WeightedRandomPicker([1,2,4,5], vlanRatioIn=None, MaxLength=12)
    print b.vlanRatio, b.wrMaxLen
    print b.m_shuffleSeq
    print "@@@@@@@@@@@@@@@@@@@@@@@@@"
    b.vlanRatio=[0.6,0.0,0.3,0.6]
    print b.vlanRatio
    print b.m_vlanSeq
    for i in range(b.wrMaxLen+5):
        print i, b.vlanChoice, b.m_shuffleSeq[i%b.wrMaxLen]
    print "@@@@@@@@@@@@@@@@@@@@@@@@@"
    b.vlanRatio=[1.,1.,1.,1.]
    b.wrMaxLen = 4
    print b.vlanRatio
    print b.m_vlanSeq
    for i in range(b.wrMaxLen+5):
        print i, b.vlanChoice, b.m_shuffleSeq[i%b.wrMaxLen]
#     print "NNNNNNNNNNNNNNNNNNNNNNNNNNNNN"    
#     print 'TOPO_DIR=%s,current dir=%s' %(TOPO_DIR,os.path.dirname(os.path.realpath(__file__)))
#     print f_getTopoFile('nat.csv')    
    print "WWWWWWWWWWWWWWWWWWWWWWWWWWWWW"
    print f_getRawList('weightP2_2','Weight')   # ('[0.5,1.0]')
    print f_getRawList('[0.0,0.5,1.0]')
    traffic_raw_in = f_getRawList('[[0.5,0.9],[0.9,1.3]]')
    print f_getTrafficDict(traffic_raw_in)
    print "IIIIIIIIIIIIIIIIIIIIIIIIIIIII"
    print f_getTrafficDict(f_getIncreList('[0.1,0.5],[1.0,1.4],0.1'))
    print f_get2DIncreList('[0.1,0.1],[0.3,0.4],0.1')
    print f_getTrafficDict(f_get2DIncreList('[0.1,0.1],[0.3,0.4],0.1')[0])
    print "PPPPPPPPPPPPPPPPPPPPPPPPPPPPP"
#     pingStr=("PING 192.168.200.1 (192.168.200.1) 56(84) bytes of data.\n"
#             +"\n"
#             +"--- 192.168.200.1 ping statistics ---\n"
#             +"120 packets transmitted, 120 received, 0% packet loss, time 11445ms\n"
#             +"rtt min/avg/max/mdev = 21.333/28.374/37.125/3.603 ms, pipe 6\n")
    ping_file = open('pingUsnet11.log') #open('test_ping_out.txt', 'r')
    pingStr = ping_file.read()
    pingout,pinglist = f_parse_ping_out(pingStr,True)
    print pingout
#     data = {}
#     data['x'] = np.array([1,3,5,4,6])
#     scipy.io.savemat('testmat.mat',data)
    print 'ssssssssssssssssssssss'
    print f_getScale('[0.02 18.0]')
    print f_getScale('0.02,18.0')
    print f_getScale('[0.02]')
    print f_getScale('18.0')
    print 'dddddddddddddddddddddd'
    print f_getPara('[120,140,130,150]')
    print 'mat traffic in mmmmmmmmmmmmmmmmmmmmm'
    traffic_raw_in = f_getRawList('traffic_set')
    print 'traffic_raw_in=',traffic_raw_in
    traffic_in     = f_getTrafficDict(traffic_raw_in)
    print traffic_in
    
    
