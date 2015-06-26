#!/usr/bin/python

'''
Cloud project:
- read bwm-ng related outputs 
Tony Lin
'''

import os, sys, getopt
import numpy as np
import re
# import all constants
from utils.constant_read_topo import *
from utils.constant_sys import *
NONE_NP = None #np.array([None,None])

def f_read_bwm_csv(csv_fname,csvformat=BWM_CSVFORMAT):
    # Reading the csv file.
    csv_data = NONE_NP
    while csv_data == NONE_NP:
        try:
            csv_data = np.loadtxt(csv_fname, dtype=csvformat,
                                  comments='#',delimiter=';')
        except (IOError,IndexError) as e:
            print 'fail to load file: %s and return None for error:%s' %(csv_fname,e)
            csv_data = NONE_NP
    return csv_data

class BwmCsvParser(object):
    def __init__(self,csv_fname,num_sample_in=1):
        ''' init.        
        '''            
        self.csv_data      = f_read_bwm_csv(csv_fname)  # input raw data from np.loadtxt
        self.dt = np.dtype(BWM_CSVFORMAT)
        self.csv_fname = csv_fname
        self.f_init(num_sample_in)
        
    def f_init(self,num_sample_in):    
        self.f_initPara()
        self.f_parse_num()   # may call multiple times
        self.f_parse_name()  # call once
        self.f_classify(num_sample_in)    # default only 1 sample data frame
        self.f_ave_data()

    def f_initPara(self):
        self.pars_status= False
        self.outputAvailable = False    
        self.delimiterIdxs = None
        self.start_lineIdx = 0
        self.end_lineIdx   = 0
        self.start_intName = None
        self.end_intName   = None
        self.line_num      = 0         # number of lines in csv
        self.report_num    = 0         # number of repeats based on number of BWMCSV_DELIMITER='total'
        self.port_num      = 0         # number of physical ports
        self.port_name     = None
        self.switch_name   = None
        self.switch_port_dict = {}
        self.storage_depth = 0         # how many samples are saved
        
        self.mem = {}
        self.memDepth = 0
        self.o_portAve = None
        self.o_swAve   = None
        self.o_totalAve = None
        
    def f_parse_num(self):
        self.line_num   = len(self.csv_data[INT])
        self.delimiterIdxs = np.where(self.csv_data[INT] == BWMCSV_DELIMITER)[0]
        ''' parse num of repeats'''
        #self.report_num = self.csv_data[INT].tolist().count(BWMCSV_DELIMITER)
        self.report_num = len(self.delimiterIdxs)
#         print self.report_num
        if self.report_num == 0:
            self.pars_status = False
            #print 'parse fail for not finding delimiter:%s' %BWMCSV_DELIMITER
            return False
        elif (self.report_num == 1 and                     # only 1 'total' delimiter
              self.csv_data[INT][-1] != BWMCSV_DELIMITER): # last one is not 'total'
            self.pars_status = False
            return False       
        ''' parse num of ports exclude total'''
        extra = self.line_num % self.report_num
        if (extra == 0 and 
            self.csv_data[INT][-1] == BWMCSV_DELIMITER): # exact match with last one as 'total'
            self.port_num   = self.line_num / self.report_num - 1
            self.start_lineIdx = 0
            self.end_lineIdx   = self.line_num - 1 # line idx starts from 0 not 1
            self.pars_status = True
        elif (self.csv_data[INT][-1] == BWMCSV_DELIMITER): # not exact match but with last one as 'total'
            if not self.f_subtr_report_num(): return False
            self.port_num   = (self.line_num-extra) / self.report_num - 1
            self.start_lineIdx = extra
            self.end_lineIdx   = self.line_num - 1 # line idx starts from 0 not 1
            self.pars_status = True
        else:
            if not self.f_subtr_report_num(): return False
            lineIdxLastDelimiter = max(self.delimiterIdxs) # line idx starts from 0 not 1
            lineIdxFirstDelimiter = min(self.delimiterIdxs) # line idx starts from 0 not 1
            self.start_lineIdx = lineIdxFirstDelimiter+1   # +1 --> remove 'total'
            self.end_lineIdx   = lineIdxLastDelimiter
            trueLineNum = lineIdxLastDelimiter - lineIdxFirstDelimiter
            self.port_num   = trueLineNum / (len(self.delimiterIdxs)-1) - 1
            self.pars_status = True
        self.start_intName = self.csv_data[INT][self.start_lineIdx]
        #print self.csv_data[INT][self.start_lineIdx]
        return True
    def f_subtr_report_num(self):
        self.report_num = self.report_num - 1
        if self.report_num == 0:
            self.pars_status = False
            print 'incomplete report' 
            return False
        else: 
            return True

    def f_parse_name(self):
        self.port_name   = list(set(self.csv_data[INT].tolist())-set([BWMCSV_DELIMITER])) # remove 'total' item
        self.switch_name = list(set([pname.split('-eth')[0] for pname in self.port_name]))
        for pt in self.port_name:
            sw = pt.split('-eth')[0]
            if sw in self.switch_port_dict.keys():
                self.switch_port_dict[sw].append(pt)
            else:
                self.switch_port_dict[sw] = [pt]
                     

    def f_classify(self,num_sample_in=1,format=8):  
        assert (num_sample_in > 0)
        num_sample = min(num_sample_in,self.report_num)
        blockSize = self.port_num+1 # +1 means add 'total'
        real_sampleNum = 0
        for blockIdx in range(num_sample): 
            # backward reading
            sIdx = self.end_lineIdx - blockIdx*blockSize
            # skip 'total'==0 block
            if self.csv_data[sIdx][RA_BYTE] <= 0: continue 
            #print 'sIdx=%d, blockSize=%d' %(sIdx,blockSize)
            eIdx = sIdx - blockSize
            for lineIdx in range(sIdx-1,eIdx,-1): # skip 'total' item and backward with step -1
                self.f_update_line(lineIdx,format)
                
            real_sampleNum += 1
        self.report_num = real_sampleNum
        self.memDepth += real_sampleNum
        #print 'real_sampleNum=%d' %(real_sampleNum)
        return
    def f_update_line(self,lineIdx,format):
        intf_name = self.csv_data[lineIdx][INT]
        if intf_name in self.port_name:
            # do
            #print self.csv_data[lineIdx][INT]
            dataArray = self.f_extract_dataLine(lineIdx,format)
            if intf_name in self.mem.keys():
                #print 'intf_name=%s' %(intf_name)
                self.mem[intf_name] = np.concatenate([self.mem[intf_name], dataArray], axis=0)
            else:
                self.mem[intf_name] = dataArray                  
            return
    def f_extract_dataLine(self,lineIdx,format): # default is byte, *8 convert to bit, /PACKET_OVERHEAD to remove iperf overhead
        line_npArray = self.csv_data[lineIdx]
        dataArray = np.array([[line_npArray[idx]*format/PACKET_OVERHEAD for idx in BWM_CSVDATA_RBYTE]])            
        return dataArray
    
    def f_ave_data(self):
        self.o_portAve = {}
        if not self.mem:  # return None for empty mem
            self.outputAvailable = False
            return False
        for port in self.port_name:
            data = self.mem[port]
            self.o_portAve[port] = np.average(data, axis=0)
        '''calculate the average aggregated input traffic on each switch'''
        self.f_get_swAgg_traffic_ave()
        self.o_totalAve = dict(self.o_portAve.items() + self.o_swAve.items())
        self.outputAvailable = True
        return True
    def f_get_swAgg_traffic_ave(self):
        self.o_swAve   = {}
        inIdx  = BWM_CSVDATA_RBYTE.index(RI_BYTE)
        outIdx = BWM_CSVDATA_RBYTE.index(RO_BYTE)        
        for sw in self.switch_name:
            inAgg  = 0.0
            outAgg = 0.0
            for port in self.switch_port_dict[sw]:
                #print port, inIdx, self.o_portAve[port][inIdx]
                inAgg  += self.o_portAve[port][inIdx]
                outAgg += self.o_portAve[port][outIdx]
            self.o_swAve[sw] = [inAgg]    

    def f_readNew(self,readMode='w',num_sample_in=1):
        readSuccess = False
        while not readSuccess:
            try:
                '''read target file again'''
                self.csv_data      = f_read_bwm_csv(self.csv_fname)
                if readMode == 'w':
                    '''init data memory'''
                    self.mem = {}
                    self.memDepth = 0
                '''check data start and end'''
                if (self.csv_data[INT][0] != self.start_intName or
                    self.csv_data[INT][-1] != BWMCSV_DELIMITER):
                    ''' re-synch the input data'''
                    self.f_parse_num()
                self.f_classify(num_sample_in)
                self.f_ave_data()
                readSuccess = True
            except (IOError,IndexError) as e:
                print 'fail to read and process file: %s and return None for error:%s' %(self.csv_fname,e)
                fid = open(ERROR_LOG, 'a')
                fid.write('fail to read and process file: %s and return None for error:%s' %(self.csv_fname,e))
                fid.close()
                #np.savetxt(ERROR_CSV, s, delimiter=",")
    
    def f_getMapOut(self,intKeys,format=1):
        out = []
        for resKey in intKeys:
            out.append(self.o_totalAve[resKey][-1]*format) # -1 for total only
        return out        
                                
        
    
if __name__ == "__main__":    
#     csv_out = f_read_bwm_csv('netbw1')
#     print len(csv_out[INT])
#     print csv_out[INT]
#     bwmCsvObj = BwmCsvParser('netbw1null')
#     print bwmCsvObj.pars_status, bwmCsvObj.report_num, bwmCsvObj.port_num, bwmCsvObj.delimiterIdxs
#     bwmCsvObj = BwmCsvParser('netbw1')
#     print bwmCsvObj.pars_status, bwmCsvObj.report_num, bwmCsvObj.port_num, bwmCsvObj.delimiterIdxs
#     bwmCsvObj = BwmCsvParser('netbw1a')
#     print bwmCsvObj.pars_status, bwmCsvObj.report_num, bwmCsvObj.port_num, bwmCsvObj.delimiterIdxs
    bwmCsvObj = BwmCsvParser('j3',num_sample_in=3) #('netbw1c',num_sample_in=3) #
    print bwmCsvObj.pars_status, bwmCsvObj.memDepth,bwmCsvObj.report_num, bwmCsvObj.port_num, bwmCsvObj.start_intName,bwmCsvObj.delimiterIdxs
    #print bwmCsvObj.port_name
    #print bwmCsvObj.switch_name
    #print bwmCsvObj.mem
    print bwmCsvObj.o_portAve #['M2-eth4'][2]
    #print bwmCsvObj.switch_port_dict
    print bwmCsvObj.o_swAve
    print 'NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN'
    bwmCsvObj.f_readNew()
    print bwmCsvObj.pars_status, bwmCsvObj.memDepth,bwmCsvObj.report_num, bwmCsvObj.port_num, bwmCsvObj.start_intName,bwmCsvObj.delimiterIdxs
    print bwmCsvObj.o_portAve
    print bwmCsvObj.o_swAve
    print 'NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN'
    bwmCsvObj.f_readNew(readMode='a',num_sample_in=3)
    print bwmCsvObj.pars_status, bwmCsvObj.memDepth,bwmCsvObj.report_num, bwmCsvObj.port_num, bwmCsvObj.start_intName,bwmCsvObj.delimiterIdxs
    print bwmCsvObj.o_portAve
    print bwmCsvObj.o_swAve
    print bwmCsvObj.f_getMapOut(bwmCsvObj.port_name)
    print bwmCsvObj.switch_name
    print bwmCsvObj.f_getMapOut(bwmCsvObj.switch_name)
    
    
    
