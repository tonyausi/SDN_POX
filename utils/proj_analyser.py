import os,sys
import re
import time
import numpy as np
#import shutil # High-level file operations,used to delete files in a dir
import scipy.io

from utils.constant_sys import * 
from utils.proj_util import f_load_mat
import utils.paras as PARAS


class ResultsRecorder (object):
    def __init__(self,D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,LB_mode='user',i_fileName=TRAFFIC_OUT,normScale=None,
                 plot_en=False,srecord_en=False):
        '''input'''
        self.i_traffic_in=np.array(D)       # input traffic vector dim: J
        self.i_RES_cap   =np.array(C)       # resource capability dim: N=I+H+L
        self.i_RES_maxpow=np.array(PMAX)    #/1000000.0    # resource peak power dim: N   # PMAX./1000000
        if np.max(self.i_RES_maxpow) > 1000000.0:
            self.i_RES_maxpow = self.i_RES_maxpow / 1000000.0
        #print self.i_RES_maxpow
        self.i_RES_idleScale=np.array(Beta) # resource idle power scale coeff,p0=\beta * p_max dim: N
        self.i_SGW_num=int(I)          # number of SGW: I
        self.i_MUX_num=int(H)          # number of MUX: H
        self.i_link_num=int(L)         # number of link: L
        self.i_EnbRouteMat=np.array(EM)     # ENB-route matrix, 0-1 matrix dim: J * R
        self.i_ResRouteMat=np.array(RM)     # resource-route matrix, 0-1 matrix dim: N * R
        self.i_Weight=Weight 
        self.io_load_ratio_th = Threshold   # Load ratio threshold 
        self.io_iter_limit=Iteration        # iteration limit
        self.io_norm_mode=NormMode          # power norm mode: mean, max (per SGW) 
        self.i_sdiag = None         # Small value to be added on diagnal of sigular matrix  
        self.i_outloop_en=Outloop
        self.io_method_mode = GameMode      # [] for GR for gradient mode, BR for best response mode 
        self.i_RES_ratio = np.array(RES_ratio)
        self.i_constraint_mode = constraint_mode
        self.i_LB_mode = LB_mode
        #print self.i_LB_mode
        self.i_fileName = i_fileName
        #print self.i_fileName
        self.i_normScale = normScale
        
        self.i_fsimName = SIM_OUT
        self.i_control_mode = None
        self.i_fpingName= PING_OUT
        self.i_cntIdx = 0          
        self.m_plot_en=plot_en
        self.m_srecord_en=srecord_en # Enable single results recording  
              
        self.f_init_m()
        self.f_check()  
        self.f_init()
    def f_init_m(self):    
#         self.io_dif_mode='sgw' # check 'sgw' or 'res' status to trigger external loop        
        self.io_scale_mode=None    # special case for w=0 with res of small pow/cap difference
           # dynamic input
        self.m_TrafficDataAvg  = None
        self.m_TrafficBuffer   = []
        self.m_pingData =None
        self.m_pingDataSingleDict = {}
           # class member
        self.m_WeightPow=None     # weight vector for power, dim: 1xJ
        self.m_WeightSgw=None     # Weight for SGW
        self.m_WeightMux=None     # Weight for MUX
#         self.m_throughput=None    # system throughput        
        self.m_ResNum=None        # N for RES number
        self.m_SgwNum=None        # I for S-GW number
        self.m_MuxNum=None        # H for MUX number
        self.m_LinkNum=None       # L for links number
        self.m_EnbNum=None        # J for BS number
        self.m_RtNum=None         # R for route number
        self.m_ResRouteMat=None   # A matrix for RES-route map, DIM: N x R
        self.m_SgwRouteMat=None   # A(I) matrix for SGW-route map, DIM: I x R
        self.m_MuxRouteMat=None   # A(M) matrix for SGW-route map, DIM: M x R
        self.m_EnbRouteMat=None   # B matrix for active route, DIM: J x R
        self.m_EnbResMat=None     # B^sgw matrix for active RES for EnB, DIM: J x N
        self.m_EnbSgwMat=None     # B^sgw matrix for active SGW for EnB, DIM: J x I
        self.m_EnbMuxMat=None     # B^mux matrix for active MUX for EnB, DIM: J x M
        self.m_sgw_start=None     # SGW start postion
        self.m_mux_start=None     # MUX start postion
        self.m_link_start=None    # Link start postion 
#         self.m_RES_peff=None      # RES power efficiency dim: N
        self.m_res_appox_en=None  # RES power efficiency approximation mode enable: [0,1]
        self.m_RES_peff_approx=None # RES power efficiency approximation by utilisation ratio, dim: N
#         self.m_RES_p0=None        # RES idle power cosumption dim: N        
#         self.m_SGW_cap=None       # SGW capability dim: I  
        self.m_userSgwSysCap=None # user specific/allocated SGWs' capacity sum, dim: J
#         self.m_SGW_peff=None      # SGW power efficiency dim: I
#         self.m_SGW_p0=None        # SGW idle power cosumption dim: I
        self.m_MUX_cap=None       # MUX capability dim: H
#         self.m_MUX_peff=None      # MUX power efficiency dim: H
#         self.m_MUX_p0=None        # MUX idle power cosumption dim: H
#         self.m_Link_cap=None      # Link constrain dime: L
#         self.m_Link_peff=None     # Link power efficiency dim: L
#         self.m_Link_p0=None       # Link idle power cosumption dim: L
#         self.m_pmax=None            # dynamicly updated max power sum for active RES
        self.m_Evec=None            # dynamicly normalised slop vector Dim: N
        self.m_fvec=None          # f vector for first order in solver
        self.m_slop_norm   = None    
#         self.m_RES_cap=None       # dynamic RES capacity for active RES Dim: N
#         self.m_sdiag=None         # small value added to diagonal
#         self.m_rate_update=None   # rate allocation aggregation update for each route dim: 1xR        
#         self.m_Pratio=None        # Power utilisation ratio for normalisation
        self.m_ideal_lb_sgw=None  # ideal LB ratio for sgw
        self.m_sgw_user_load=None # user specific SGW load for user load balance mode, dim: 1xJ
        self.m_sgw_user_raio=None # user specific load ratio for user load balance mode, dim: 1xJ
#         self.m_ideal_lb_mux=None  # ideal LB ratio for mux     
#         self.m_iter_limit=None    # iteration limit
        self.m_iteration=0        # number of iteration ??????????????????????????????????
        self.m_lbMux_en=None      # load balancing MUX enable
#         self.m_varNorm_en=None    # load variance normalisation
        self.m_normScale = 1  # constant for level normalisation factors for POW,LB and delay (0.08,0.05,0.02
#           # penalty related
#         self.m_Bsquare_pen=None   # \lambda.*B^rt'*B^rt for eq constraint penalty
#         self.m_fvec_eq_pen=None   # -2\lambda.*d^in*B^rt' for eq constraint penalty
#         self.m_const_pen=0  # \lambda.*d^in*d^in' for eq constraint penalty
#         self.m_rescap_pen=None    # RES capacity penalty
#           # dynamic updats
#         self.md_sgw_config_pre=None  # vector of pre SGW configuration 1-->en, 0-->disable
#         self.md_sgw_config_new=None  # vector of new SGW configuration 1-->en, 0-->disable
#         self.md_dif_sgw=0      # flag for in/out active sgw change 1-->change, 0-->no change
        self.md_new_SgwNum=None      # new SGW dim for outter loop        
#         self.md_mux_config_pre=None  # vector of pre MUX configuration 1-->en, 0-->disable
        self.md_mux_config_new=None  # vector of new MUX configuration 1-->en, 0-->disable
#         self.md_dif_mux=0      # flag for in/out active mux change 1-->change, 0-->no change
        self.md_new_MuxNum=None      # new MUX dim for outter loop
#         self.md_max_pow=None         # dynamicly updated max power based on RES reconfig for max power normalisation
        self.md_pnorm=None           # RES normalised power consumption     
        self.md_new_ResNum=None      # new resource dim for outter loop 
#         self.md_res_config_prev=None  # vector of pre SGW configuration 1-->en, 0-->disable
#         self.md_res_config_new=None     # enabling map for resource,dim:1xN
#         self.md_dif_res=0      # flag for in/out active RES change 1-->change, 0-->no change        
#         self.md_dif=0          # flag to trigger external loop, 1-->change, 0-->no change
        self.md_C_sys_sgw=None       # dynamic SGW capacity sum
#         self.md_C_sys_mux=None       # dynamic MUX capacity sum
        self.md_var_norm=None        # dynamic LB variance norm factor
        self.md_MUX_ntime=None       # MUX normalised time delay,dim: 1xM
        self.md_mux_norm=None        # norm weighting factor 1/(M*S_MUX)
#         self.md_Qtilda=None          # w/I * A(I)^T Q^2 A(I) for variance matrix
#         self.md_QtildaSgw=None
#         self.md_QtildaMux=None
#           # constant
#         self.mc_max_iteration=8000 # maximal iteration number  
# #         self.mc_reOnOff_th=1.0e-10  # RES on/off threshold for traffic
        self.mc_reOnOff_th= 1.0e-4  # RES on/off threshold for traffic   
#         self.mc_minRate   = 1.0e-10      
          # plot related can be disabled without any fig     
          # file operation
        self.m_firstopen=1   # bull para for first time open file
        self.m_firstopen_sim=1
        self.m_firstopen_ping=1
        self.m_title_on=0
        self.m_title_on_sim=0
        self.m_title_on_ping=0
        self.m_fig_dir=None       # fig saving dir
        self.m_logPing_en = False
        self.m_trueTrafficDict = {}
          # debug
        self.m_debug=None
        # output
        self.o_fval=None         # output optimal value
#         self.o_Amatrix=None       # output allocation ratio matrix J * R
#         self.o_ResLoad=None          # load for each RES dim: N
        self.o_ResLoadRatio=None     # load ratio (load/capability) for each RES dim: N  
        self.o_RES_status=None    # RES status 1/0 --> on/off dim: N  
#         self.o_SgwLoad=None          # load for each SGW dim: I
#         self.o_SgwLoadRatio=None     # load ratio (load/capability) for each SGW dim: I
        self.o_SgwLR_std=None     # STD of SGW load ratio
#         self.o_SgwLBVarEnb=None   # SGW LB variance per ENB dim: 1xJ
        self.o_SgwLBVarEnbActual=None # actural LB variance per ENB dim: 1xJ
        self.o_SgwLBVarSum=None   # weighted sum of SGW LB variance over ENB
#         self.o_SgwLBVarSumActual=None   # Actual weighted sum of SGW LB variance over ENB
#         self.o_SGW_status=None    # SGW status 1/0 --> on/off dim: I   
        self.o_MuxLoad=None          # load for each MUX dim: M
        self.o_MuxLoadRatio=None     # load ratio (load/capability) for each MUX dim: I
        self.o_MuxLR_std=None     # STD of MUX load ratio 
        self.o_MUX_status=None    # SGW status 1/0 --> on/off dim: M
        self.o_MuxDelay=None      # delay for each MUX
        self.o_EnbDelay=None      # delay for each ENB (sum all MUX for each ENB),dim: J
        self.o_DelaySum=None      # weighted delay sum over all ENBs
        self.o_PowConsum1=None    # final incremental power consumption without idle power
        self.o_PowConsum2=None    # final power consumption with idle power
        self.o_powRes_enb=None    # appoximated power sum per ENB, dim: 1xJ
        self.init_2D_m()
    def init_2D_m(self):
        if (IN_TRAFFIC_ROW in PARAS.ACTION_PARAS.keys() 
            and IN_TRAFFIC_COL in PARAS.ACTION_PARAS.keys()):
            self.m_2D_en = True
            self.m_fpowName = POW2D_OUT
            self.m_tPowfirstopen = 1
            self.m_rowNum = PARAS.ACTION_PARAS[IN_TRAFFIC_ROW]
            self.m_colNum = PARAS.ACTION_PARAS[IN_TRAFFIC_COL]
            self.m_traffic_in_s = PARAS.ACTION_PARAS[IN_TRAFFIC_SLIST]
            self.m_traf_step    = PARAS.ACTION_PARAS[IN_TRAFFIC_STEP]
            self.m_boundary = self.m_rowNum*self.m_colNum-1
            self.m_rowCnt = 0
            self.m_colCnt = 0
            self.m_totalPowMat = np.zeros((self.m_rowNum,self.m_colNum))
        else:
            self.m_2D_en = False
            self.m_rowNum = 0
            self.m_colNum = 0
                    
    def f_check(self):
        self.m_SgwNum = self.i_SGW_num
        self.m_MuxNum = self.i_MUX_num
        self.m_LinkNum= self.i_link_num                        
        self.m_ResNum = self.m_SgwNum + self.m_MuxNum + self.m_LinkNum
        self.md_new_ResNum = self.m_ResNum
        self.md_new_SgwNum = self.m_SgwNum
        self.md_new_MuxNum = self.m_MuxNum
        # check RES paras
        if (len(self.i_RES_cap)!= self.m_ResNum 
            or len(self.i_RES_maxpow)!= self.m_ResNum
            or len(self.i_RES_idleScale)!= self.m_ResNum):
            #print self.i_RES_cap, len(self.i_RES_maxpow), len(self.i_RES_idleScale)
            raise Exception('Input i_RES_cap or i_RES_maxpow or i_RES_idleScale dim inconsistent')
        #check enb-route map and res-route map
        [self.m_EnbNum, self.m_RtNum]=np.shape(self.i_EnbRouteMat)
        t=np.shape(self.i_ResRouteMat)
        if t[0]!=self.m_ResNum:
            raise Exception('Number of resource mismatch for resource-route map')
        if t[1]!=self.m_RtNum:
            raise Exception('Number of routes mismatch for resource-route map')
        # check weighting factor
        w=np.shape(self.i_Weight)
        if not w:  # for single value
            self.i_Weight = np.array([[self.i_Weight]])
            w=np.shape(self.i_Weight)
        elif len(w)==1: # for one dim array
            self.i_Weight = np.array([self.i_Weight])
            w=np.shape(self.i_Weight)
        #print self.i_Weight
        if (w[0]==1) and (w[1]==1):
            if self.i_Weight>1 or self.i_Weight<0:
                raise Exception('Incorrect range for weighting factor')
        elif (w[0]!=self.m_EnbNum) or (w[1]!=3):
            raise Exception('Input i_Weight dim mismatch number of user')
            
        # check loop control
        if not (self.i_outloop_en==0 or self.i_outloop_en==1):
            print 'self.i_outloop_en=',self.i_outloop_en
            raise Exception('Out loop control should be 1 or 0')
        # check RES util ratio para
        if not(self.i_RES_ratio.any()):
            self.m_res_appox_en=0
        else:
            self.m_res_appox_en=1
            if len(self.i_RES_ratio)!= len(self.i_RES_idleScale):
                raise Exception('Input i_RES_ratio dim mismatch number of res')
        # check constraint mode
        if not(self.i_constraint_mode):
            self.i_constraint_mode = 'lag' # Lagrangian
        elif (self.i_constraint_mode!='lag'
              and self.i_constraint_mode!='pen'):
            raise Exception('Input i_constraint_mode not supported yet')
        #print'constraint mode is %s ' %self.i_constraint_mode
        # parse route map for RES and BS
        self.f_pars_routeMap()
        # check sgw, mux and links against system throughput
        self.m_throughput = sum(self.i_traffic_in)
        if sum(self.m_SGW_cap) <= self.m_throughput:
            raise Exception('SGW total capacity is less than the system input')
        if sum(self.m_MUX_cap) <= self.m_throughput:
            raise Exception('MUX total capacity is less than the system input')
        if sum(self.m_Link_cap) <= self.m_throughput:
            raise Exception('Link total capacity is less than the system input')
        # check i_LB_mode
        if not(self.i_LB_mode):
            self.i_LB_mode = 'global'
        elif (self.i_LB_mode!='global'
              and self.i_LB_mode!='user'):
            raise Exception('Only support global or user mode for i_LB_mode')
        # check i_normScale
        if (self.i_normScale):
            if self.i_normScale <= 0:
                raise Exception('i_normScale should be a positive numer')
            self.m_normScale = self.i_normScale
            # need put more check
    def f_pars_routeMap(self):
        # Derive RES paras
        self.m_RES_peff  = ((1.0-self.i_RES_idleScale)
                                   * self.i_RES_maxpow
                                   / self.i_RES_cap)
        if self.m_res_appox_en:
            idle_amortise = (self.m_res_appox_en * self.i_RES_idleScale * self.i_RES_maxpow
                            /(self.i_RES_ratio * self.i_RES_cap)) 
            self.m_RES_peff_approx = self.m_RES_peff + idle_amortise
        else:
            self.m_RES_peff_approx = self.m_RES_peff   
        self.m_RES_p0    = self.i_RES_idleScale * self.i_RES_maxpow                   
        # get start postion in res-route map
        self.m_sgw_start = 0 # 0 for python, 1 for matlab
        self.m_mux_start = self.m_SgwNum+self.m_sgw_start
        self.m_link_start = self.m_mux_start+self.m_MuxNum
        # Derive SGW paras
        sgw_start = self.m_sgw_start
        sgw_end   = self.m_mux_start # -1 only for matlab
        self.m_SGW_cap   = self.i_RES_cap[sgw_start:sgw_end]
        self.m_SGW_peff  = self.m_RES_peff_approx[sgw_start:sgw_end]
        self.m_SGW_p0    = self.m_RES_p0[sgw_start:sgw_end]
        # Derive MUX paras
        if self.i_MUX_num > 0:
            mux_start = self.m_mux_start
            mux_end   = self.m_link_start # -1 only for matlab
            self.m_MUX_cap   = self.i_RES_cap[mux_start:mux_end]
            self.m_MUX_peff  = self.m_RES_peff_approx[mux_start:mux_end]
            self.m_MUX_p0    = self.m_RES_p0[mux_start:mux_end]   
        # Derive link paras
        if self.i_link_num > 0:
            link_start = self.m_link_start
            link_end   = self.m_ResNum
            self.m_Link_cap   = self.i_RES_cap[link_start:link_end]
            self.m_Link_peff  = self.m_RES_peff_approx[link_start:link_end]
            self.m_Link_p0    = self.m_RES_p0[link_start:link_end]   
    # Initialise
    def f_init(self):
        # initial rate,res,sgw,mux set config
        self.f_init_set()
        # initial weights for power, SGW and MUX LB
        self.f_init_weight()
        # initial method mode related (LBMUX, LBMUX_NVAR)
        self.f_init_methodMode()  
        # initial power norm term
        self.f_init_norm()
        # Init user specific SGW cap sum
        self.f_init_userSysCap()                                     
        # Init output
        self.f_init_out()  
        # not useful yet
        self.f_form_fmin()     
    def f_init_set(self):   
        self.md_res_config_new = np.ones((self.m_ResNum,)) # (1,self.m_ResNum)
        # Set new SGW reconfig vector with all 1
        self.md_sgw_config_new = np.ones((self.m_SgwNum,)) # (1,self.m_SgwNum)
        # Set new MUX reconfig vector with all 1
        self.md_mux_config_new = np.ones((self.m_MuxNum,)) # (1,self.m_MuxNum)
        # Initial SGW load 
        self.m_load_update = np.zeros((self.m_ResNum,))    # (1,self.m_ResNum)
        if not(self.i_sdiag):
            self.m_sdiag = 0.001 / max(1,sum(self.i_traffic_in))
        else:
            self.m_sdiag = self.i_sdiag    
    def f_init_weight(self):
        w=np.shape(self.i_Weight)
        t1 = np.ones(self.m_EnbNum,)
        if (w[0]==1) and (w[1]==1):
            self.m_WeightPow = (1 - self.i_Weight[0,0]) * t1
            #print self.m_WeightPow
            self.m_WeightSgw = self.i_Weight[0,0] / 1 * t1
            self.m_WeightMux = self.i_Weight[0,0] / 1 * t1
        elif (w(1)==self.m_EnbNum) and (w(2)==3):
            self.m_WeightPow = self.i_Weight[:,0]
            self.m_WeightSgw = self.i_Weight[:,1]
            self.m_WeightMux = self.i_Weight[:,2]       
    def f_init_methodMode(self): ########################################## to be recalled by outer loop
        # init MUX LB enable
        if self.io_method_mode == 'LBMUX':
            self.m_lbMux_en = 1
        # init both MUX LB enable and LB variance norm
        if self.io_method_mode=='LBMUX_NVAR':
            self.m_lbMux_en = 1
            self.m_varNorm_en = 1       
    def f_init_norm(self):
        # form active RES-route map A based on md_res_config_new
        self.f_form_active_map()
        # get active RES cap
        self.m_RES_cap = self.md_res_config_new * self.i_RES_cap # 1xN
        # Get norm factor for power
        self.f_init_pow_norm()
        # Get updated ideal load ratio for SGW and MUX
        self.f_init_lb_norm()
        # Get norm factor for var
        self.f_init_delay_norm()
    def f_form_active_map(self):            
        # form active RES-route map A based on md_res_config_new
        self.m_ResRouteMat = self.i_ResRouteMat * self.md_res_config_new.flatten()[:,None]#.T --> .transpose() # NxR b* Nx1-->NxR (python index)
        # get active route from active SGW-route mat A(I),DIM: I x R
        self.m_SgwRouteMat = self.m_ResRouteMat[self.m_sgw_start:self.m_mux_start,:] # IxR, no need -1 as in matlab
        # get active route from active MUX-route mat A(M),DIM: M x R
        self.m_MuxRouteMat = self.m_ResRouteMat[self.m_mux_start:self.m_link_start,:] # MxR, no need -1 as in matlab           
        ########%%%%%%%%%%%%%%%%%%%%%%%%%%%%                        
        # form equality constrain: Bd=d^{in}, F with DIM: J x R
        ActiveSgwRtVect = (np.sum(self.m_SgwRouteMat,0)>0).astype(float)            # sum over SGW, 1xR
            # get active route path from active link & MUX
        MuxLinkRouteMat = self.m_ResRouteMat[self.m_mux_start:self.m_ResNum,:] #(ML)xR
        ActiveMuxLinkRtVect = (np.sum(MuxLinkRouteMat,0)>0).astype(float)           # sum over MUX_link, 1xR
            # get upated active route: Dim: 1 x R
        ActiveRtVect = ActiveSgwRtVect * ActiveMuxLinkRtVect                   # 1xR
            # get B(t) as Aeq, DIM: JxR
        self.m_EnbRouteMat = self.i_EnbRouteMat * ActiveRtVect # JxR b* 1xR -->JxR (python index)                        
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            # form Enb-Res mapping matrix, DIM: J x N
        self.m_EnbResMat = (np.dot(self.m_EnbRouteMat, self.m_ResRouteMat.T)>0).astype(float) #JxR * (NxR)^T=JxN
            # form Enb-Sgw mapping matrix, DIM: J x I
        self.m_EnbSgwMat = (np.dot(self.m_EnbRouteMat, self.m_SgwRouteMat.T)>0).astype(float) #JxR * (IxR)^T=JxI
            # form Enb-Mux mapping matrix, DIM: J x M
        self.m_EnbMuxMat = (np.dot(self.m_EnbRouteMat, self.m_MuxRouteMat.T)>0).astype(float)  #JxR * (MxR)^T=JxM                
    def f_init_pow_norm(self):   
        if not(self.io_norm_mode):
            self.io_norm_mode = 'mean'
        # get power normalisation term
        #self.md_max_pow     = self.i_RES_maxpow .* self.md_res_config_new
        if self.m_res_appox_en:
            self.md_max_pow = (self.m_RES_peff_approx * self.md_res_config_new * 
                                 self.i_RES_cap * self.i_RES_ratio)
        else:
            self.md_max_pow = self.i_RES_maxpow * self.md_res_config_new
        if self.io_norm_mode=='mean':
            self.m_Pratio = 1.0 #       0.8  #       
#                  self.md_pnorm = sum(self.md_max_pow) / sum(self.md_res_config_new) * self.m_Pratio
            pow_scale = 1 #sum(self.md_sgw_config_new) / sum(self.md_res_config_new) # ???? not clear scale by sum(self.md_sgw_config_new)
#                 if self.m_res_appox_en  
#                     pow_scale = self.i_RES_ratio
            #print self.m_WeightPow
            if min(self.m_WeightPow.flatten()) < (1 - 0.001):                    
                self.md_pnorm = sum(self.md_max_pow * pow_scale) 
            else:
                if self.io_scale_mode ==1:
                    p_weight = 0.1 * sum(self.i_traffic_in) / self.m_EnbNum
#                     p_weight = p_weight + 0.02
                else:                        
                    p_weight = 1.0
                    #print p_weight 
                self.md_pnorm = p_weight*sum(self.md_max_pow)*pow_scale 
        elif self.io_norm_mode=='max':
            self.md_pnorm = self.i_RES_maxpow
        else:
            raise Exception('Incorrect SGW normalisation mode')  
        # scale norm weights to proper level
        self.md_pnorm = self.md_pnorm * self.m_normScale                              
    def f_init_lb_norm(self):            
        m_new_SGW_cap = self.m_RES_cap[self.m_sgw_start : self.m_mux_start]
        self.md_C_sys_sgw = sum(m_new_SGW_cap)
        self.m_ideal_lb_sgw = self.m_throughput / self.md_C_sys_sgw #m_throughput as T calculated in f_check
        if self.m_varNorm_en == 0:
            return
#             self.md_var_norm = ((1.0 - self.m_ideal_lb_sgw).^2 
#                        + (self.md_new_SgwNum-1)*self.m_ideal_lb_sgw.^2) 
#                        ./ self.md_new_SgwNum
        sgw_weight = np.mean(np.sum(self.m_EnbSgwMat,1))
#             sgw_weight = sgw_weight .* 30              # debug purpose !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        var_weight = sgw_weight / self.md_new_SgwNum
        self.md_var_norm = var_weight * self.m_ideal_lb_sgw**2 
        self.md_var_norm = self.md_var_norm * self.m_normScale 
        #print 'True var normaliser = %2.5f ' %self.md_var_norm            
    def f_init_delay_norm(self):
        if self.m_lbMux_en == 1:
            m_new_MUX_cap = self.m_RES_cap[self.m_mux_start : self.m_link_start]
            self.md_C_sys_mux = sum(m_new_MUX_cap)
#                 self.m_ideal_lb_mux = self.m_throughput / self.md_C_sys_mux
            if self.m_res_appox_en:
                mux_start = self.m_mux_start
                mux_end   = self.m_link_start
                mux_ratio = self.i_RES_ratio[mux_start:mux_end]
                self.md_MUX_ntime = 1.0 / (self.m_MUX_cap * (1.0 - mux_ratio)) # 1xM
                #self.md_MUX_ntime = self.md_MUX_ntime * 2
            else:
                self.md_MUX_ntime = 1.0 / (self.md_C_sys_mux - self.m_throughput)
            # Get number of average routes over all users
            #routeNumEnb_weight = mean(sum(self.i_EnbRouteMat,1))
            routeNumEnb_weight = max(np.sum(self.i_EnbRouteMat,1))
            #print routeNumEnb_weight
            # converge for 0.01 and BsAggC=1
            self.md_MUX_ntime = self.md_MUX_ntime * routeNumEnb_weight # 10  ###!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            self.md_MUX_ntime = self.md_MUX_ntime *  self.m_normScale 
#           print self.md_MUX_ntime       
    def f_init_userSysCap(self):
        if self.i_LB_mode=='user':
            self.m_userSgwSysCap = np.dot(self.m_SGW_cap, self.m_EnbSgwMat.transpose()) #1xI * (JxI)'-->1xJ                      
    def f_init_out(self):
        self.o_traffic = np.zeros((self.m_EnbNum, self.m_RtNum))
        self.o_Amatrix = np.zeros((self.m_EnbNum, self.m_RtNum))  
        self.o_SgwLoadRatio = np.zeros((1,self.m_SgwNum))
        self.o_RES_status   = np.ones((1,self.m_ResNum))
        self.o_powRes_enb   = np.zeros(self.m_EnbNum,)

    def f_form_fmin(self):
        # form system normalised incremental power
        self.f_form_fmin_syspow()
        # form SGW load variance term
        self.f_form_fmin_lb_sgw()            
    def f_form_fmin_syspow(self):
        self.m_slop_norm = self.m_RES_peff_approx / self.md_pnorm #1xN
        self.m_fvec = np.zeros((1,self.m_RtNum))
        for enb_idx in range(self.m_EnbNum):
            # get weight for power for user j
            power_w = self.m_WeightPow[enb_idx]         
            # form eff vector
            self.m_Evec = power_w * self.m_slop_norm # 1xN
            # form vector as f for first order coefficient in slover
            # get EAB_j=E*A*B_j
            Bj_diag = np.diag(self.m_EnbRouteMat[enb_idx,:]) # RxR
            # 1xN * NxR * RxR --> 1xR  %%%%%%%%%%need to confirm to use i_ResRouteMat or m_ResRouteMat
            EAB_j = np.dot(np.dot(self.m_Evec, self.i_ResRouteMat), Bj_diag) #
            self.m_fvec = self.m_fvec+EAB_j #1xR   
    def f_form_fmin_lb_sgw(self):
        # for 'global' mode get same Qsrv_mat for all uers since C_sys
        # are the same
        if (self.i_LB_mode=='global'): 
            Qsrv_mat = self.f_form_QsrvMat(self.md_C_sys_sgw)
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # inite self.md_QtildaSgw
        self.md_QtildaSgw = np.zeros((self.m_RtNum, self.m_RtNum)) # RxR
        for enb_idx in range(self.m_EnbNum):
            if (self.i_LB_mode=='global'): 
                # diag(self.m_EnbSgwMat(enb_idx,:)) * Qsrv_mat # B*Q IxI
                Q_enb_mat = np.dot(np.diag(self.m_EnbSgwMat[enb_idx,:]), Qsrv_mat) # IxI
            elif (self.i_LB_mode=='user'):
                # use user specific C_sys,j to calculate Q^srv,j matrix
                Qsrv_mat = self.f_form_QsrvMat(self.m_userSgwSysCap[enb_idx]) # IxI
                B_diag_j = np.diag(self.m_EnbSgwMat[enb_idx,:])               # IxI
                Q_enb_mat = np.dot(np.dot(B_diag_j, Qsrv_mat), B_diag_j) # B*Q*B,IxI
            Q_enb_sq  = np.dot(Q_enb_mat.T, Q_enb_mat) # (IxI)^T * IxI = IxI
            # get A(I)^T B_j Q^2 A(I)
            QtildaSgwEnb = np.dot(np.dot(self.m_SgwRouteMat.T, Q_enb_sq), self.m_SgwRouteMat) # RxR
            # multiplied by weight_sgw
            QtildaSgwEnb = QtildaSgwEnb * self.m_WeightSgw[enb_idx]
            self.md_QtildaSgw = self.md_QtildaSgw + QtildaSgwEnb

        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
        # vaiance weight: 1/(I*S^hat)
        var_w = 1 / (self.md_new_SgwNum * self.md_var_norm)
        self.md_QtildaSgw = var_w * self.md_QtildaSgw  # RxR  
        self.md_Qtilda    = 2*self.md_QtildaSgw    
    def f_form_QsrvMat(self,C_sys_in):
        # get active system capacity based on user/global mode
        C_sys  = C_sys_in               # C_sys
#             Q_mat  = -(1.0/C_sys)*ones(self.md_new_SgwNum,self.md_new_SgwNum)  # -1/C_sys
        Q_mat  = -(1.0/C_sys) * np.ones((self.i_SGW_num,self.i_SGW_num))  # -1/C_sys, No need for dynamic SGW number
        t_diag = np.diag(1 / self.m_SGW_cap + self.m_sdiag)                    # diag(1/c_i+\tau)
        Q_mat  = Q_mat  + t_diag 
        return Q_mat 
                    
    def f_update_results(self):
        self.f_update_general_results()
        self.f_update_sgw_results()
        self.f_update_mux_results()
        '''
        
        '''
    def f_update_general_results(self): 
#         self.o_traffic = self.m_EnbRouteMat * self.m_rate_update.flatten() # JxR b* Rx1 (python index) --> JxR 
#         # bsxfun(@times,self.o_traffic,self.i_traffic_in')
#         self.o_Amatrix = self.o_traffic / self.i_traffic_in.flatten()[:,None] # JxR b/ Jx1 (python index) --> JxR                                   
        # get RES load,load ratio and status
        self.o_ResLoad = self.m_TrafficDataAvg     
        #self.o_RES_status = self.m_load_update > self.mc_reOnOff_th 
        self.o_ResLoadRatio = self.o_ResLoad / self.i_RES_cap
        self.o_RES_status = (self.o_ResLoadRatio > self.mc_reOnOff_th).astype(float)
        # calculate RES incremental power
        self.o_PowConsum1 = sum(self.m_RES_peff * (self.o_ResLoad))
        # calculate RES p0
        ipow_sum = sum(self.o_RES_status * (self.m_RES_p0))
        self.o_PowConsum2 = ipow_sum+self.o_PowConsum1          
    def f_update_sgw_results(self):                       
        # get SGW load,load ratio and status
        self.o_SgwLoad = self.o_ResLoad[self.m_sgw_start:self.m_mux_start] #-1)  
        #print self.o_SgwLoad          
        #self.o_SGW_status = self.o_SgwLoad > self.mc_reOnOff_th 
        self.o_SgwLoadRatio = self.o_ResLoadRatio[self.m_sgw_start:self.m_mux_start] #-1)
        self.o_SGW_status = (self.o_SgwLoadRatio > self.mc_reOnOff_th).astype(int) 
        SgwLR_var           = sum((self.o_SgwLoadRatio - self.m_ideal_lb_sgw)**2 / self.md_new_SgwNum)
        self.o_SgwLR_std    = np.sqrt(SgwLR_var)
        # get SGW LB output
        self.f_update_lb_sgw(self.m_sdiag)
            #disp(self.m_sdiag)
            #self.f_update_lb_sgw(0.00001)
            # get actual SGW LB for enbs
        self.f_calculate_lb_actual()
    def f_update_lb_sgw(self,sdiag): # similar to f_form_fmin_lb_sgw            
        # get active system capacity
        C_sys  = self.md_C_sys_sgw               # C_sys
        Q_mat  = -(1.0/C_sys)*np.ones((self.i_SGW_num,self.i_SGW_num))  # -1/C_sys, No need for dynamic SGW number (IxI)
        t_diag = np.diag(1./self.m_SGW_cap + sdiag)                    # diag(1/c_i+\tau)
        Q_mat  = Q_mat  + t_diag                                      # IxI, C_i -1/S_sys at diag  
        # vaiance weight: 1/(I*S^hat)
        var_w = 1 / (self.md_new_SgwNum * self.md_var_norm / self.m_normScale)
        # initial output per ENB
        self.o_SgwLBVarEnb = np.zeros(self.m_EnbNum,) # (1,self.m_EnbNum)
        for enb_idx in range(self.m_EnbNum):
            Q_enb_mat = np.dot(np.diag(self.m_EnbSgwMat[enb_idx,:]), Q_mat) # IxI
            Q_enb_sq  = np.dot(Q_enb_mat.T, Q_enb_mat) # IxI
            ## get A(I)^T B_j Q^2 A(I)
            #QtildaSgwEnb = np.dot(np.dot(self.m_SgwRouteMat.T, Q_enb_sq), self.m_SgwRouteMat) # RxR
            #self.o_SgwLBVarEnb[enb_idx] = var_w * np.dot(np.dot(self.m_rate_update, QtildaSgwEnb), self.m_rate_update)  #!!!!!!!!!!!!!!
            self.o_SgwLBVarEnb[enb_idx] = var_w * np.dot(np.dot(self.o_SgwLoad, Q_enb_sq), self.o_SgwLoad)
        self.o_SgwLBVarSum = np.dot(self.m_WeightSgw, self.o_SgwLBVarEnb) # 1xJ * 1xJ inner product        
    def f_calculate_lb_actual(self):
        # get SGW load ratio
        self.o_SgwLBVarEnbActual = np.zeros(self.m_EnbNum,) # (1,self.m_EnbNum)
        sgw_load_all  = self.o_ResLoadRatio[self.m_sgw_start:self.m_mux_start] # -1)
        if (self.i_LB_mode=='global'):             
            diff_ratio = sgw_load_all - self.m_ideal_lb_sgw
            diff_ratio_square = diff_ratio**2 # 1xI            
            for enb_idx in range(self.m_EnbNum):
                enbSgwVet       = self.m_EnbSgwMat[enb_idx,:]   # 1xI
                sgw_en_num      = np.sum(enbSgwVet)
                enbSgwLdVar_raw = np.dot(enbSgwVet, diff_ratio_square)    # 1xI * (1xI)' --> 1x1
                self.o_SgwLBVarEnbActual[enb_idx] = enbSgwLdVar_raw / (sgw_en_num * self.md_var_norm / self.m_normScale)
#            self.m_debug.diff_ratio = diff_ratio
        elif (self.i_LB_mode=='user'):
            #disp('user mode to get LB')
            # recalculate the user specific total load ratio over
            # allocated DCs (self.m_sgw_user_raio)
            self.f_calculate_loadUser()
            for enb_idx in range(self.m_EnbNum):
                # get loadRatio diff against a user specific ratio
                diff_ratio = sgw_load_all - self.m_sgw_user_raio[enb_idx] # 1xI
                diff_ratio_square = diff_ratio**2 # 1xI
                enbSgwVet       = self.m_EnbSgwMat[enb_idx,:]   # 1xI
                sgw_en_num      = sum(enbSgwVet)
                # remove the unallocated DCs and sum the rest up
                enbSgwLdVar_raw = np.dot(enbSgwVet, diff_ratio_square)    # 1xI * (1xI)^T --> 1x1
                self.o_SgwLBVarEnbActual[enb_idx] = enbSgwLdVar_raw / (sgw_en_num * self.md_var_norm / self.m_normScale)
        self.o_SgwLBVarSumActual = np.dot(self.m_WeightSgw, self.o_SgwLBVarEnbActual) # 1xJ * 1xJ^T
    def f_calculate_loadUser(self):
        new_sgw_load = self.o_SgwLoad #1xI
        self.m_sgw_user_load = np.dot(new_sgw_load, self.m_EnbSgwMat.transpose()) #1xI * (JxI)'-->1xJ
        self.m_sgw_user_raio = self.m_sgw_user_load / self.m_userSgwSysCap # 1xJ
    def f_update_mux_results(self):
        # get MUX load,load ratio and status
        if self.i_MUX_num > 0 and self.m_lbMux_en:
            self.o_MuxLoad = self.o_ResLoad[self.m_mux_start:self.m_link_start] # -1)            
            #self.o_MUX_status = self.o_MuxLoad > self.mc_reOnOff_th 
            self.o_MuxLoadRatio = self.o_ResLoadRatio[self.m_mux_start:self.m_link_start] #-1)
            self.o_MUX_status = (self.o_MuxLoadRatio > self.mc_reOnOff_th).astype(float)
            #disp(self.md_new_MuxNum)
            MuxLR_var        = sum((self.o_MuxLoadRatio - np.mean(self.o_MuxLoadRatio))**2 / self.md_new_MuxNum)
            self.o_MuxLR_std = np.sqrt(MuxLR_var)
            # calculate delay for each MUX 
            tcapLeft_mux = self.m_MUX_cap - self.o_ResLoad[self.m_mux_start:self.m_link_start] #-1)
            capLeft_mux  = np.maximum(tcapLeft_mux,0.0000001) # 1xM
            tem_MuxDelay = self.md_mux_config_new / capLeft_mux # 1xM               
            self.o_MuxDelay = tem_MuxDelay # .* self.o_MUX_status # 1xM                 
            # calculate delay sum for each ENB
            self.f_calculate_enb_delay()
    def f_calculate_enb_delay(self):
        self.m_MuxRouteMat = self.m_ResRouteMat[self.m_mux_start:self.m_link_start,:] # MxR
        self.md_mux_norm = 1.0 / (self.md_MUX_ntime / self.m_normScale * self.md_new_MuxNum) # 1xM
        muxDelayNorm = self.o_MuxDelay * self.md_mux_norm # 1xM, /(S^mux * M)
        route_delay = np.dot(self.m_MuxRouteMat.T, muxDelayNorm) # MxR' * 1xM'-->Rx1            
        self.o_EnbDelay    = np.dot(self.i_EnbRouteMat, route_delay)  # (JxR * Rx1)'-->1xJ               
        self.o_DelaySum    = np.dot(self.m_WeightMux, self.o_EnbDelay) # 1xJ * 1xJ' 

    def f_recordMat(self):
        if not self.m_2D_en:
            return    
        self.m_colCnt = self.i_cntIdx % self.m_colNum
        self.m_rowCnt = int((self.i_cntIdx) / self.m_colNum)
        self.m_totalPowMat[self.m_rowCnt,self.m_colCnt] = self.o_PowConsum2
        
    def f_record_results(self):            
        if self.m_firstopen==1:
            fid = open(self.i_fileName, 'w')
        else:
            fid = open(self.i_fileName, 'a')
        self.m_firstopen=0
        if self.m_title_on ==0:         
            fid.write('#########constraint mode is %s \n' %self.i_constraint_mode)
            for j in range(self.m_EnbNum):
                fid.write('user%d,weight_pow=%4.5f,weight_sgw=%4.5f,weight_mux=%4.5f \n' 
                           %(j,self.m_WeightPow[j],self.m_WeightSgw[j],self.m_WeightMux[j]))
            self.m_title_on =1
        fid.write('iteration=%d \n' %self.m_iteration)
        fid.write('route load allocation, \n')
        fid.write('BS/Route index,')
        for item in range(1):
            for r in range(self.m_RtNum):
                fid.write('r%d,' %r)
            fid.write('$,')
        fid.write('\n')
        for j in range(self.m_EnbNum):
            fid.write('eNodB%d,' %j)
            # print out traffic allocation
            for r in range(self.m_RtNum):
                fid.write('%4.5f,' %DUMMY_NUM) # dummy value %self.o_traffic[j,r])
            fid.write('\n')
        # print out sum per BS
        fid.write('sum,')
        for r in range(self.m_RtNum):
            #print 'self.m_rate_update[r]=',self.m_rate_update[r]
            fid.write('%4.5f,' %DUMMY_NUM) # dummy value # dummy value%(self.m_rate_update[r]))
        fid.write('\n')            
        fid.write('SgwIndex,load,load ratio,load status, price, slop, load ratio STD \n')
        for i in range(self.m_SgwNum):
            fid.write('SGW%d, %2.5f, %2.5f, %d,%2.5f,%2.5f,%2.5f\n' %(i,self.o_ResLoad[i],
                    self.o_ResLoadRatio[i],self.o_RES_status[i]>0,
                    DUMMY_NUM,
                    self.m_RES_peff_approx[i],
                    self.o_SgwLR_std))          
        if self.i_MUX_num > 0 :
            fid.write('MuxIndex,load,load ratio,load status, price, slop, load ratio STD, Delay \n')
            mux_start = self.m_mux_start
            mux_end   = self.m_link_start #-1
            for h in range(mux_start,mux_end):
                fid.write('MUX%d, %2.5f, %2.5f, %d,%2.5f,%2.5f,%2.5f,%2.5f\n' 
                        %(h-mux_start+1,self.o_ResLoad[h],
                        self.o_ResLoadRatio[h],self.o_RES_status[h],
                        999,
                        self.m_RES_peff_approx[h],
                        self.o_MuxLR_std,
                        self.o_MuxDelay[h-mux_start]))  #+1
        if self.i_link_num > 0:
            fid.write('LinkIndex,load,load ratio,load status, price, slop \n')
            link_start = self.m_link_start
            link_end   = self.m_ResNum
            for l in range(link_start,link_end):
                fid.write('LINK%d, %2.5f, %2.5f, %d,%2.5f,%2.5f\n' 
                        %(l-link_start+1,self.o_ResLoad[l],
                        self.o_ResLoadRatio[l],self.o_RES_status[l],
                        999,
                        self.m_RES_peff_approx[l]))
        fid.write('EnbIndex,SumPowNorm,LB variance,MUX delay \n')
        for j in range(self.m_EnbNum):
            fid.write('ENB%d, %2.5f, %2.5f, %2.5f\n' 
                        %(j,
                        self.o_powRes_enb[j],
                        self.o_SgwLBVarEnbActual[j],
                        self.o_EnbDelay[j]))                
        fid.write('o_PowConsum1,o_PowConsum2, min value, effctive min,o_powSum_norm,o_SgwLBVarSum,o_DelaySum\n')
        print self.o_SgwLBVarSum,self.o_DelaySum, self.o_fval, self.f_calculate_min_value()
        fid.write('%2.5f, %2.5f, %2.5f, %2.5f, %2.5f, %2.5f, %2.5f \n' 
                  %(self.o_PowConsum1,self.o_PowConsum2, DUMMY_NUM, self.f_calculate_min_value(),
                    DUMMY_NUM,self.o_SgwLBVarSum,self.o_DelaySum))
        fid.close()
    def f_calculate_min_value(self):  # NOT useful calculation !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        min_value_pow = np.dot(self.m_RES_peff_approx, self.o_ResLoad) # inner product of 2 vectors
        Qsrv_mat = self.f_form_QsrvMat(self.md_C_sys_sgw)
        #print np.shape(self.o_SgwLoad), np.shape(Qsrv_mat)
        min_value_lb_sgw = np.dot(np.dot(self.o_SgwLoad, Qsrv_mat), self.o_SgwLoad) #
        min_delay_mux = self.f_form_mux_delay(self.o_ResLoad)
        #disp(min_value_pow);disp(min_value_lb_sgw);disp(min_delay_mux);
        eff_min = min_value_pow + min_value_lb_sgw + min_delay_mux
        return eff_min
    def f_form_mux_delay(self,resLoad):  
        if not self.m_lbMux_en:
            delay_mux = 0
            return delay_mux
        mux_start = self.m_mux_start
        mux_end   = self.m_link_start #-1        
        # get MUX throughput
        mux_traffic = resLoad[mux_start:mux_end] # np.dot(d_route, self.m_MuxRouteMat.T) # 1xR * MxR'=1xM
        # get active MUX entity delay
        mux_dif = np.maximum(1e-10,self.m_MUX_cap-mux_traffic)
        delay_mux_active = self.md_mux_config_new / mux_dif # 1xM
        # get 1/(M*md_mux_norm)
        mux_weight = 1.0 / (self.md_MUX_ntime * self.md_new_MuxNum) # 1xM
        # get weighted mux delay
        delay_mux_norm = delay_mux_active * mux_weight # 1xM
        # get MUX delay sum per route
        delay_mux_route = np.dot(delay_mux_norm, self.m_MuxRouteMat) # 1xM * MxR = 1xR
        # get MUX delay sum per user
        delay_mux_enb   = np.dot(delay_mux_route, self.m_EnbRouteMat.T) # 1xR * JxR'=1xJ
        # get MUX delay over users scaled by weights            
        delay_mux = np.dot(delay_mux_enb, self.m_WeightMux) # 1xJ * 1xJ'=1x1            
#             disp('ok for mux lb')
        return delay_mux        

    @property
    def bwMeasure(self):
        return self.m_TrafficDataAvg
    @bwMeasure.setter
    def bwMeasure(self, value):
        assert(len(value) == 2) # need raw data and mode control
        mode    = value[0]
        try:
            rawData = np.array(value[1])
            if len(rawData) != self.m_ResNum:
                raise ('inconsistence data length %d' % len(rawData))
        except ValueError:
            print 'can NOT convert to numpy with input:',  value[1]
            return
        if mode == RECORD_ONCE:
            self.m_TrafficBuffer = [rawData]
            self.m_TrafficDataAvg = rawData
        elif mode == RECORD_MULTI:
            self.m_TrafficBuffer.append(rawData)
            self.m_TrafficDataAvg = np.average(np.array(self.m_TrafficBuffer),axis=0)
        else:
            raise('unsupported mode: %s' % mode)

    @property 
    def pingMeasure(self):
        return self.m_pingData
    @pingMeasure.setter
    def pingMeasure(self, value):
        if not value:
            return
        assert(len(value) == self.m_EnbNum) # need to align host number
        self.m_pingData = value
        self.m_logPing_en = True
        
    @property
    def pingMeasureSingle(self):
        return self.m_pingDataSingleDict
    @pingMeasureSingle.setter
    def pingMeasureSingle(self,dictIn):
        if not dictIn:
            return
        cnTStr   = '_'+str(self.i_cntIdx)
        weightStr=('_Wp'+str(self.m_WeightPow[0])
                   +'_Ws'+str(self.m_WeightSgw[0])
                   +'_Wm'+str(self.m_WeightMux[0]))
        tStr     = ''
        for key, value in self.m_trueTrafficDict.iteritems():
            tStr += '_T'+key+'_'+str(value)             
        f_matStr = PINT_MAT_OUT_PREFIX + cnTStr + weightStr + tStr + '.mat'
        print 'f_matStr=',f_matStr
        scipy.io.savemat(f_matStr,dictIn)
   
    def f_analyse(self):
        # analys data
        self.f_update_results()
        # for 2D mode only
        self.f_recordMat()
        # recored data
        self.f_record_results()
        # record self format results
        self.f_record_results_sim(self.i_cntIdx,self.i_fsimName)
        # record ping results
        self.f_record_results_ping(self.i_cntIdx,self.i_fpingName)
        # for 2D mode only  
        self.f_logMat()      
        
    def f_record_results_sim(self,cntIdx,file_dest):
#         if not self.m_srecord_en:
#             return    
        if self.m_firstopen_sim==1:
            if cntIdx==0:
                fid = open(file_dest, 'w')
            else:
                fid = open(file_dest, 'a')
            self.f_record_header(fid,cntIdx)
            self.m_title_on_sim = 1
            self.f_record_content(fid,cntIdx)
        else:
            fid = open(file_dest, 'a')
            if not self.m_title_on_sim:
                self.f_record_header(fid,cntIdx)
                self.m_title_on_sim = 1
            self.f_record_content(fid,cntIdx)
        self.m_firstopen_sim=0    
        fid.close()       
    def f_record_header(self,fid_in,cntIdx):
        fid_in.write('\n')
        for j in range(self.m_EnbNum):
            fid_in.write('user%d,weight_pow=%4.5f,weight_sgw=%4.5f,weight_mux=%4.5f \n'
                         %(j+1,self.m_WeightPow[j],self.m_WeightSgw[j],self.m_WeightMux[j]))
             
        if self.i_control_mode==0: # distributed mode
            fid_in.write('mc_bs_agg_gradient=%2.2f \n' %DUMMY_NUM) #self.mc_bs_agg_gradient);
        else:
            fid_in.write('no gradient for distributed mode\n')
   
        fid_in.write('Cnt+1,')
        for j in range(self.m_EnbNum):
            fid_in.write('Enb%d_data,' %(j+1))
        fid_in.write('sum_data,')
        fid_in.write('min_value,')                                  
        fid_in.write('pow_incre,')
        fid_in.write('pow_total,')
        for i in range(self.i_SGW_num):
            fid_in.write('Sgw%d loadRatio,' %(i+1))
        if self.o_EnbDelay != None:
            for j in range(self.m_EnbNum):                
                fid_in.write('Enb%d Delay,' %(j+1))
            fid_in.write('DelaySum,');
        if self.o_SgwLBVarEnbActual != None:                
            for j in range(self.m_EnbNum):
                fid_in.write('Enb%d LB var,' %(j+1))
            fid_in.write('LBVarSum,')
        if self.o_powRes_enb != None:
            for j in range(self.m_EnbNum):
                fid_in.write('Enb%d pow approx,' %(j+1))
            fid_in.write('PowNorm,')  
        fid_in.write('sgwLR_std,')
        fid_in.write('muxLR_std,')
        if self.m_plot_en:                
            # record RES status
            for i in range(self.i_SGW_num):
                fid_in.write('Sgw%d,' %(i+1))          
            for h in range(self.i_MUX_num):
                fid_in.write('Mux%d,' %(h+1))           
            for l in range(self.i_link_num):
                fid_in.write('Link%d,' %(l+1))
        fid_in.write('\n')        
    def f_record_content(self,fid_in,cntIdx):
        if not self.m_2D_en:
            fid_in.write('%d,' %cntIdx)
        else:
            fid_in.write('%d:[%d %d],' %(cntIdx,self.m_rowCnt,self.m_colCnt))
        for j in range(self.m_EnbNum):
            fid_in.write('%2.5f,' %self.i_traffic_in[j])
        fid_in.write('%2.5f,' %sum(self.i_traffic_in))
        fid_in.write('%s,' %'?') # fid_in.write('%2.5f,',self.o_fval); %;
        fid_in.write('%2.5f,' %self.o_PowConsum1)
        fid_in.write('%2.5f,' %self.o_PowConsum2)
        for i in range(self.i_SGW_num):
            fid_in.write('%2.5f,' %self.o_SgwLoadRatio[i])
        if self.o_EnbDelay != None:
            for j in range(self.m_EnbNum):
                fid_in.write('%2.5f,' %self.o_EnbDelay[j])
            fid_in.write('%2.5f,' %self.o_DelaySum)
        if self.o_SgwLBVarEnbActual != None:
            for j in range(self.m_EnbNum):
                fid_in.write('%2.5f,' %self.o_SgwLBVarEnbActual[j])
            fid_in.write('%2.5f,' %self.o_SgwLBVarSumActual)
        if self.o_powRes_enb != None:
            for j in range(self.m_EnbNum):
                fid_in.write('%2.5f,' %self.o_powRes_enb[j])
            fid_in.write('%2.5f,' %DUMMY_NUM)           
        fid_in.write('%2.5f,' %self.o_SgwLR_std)
        fid_in.write('%2.5f,' %self.o_MuxLR_std)
        if self.m_plot_en:                
            for i in range(self.i_SGW_num):
                fid_in.write('%d,' %self.o_SGW_status[i])
            for h in range(self.m_mux_start,self.m_link_start):
                fid_in.write('%d,' %self.o_RES_status[h])
            for l in range(self.m_link_start,self.m_ResNum):
                fid_in.write('%d,' %self.o_RES_status[l])
        fid_in.write('\n')

    def f_record_results_ping(self,cntIdx,file_dest):
        if not self.m_logPing_en:
            return    
        if self.m_firstopen_ping==1:
            if cntIdx==0:
                fid = open(file_dest, 'w')
            else:
                fid = open(file_dest, 'a')
            self.f_record_header_ping(fid,cntIdx)
            self.m_title_on_ping = 1
            self.f_record_content_ping(fid,cntIdx)
        else:
            fid = open(file_dest, 'a')
            if not self.m_title_on_ping:
                self.f_record_header_ping(fid,cntIdx)
                self.m_title_on_ping = 1
            self.f_record_content_ping(fid,cntIdx)
        self.m_firstopen_ping=0    
        fid.close()        
    def f_record_header_ping(self,fid_in,cntIdx):
        fid_in.write('\n')
        for j in range(self.m_EnbNum):
            fid_in.write('user%d,weight_pow=%4.5f,weight_sgw=%4.5f,weight_mux=%4.5f \n'
                         %(j+1,self.m_WeightPow[j],self.m_WeightSgw[j],self.m_WeightMux[j]))
             
        if self.i_control_mode==0: # distributed mode
            fid_in.write('mc_bs_agg_gradient=%2.2f \n' %DUMMY_NUM) #self.mc_bs_agg_gradient);
        else:
            fid_in.write('no gradient for distributed mode\n')
   
        fid_in.write('Cnt,')
        for j in range(self.m_EnbNum):
            fid_in.write('Enb%d_data,' %(j+1))
        fid_in.write('sum_data,')
        for j in range(self.m_EnbNum):
            fid_in.write('Enb%d_transmitted,' %(j+1)) 
        for j in range(self.m_EnbNum):
            fid_in.write('Enb%d_received,' %(j+1))
        for j in range(self.m_EnbNum):
            fid_in.write('Enb%d_lossRatio,' %(j+1))
        for j in range(self.m_EnbNum):
            fid_in.write('Enb%d_totalTime,' %(j+1))
        ###########
        for j in range(self.m_EnbNum):
            fid_in.write('Enb%d_min,' %(j+1)) 
        for j in range(self.m_EnbNum):
            fid_in.write('Enb%d_ave,' %(j+1))
        for j in range(self.m_EnbNum):
            fid_in.write('Enb%d_max,' %(j+1))
        for j in range(self.m_EnbNum):
            fid_in.write('Enb%d_mdev,' %(j+1))           
        fid_in.write('\n')        
    def f_record_content_ping(self,fid_in,cntIdx):
        if not self.m_2D_en:
            fid_in.write('%d,' %cntIdx)
        else:
            fid_in.write('%d:[%d %d],' %(cntIdx,self.m_rowCnt,self.m_colCnt))
        for j in range(self.m_EnbNum):
            fid_in.write('%2.5f,' %self.i_traffic_in[j])
        fid_in.write('%2.5f,' %sum(self.i_traffic_in))
        for item in range(PING_ITEMS):
            for j in range(self.m_EnbNum):
                hostKey = 'h'+str(j+1)
                fid_in.write('%s,' %self.m_pingData[hostKey][item])
        # log time unit
        fid_in.write('%s,' %self.m_pingData[hostKey][-1])
        fid_in.write('\n')

    def f_logMat(self):
        if not self.m_2D_en:
            return        
        if self.i_cntIdx < self.m_boundary or self.i_cntIdx > self.m_boundary:
            return
        if self.m_tPowfirstopen==1:
            fid = open(self.m_fpowName, 'w')
            self.f_record_tPow(fid)
        else:
            fid = open(self.m_fpowName, 'a')
            self.f_record_tPow(fid);
        self.m_tPowfirstopen=0;         
        fid.close()
    def f_record_tPow(self,fid_in):
        fid_in.write('\n')
        for j in range(self.m_EnbNum):
            fid_in.write('user%d,weight_pow=%2.2f,weight_sgw=%2.2f,weight_mux=%2.2f \n'
                         %(j+1,self.m_WeightPow[j],self.m_WeightSgw[j],self.m_WeightMux[j]))             
        if self.i_control_mode==0: # distributed mode
            fid_in.write('mc_bs_agg_gradient=%2.2f \n' %DUMMY_NUM) #self.mc_bs_agg_gradient);
        else:
            fid_in.write('no gradient for distributed mode\n')
        # first line for col 
        for col in range(self.m_colNum):
            t_col = self.m_traffic_in_s[1] + (col) * self.m_traf_step
            fid_in.write(',%2.5f' %t_col)
        for row in range(self.m_rowNum):
            t_row = self.m_traffic_in_s[0] + (row) * self.m_traf_step
            fid_in.write('\n%2.5f' %t_row)
            for col in range(self.m_colNum):
                fid_in.write(',%2.5f' %self.m_totalPowMat[row,col])               
        fid_in.write('\n')


class ResultsRecorder_P1 (ResultsRecorder):  
    ''' overloaded function''' 
    def f_record_header(self,fid_in,cntIdx):
        fid_in.write('\n')
        for j in range(self.m_EnbNum):
            fid_in.write('user%d,weight_pow=%4.5f,weight_sgw=%4.5f,weight_mux=%4.5f \n'
                         %(j+1,self.m_WeightPow[j],self.m_WeightSgw[j],self.m_WeightMux[j]))
             
        if self.i_control_mode==0: # distributed mode
            fid_in.write('mc_bs_agg_gradient=%2.2f \n' %DUMMY_NUM) #self.mc_bs_agg_gradient);
        else:
            fid_in.write('no gradient for distributed mode\n')
   
        fid_in.write('Cnt,')
        for j in range(self.m_EnbNum):
            fid_in.write('Enb%d_data,' %(j+1))
        fid_in.write('sum_data,')
        fid_in.write('min_value,')                                  
        fid_in.write('pow_incre,')
        fid_in.write('pow_total,')
        for i in range(self.i_SGW_num):
            fid_in.write('Sgw%d loadRatio,' %(i+1))        
        fid_in.write('sgwLR_std,')
        fid_in.write('muxLR_std,')
        if self.m_plot_en:                
            # record RES status
            for i in range(self.i_SGW_num):
                fid_in.write('Sgw%d,' %(i+1))          
            for h in range(self.i_MUX_num):
                fid_in.write('Mux%d,' %(h+1))           
            for l in range(self.i_link_num):
                fid_in.write('Link%d,' %(l+1))
        fid_in.write('\n');
    ''' overloaded function'''    
    def f_record_content(self,fid_in,cntIdx):
        if not self.m_2D_en:
            fid_in.write('%d,' %cntIdx)
        else:
            fid_in.write('%d:[%d %d],' %(cntIdx,self.m_rowCnt,self.m_colCnt))
        for j in range(self.m_EnbNum):
            fid_in.write('%2.5f,' %self.i_traffic_in[j])
        fid_in.write('%2.5f,' %sum(self.i_traffic_in))
        fid_in.write('%s,' %'?') # fid_in.write('%2.5f,',self.o_fval); %;
        fid_in.write('%2.5f,' %self.o_PowConsum1)
        fid_in.write('%2.5f,' %self.o_PowConsum2)
        for i in range(self.i_SGW_num):
            fid_in.write('%2.5f,' %self.o_SgwLoadRatio[i])
        fid_in.write('%2.5f,' %self.o_SgwLR_std)
        fid_in.write('%2.5f,' %self.o_MuxLR_std)
        if self.m_plot_en:                
            for i in range(self.i_SGW_num):
                fid_in.write('%d,' %self.o_SGW_status[i])
            for h in range(self.m_mux_start,self.m_link_start):
                fid_in.write('%d,' %self.o_RES_status[h])
            for l in range(self.m_link_start,self.m_ResNum):
                fid_in.write('%d,' %self.o_RES_status[l])
        fid_in.write('\n')

                    
if __name__ == "__main__": 
    ''' set up input '''      
    #matIn = f_load_mat('USNET_2.mat')
    matIn = f_load_mat('../alg/case2.mat')
    '''
    D,C,PMAX,Beta,I,H,L,EM,RM,
    Weight,Threshold,Iteration,NormMode,Outloop,
    GameMode,RES_ratio,constraint_mode,
    '''
    #print matIn
    D = matIn['D'].flatten();C = matIn['S'].flatten();PMAX = matIn['PMAX'].flatten();Beta = matIn['Beta'].flatten();    
    I = matIn['I'].flatten()[0];H = matIn['H'].flatten()[0];L = matIn['L'].flatten()[0];EM = matIn['EM'];RM = matIn['RM'];
    Weight = 0.1 # matIn['Weight']  #  0.5 #   
    Threshold = matIn['Threshold'].flatten()[0];Iteration = matIn['Iteration'];
    NormMode = [];Outloop = 0;
    o_file = 'test_analyser_sdn.txt'
    GameMode = matIn['Mode'][0];RES_ratio = matIn['res_ratio'].flatten();constraint_mode = 'lag'       
    ################################################    
    analyser = ResultsRecorder(D,C,PMAX,Beta,I,H,L,EM,RM,
                         Weight,Threshold,Iteration,NormMode,Outloop,
                         GameMode,RES_ratio,constraint_mode,
                         i_fileName=o_file)
    analyser.i_fsimName = 'test_analyser_sdn_sim.txt'
    din1 = [0.0,0.91,0.49,0.0,1.4,0.0,0.5,0.0,0.9,0.0,0.0,0.0,0.0,0.91,0.48]
    analyser.bwMeasure = [RECORD_ONCE,din1]
    analyser.f_analyse()
    din2 = [1.0,0.91,0.49,0.0,1.4,0.0,0.5,0.0,0.9,0.0,0.0,0.0,0.0,0.91,0.48]
    analyser.bwMeasure = [RECORD_MULTI,din2]
    analyser.f_analyse()
    #print analyser.m_TrafficBuffer
    
        
        
        
        
        
        
        
        
        
        
                