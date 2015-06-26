#!/usr/bin/python

'''
Cloud project:
- Software Defined Networking (SDN) 
-- project related algorithm (template)

Tony Lin
'''
#import re
import numpy as np
#from scipy.optimize import minimize
#import scipy.io

TEM_OUT = 'fmin_iteration_out.txt'
'''
['EM', 'res_ratio', 'D', 'Weight', 'I', 'H', 'Beta', 'Iteration', 'S', 'L', 
'NormMode', 'BsAggC', 'RM', 'Threshold', 'Outloop', 'ResCapC', 'PMAX', 'Mode']
'''
class projSolverTopTemplate(object):
    def __init__(self,D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,LB_mode='user',
                 i_sdiag = None, normScale=None,
                 i_fileName=TEM_OUT,                 
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
        self.i_sdiag = i_sdiag         # Small value to be added on diagnal of sigular matrix  
        self.i_outloop_en=Outloop
        self.io_method_mode = GameMode      # [] for GR for gradient mode, BR for best response mode 
        self.i_RES_ratio = np.array(RES_ratio)
        self.i_constraint_mode = constraint_mode
        self.i_LB_mode = LB_mode
        #print self.i_LB_mode
        self.i_fileName = i_fileName
        #print self.i_fileName
        self.i_normScale = normScale
                
        self.m_plot_en=plot_en
        self.m_srecord_en=srecord_en # Enable single results recording 
        '''init class member'''
        self.f_init_m()
        # call member def
        self.f_check()
        self.f_init()
    def f_init_m(self):    
        self.m_pen_eq_weight  =300000.0  #best is 1.3
        self.m_pen_ieq_weight =500000.0
          # optinal input
        self.io_dif_mode='sgw' # check 'sgw' or 'res' status to trigger external loop        
        self.io_scale_mode=None    # special case for w=0 with res of small pow/cap difference
          # dynamic input
          # class member
        self.m_WeightPow=None     # weight vector for power, dim: 1xJ
        self.m_WeightSgw=None     # Weight for SGW
        self.m_WeightMux=None     # Weight for MUX
        self.m_throughput=None    # system throughput        
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
        self.m_RES_peff=None      # RES power efficiency dim: N
        self.m_res_appox_en=None  # RES power efficiency approximation mode enable: [0,1]
        self.m_RES_peff_approx=None # RES power efficiency approximation by utilisation ratio, dim: N
        self.m_RES_p0=None        # RES idle power cosumption dim: N        
        self.m_SGW_cap=None       # SGW capability dim: I  
        self.m_userSgwSysCap=None # user specific/allocated SGWs' capacity sum, dim: J
        self.m_userMuxSysCap=None # user specific/allocated MUXs' capacity sum, dim: J
        self.m_SGW_peff=None      # SGW power efficiency dim: I
        self.m_SGW_p0=None        # SGW idle power cosumption dim: I
        self.m_MUX_cap=None       # MUX capability dim: H
        self.m_MUX_peff=None      # MUX power efficiency dim: H
        self.m_MUX_p0=None        # MUX idle power cosumption dim: H
        self.m_Link_cap=None      # Link constrain dime: L
        self.m_Link_peff=None     # Link power efficiency dim: L
        self.m_Link_p0=None       # Link idle power cosumption dim: L
        self.m_pmax=None            # dynamicly updated max power sum for active RES
        self.m_Evec=None            # dynamicly normalised slop vector Dim: N
        self.m_fvec=None          # f vector for first order in solver       
        self.m_RES_cap=None       # dynamic RES capacity for active RES Dim: N
        self.m_sdiag=None         # small value added to diagonal
        self.m_rate_update=None   # rate allocation aggregation update for each route dim: 1xR        
        self.m_load_update=None   # load update for each RES dim: N
        self.m_Pratio=None        # Power utilisation ratio for normalisation
        self.m_ideal_lb_sgw=None  # ideal LB ratio for sgw
        self.m_sgw_user_load=None # user specific SGW load for user load balance mode, dim: 1xJ
        self.m_sgw_user_raio=None # user specific load ratio for user load balance mode, dim: 1xJ
        self.m_ideal_lb_mux=None  # ideal LB ratio for mux
        self.m_load_ratio_th=None # Actual Load ratio threshold to avoid very low resorece utilisation       
        self.m_iter_limit=None    # iteration limit
        self.m_iteration=0        # number of iteration ??????????????????????????????????
        self.m_lbMux_en=None      # load balancing MUX enable
        self.m_varNorm_en=None    # load variance normalisation
        self.m_normScale = 1  # constant for level normalisation factors for POW,LB and delay (0.08,0.05,0.02
          # penalty related
        self.m_Bsquare_pen=None   # \lambda.*B^rt'*B^rt for eq constraint penalty
        self.m_fvec_eq_pen=None   # -2\lambda.*d^in*B^rt' for eq constraint penalty
        self.m_const_pen=0  # \lambda.*d^in*d^in' for eq constraint penalty
        self.m_rescap_pen=None    # RES capacity penalty
          # dynamic updats
        self.md_sgw_config_pre=None  # vector of pre SGW configuration 1-->en, 0-->disable
        self.md_sgw_config_new=None  # vector of new SGW configuration 1-->en, 0-->disable
        self.md_dif_sgw=0      # flag for in/out active sgw change 1-->change, 0-->no change
        self.md_new_SgwNum=None      # new SGW dim for outter loop        
        self.md_mux_config_pre=None  # vector of pre MUX configuration 1-->en, 0-->disable
        self.md_mux_config_new=None  # vector of new MUX configuration 1-->en, 0-->disable
        self.md_dif_mux=0      # flag for in/out active mux change 1-->change, 0-->no change
        self.md_new_MuxNum=None      # new MUX dim for outter loop
        self.md_max_pow=None         # dynamicly updated max power based on RES reconfig for max power normalisation
        self.md_pnorm=None           # RES normalised power consumption     
        self.md_new_ResNum=None      # new resource dim for outter loop 
        self.md_res_config_prev=None  # vector of pre SGW configuration 1-->en, 0-->disable
        self.md_res_config_new=None     # enabling map for resource,dim:1xN
        self.md_dif_res=0      # flag for in/out active RES change 1-->change, 0-->no change        
        self.md_dif=0          # flag to trigger external loop, 1-->change, 0-->no change
        self.md_C_sys_sgw=None       # dynamic SGW capacity sum
        self.md_C_sys_mux=None       # dynamic MUX capacity sum
        self.md_var_norm=None        # dynamic LB variance norm factor
        self.md_MUX_ntime=None       # MUX normalised time delay,dim: 1xM
        self.md_mux_norm=None        # norm weighting factor 1/(M*S_MUX)
        self.md_Qtilda=None          # w/I * A(I)^T Q^2 A(I) for variance matrix
        self.md_QtildaSgw=None
        self.md_QtildaMux=None
          # constant
        self.mc_max_iteration=8000 # maximal iteration number  
#         self.mc_reOnOff_th=1.0e-10  # RES on/off threshold for traffic
        self.mc_reOnOff_th= 1.0e-4  # RES on/off threshold for traffic   
        self.mc_minRate   = 1.0e-10      
          # plot related can be disabled without any fig
        self.m_plot_en=None       # Enable plot 
        self.m_srecord_en=None    # Enable single results recording       
          # file operation
        self.m_firstopen=1   # bull para for first time open file
        self.m_title_on=0
        self.m_fig_dir=None       # fig saving dir
          # debug
        self.m_debug=None
        # output
        self.o_fval=None         # output optimal value
        self.o_traffic=None       # output of traffic allocation matrix: J * R
        self.o_Amatrix=None       # output allocation ratio matrix J * R
        self.o_ratioList =[]      # list of ratio for all J       [J]
        self.o_ResLoad=None          # load for each RES dim: N
        self.o_ResLoadRatio=None     # load ratio (load/capability) for each RES dim: N  
        self.o_RES_status=None    # RES status 1/0 --> on/off dim: N  
        self.o_SgwLoad=None          # load for each SGW dim: I
        self.o_SgwLoadRatio=None     # load ratio (load/capability) for each SGW dim: I
        self.o_SgwLR_std=None     # STD of SGW load ratio
        self.o_SgwLBVarEnb=None   # SGW LB variance per ENB dim: 1xJ
        self.o_SgwLBVarEnbActual=None # actural LB variance per ENB dim: 1xJ
        self.o_SgwLBVarSum=None   # weighted sum of SGW LB variance over ENB
        self.o_SgwLBVarSumActual=None   # Actual weighted sum of SGW LB variance over ENB
        self.o_SGW_status=None    # SGW status 1/0 --> on/off dim: I   
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
        self.o_powSum_norm=None   # sum of weighted o_powRes_enb

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
        print'constraint mode is %s ' %self.i_constraint_mode
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
        # intial exit, inner, outer loop related
        self.f_init_loop()
        # initial plot, fig
        self.f_init_plot()                                         
        # Init output
        self.f_init_out()       
    def f_init_set(self):   
        # Initial rate update
        self.m_rate_update = np.zeros((self.m_RtNum,)) # (1,self.m_RtNum)
        # Initial resource enabling map
        self.md_res_config_pre = np.ones((self.m_ResNum,)) # (1,self.m_ResNum)
        self.md_res_config_new = np.ones((self.m_ResNum,)) # (1,self.m_ResNum)
        # Init pre and new SGW reconfig vector with all 0
        self.md_sgw_config_pre = np.zeros((self.m_ResNum,))# (1,self.m_ResNum)
        # Set new SGW reconfig vector with all 1
        self.md_sgw_config_new = np.ones((self.m_SgwNum,)) # (1,self.m_SgwNum)
        # Set new MUX reconfig vector with all 1
        self.md_mux_config_new = np.ones((self.m_MuxNum,)) # (1,self.m_MuxNum)
        # Initial SGW load 
        self.m_load_update = np.zeros((self.m_ResNum,))    # (1,self.m_ResNum)
        self.f_init_sdiag()
    def f_init_sdiag(self):
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
            self.m_WeightSgw = self.i_Weight[0,0] / 2 * t1
            self.m_WeightMux = self.i_Weight[0,0] / 2 * t1
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
                #print p_weight, pow_scale
                self.md_pnorm = p_weight*sum(self.md_max_pow)*pow_scale 
        elif self.io_norm_mode=='max':
            self.md_pnorm = self.i_RES_maxpow
        else:
            raise Exception('Incorrect SGW normalisation mode')  
        # scale norm weights to proper level
        self.md_pnorm = self.md_pnorm * self.m_normScale
        print 'True pow normaliser = %2.5f ' %self.md_pnorm                               
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
        print 'True var normaliser = %2.5f ' %self.md_var_norm            
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
            print 'True mux normaliser = ', self.md_MUX_ntime       
    def f_init_userSysCap(self):
        if self.i_LB_mode=='user':
            self.m_userSgwSysCap = np.dot(self.m_SGW_cap, self.m_EnbSgwMat.transpose()) #1xI * (JxI)'-->1xJ
            self.m_userMuxSysCap = np.dot(self.m_MUX_cap, self.m_EnbMuxMat.transpose()) #1xM * (JxM)'-->1xJ

    def f_calculate_loadUser(self):
        new_sgw_load = self.m_load_update[self.m_sgw_start : self.m_mux_start] #1xI
        self.m_sgw_user_load = np.dot(new_sgw_load, self.m_EnbSgwMat.transpose()) #1xI * (JxI)'-->1xJ
        self.m_sgw_user_raio = self.m_sgw_user_load / self.m_userSgwSysCap # 1xJ               
    def f_init_loop(self):
        # get iteration limit
        if not(self.io_iter_limit):
            self.m_iter_limit = self.mc_max_iteration
        else:
            self.m_iter_limit = self.io_iter_limit      
    def f_init_plot(self):      
        # Initial fig dir
        self.m_fig_dir = 'results/MatFig/'
        # get load ratio threshold
        if not(self.io_load_ratio_th):
            self.m_load_ratio_th = 0.0                
        else:
            self.m_load_ratio_th = self.io_load_ratio_th         
    def f_init_out(self):
        self.o_traffic = np.zeros((self.m_EnbNum, self.m_RtNum))
        self.o_Amatrix = np.zeros((self.m_EnbNum, self.m_RtNum))  
        self.o_SgwLoadRatio = np.zeros((1,self.m_SgwNum))
        self.o_RES_status   = np.ones((1,self.m_ResNum))
        self.o_powRes_enb   = np.zeros(self.m_EnbNum,)

    def f_form_active_map(self):            
        # form active RES-route map A based on md_res_config_new
        #bsxfun(@times,self.md_res_config_new',self.i_ResRouteMat) in matlab
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
            #bsxfun(@times,ActiveRtVect,self.i_EnbRouteMat) in matlab
        self.m_EnbRouteMat = self.i_EnbRouteMat * ActiveRtVect # JxR b* 1xR -->JxR (python index)                        
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            # form Enb-Res mapping matrix, DIM: J x N
            # (self.m_EnbRouteMat * self.m_ResRouteMat')>0 #JxR * RxN=JxN in matlab
        self.m_EnbResMat = (np.dot(self.m_EnbRouteMat, self.m_ResRouteMat.T)>0).astype(float) #JxR * (NxR)^T=JxN
            # form Enb-Sgw mapping matrix, DIM: J x I
            # (self.m_EnbRouteMat * self.m_SgwRouteMat')>0 #JxR * RxI=JxI
        self.m_EnbSgwMat = (np.dot(self.m_EnbRouteMat, self.m_SgwRouteMat.T)>0).astype(float) #JxR * (IxR)^T=JxI
            # form Enb-Mux mapping matrix, DIM: J x M
            # (self.m_EnbRouteMat * self.m_MuxRouteMat')>0 #JxR * RxM=JxM in matlab
        self.m_EnbMuxMat = (np.dot(self.m_EnbRouteMat, self.m_MuxRouteMat.T)>0).astype(float)  #JxR * (MxR)^T=JxM     
    def f_form_fmin_syspow(self):
        slop_norm = self.m_RES_peff_approx / self.md_pnorm #1xN
        self.m_fvec = np.zeros((1,self.m_RtNum))
        for enb_idx in range(self.m_EnbNum):
            # get weight for power for user j
            power_w = self.m_WeightPow[enb_idx]         
            # form eff vector
            self.m_Evec = power_w * slop_norm # 1xN
            # form vector as f for first order coefficient in slover
            # get EAB_j=E*A*B_j
            Bj_diag = np.diag(self.m_EnbRouteMat[enb_idx,:]) # RxR
            # 1xN * NxR * RxR --> 1xR  %%%%%%%%%%need to confirm to use i_ResRouteMat or m_ResRouteMat
            EAB_j = np.dot(np.dot(self.m_Evec, self.i_ResRouteMat), Bj_diag) #
            self.m_fvec = self.m_fvec+EAB_j #1xR   
#                 print self.m_EnbRouteMat[enb_idx,:]
#            self.m_debug.user[enb_idx].power_term = EAB_j  

    def f_form_fmin_lb_sgw_p1(self):
        # get active system capacity
        C_sys  = sum(self.m_SGW_cap * self.md_sgw_config_new)               # C_sys
        #print self.md_new_SgwNum
        Q_mat  = -(1.0/C_sys) * np.ones((self.md_new_SgwNum,self.md_new_SgwNum))  # -1/C_sys
        t_diag = np.diag(1./self.m_SGW_cap)                  # diag 1/c_i 
        Q_mat  = Q_mat  + t_diag  # C_i -1/S_sys at diag
        Q_sq   = np.dot(Q_mat.T,Q_mat)                                   # Q*Q   
        #print Q_sq 
        # get active route from active MUX-route mat A(M)
        #self.m_MuxRouteMat = self.m_ResRouteMat(self.m_mux_start:self.m_link_start-1,:);
        # get A(M)^T Q^2 A(M)
        self.md_QtildaSgw = np.dot(np.dot(self.m_SgwRouteMat.T, Q_sq), self.m_SgwRouteMat)
        # vaiance weight: w/I
        var_w = np.average(self.m_WeightSgw) / self.md_new_SgwNum / self.md_var_norm
        #print self.m_WeightSgw , self.md_new_SgwNum , self.md_var_norm
        self.md_QtildaSgw = var_w * self.md_QtildaSgw 
        self.md_Qtilda    = 2*self.md_QtildaSgw 
        #print var_w,'for journal 1', self.md_QtildaSgw
    def f_form_qp_lb_mux_p1(self):
        # get active system capacity
        C_sys  = sum(self.m_MUX_cap * self.md_mux_config_new)               # C_sys
        #print self.md_new_MuxNum
        Q_mat  = -(1.0/C_sys) * np.ones((self.md_new_MuxNum,self.md_new_MuxNum))  # -1/C_sys
        t_diag = np.diag(1./self.m_MUX_cap)                  # diag 1/c_i 
        Q_mat  = Q_mat  + t_diag  # C_i -1/S_sys at diag
        Q_sq   = np.dot(Q_mat.T,Q_mat)                                   # Q*Q    
        # get active route from active MUX-route mat A(M)
        #self.m_MuxRouteMat = self.m_ResRouteMat(self.m_mux_start:self.m_link_start-1,:);
        # get A(M)^T Q^2 A(M)
        self.md_QtildaMux = np.dot(np.dot(self.m_MuxRouteMat.T, Q_sq), self.m_MuxRouteMat)
        # vaiance weight: w/I
        var_w = np.average(self.m_WeightMux) / self.md_new_MuxNum / self.md_var_norm
        self.md_QtildaMux = var_w * self.md_QtildaMux 
        self.md_Qtilda    = self.md_Qtilda + 2*self.md_QtildaMux 
        #print 'for journal 1', self.md_QtildaMux
                        
    def f_form_fmin_lb_sgw(self):
        # for 'global' mode get same Qsrv_mat for all uers since C_sys
        # are the same
        if (self.i_LB_mode=='global'): 
            Qsrv_mat = self.f_form_QsrvMat(self.md_C_sys_sgw,self.i_SGW_num,self.m_SGW_cap)
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # inite self.md_QtildaSgw
        self.md_QtildaSgw = np.zeros((self.m_RtNum, self.m_RtNum)) # RxR
        for enb_idx in range(self.m_EnbNum):
            if (self.i_LB_mode=='global'): 
                # diag(self.m_EnbSgwMat(enb_idx,:)) * Qsrv_mat # B*Q IxI
                Q_enb_mat = np.dot(np.diag(self.m_EnbSgwMat[enb_idx,:]), Qsrv_mat) # IxI
            elif (self.i_LB_mode=='user'):
                # use user specific C_sys,j to calculate Q^srv,j matrix
                Qsrv_mat = self.f_form_QsrvMat(self.m_userSgwSysCap[enb_idx],self.i_SGW_num,self.m_SGW_cap) # IxI
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
    def f_form_QsrvMat(self,C_sys_in,re_num,re_cap):
        # get active system capacity based on user/global mode
        C_sys  = C_sys_in               # C_sys
#             Q_mat  = -(1.0/C_sys)*ones(self.md_new_SgwNum,self.md_new_SgwNum)  # -1/C_sys
        #Q_mat  = -(1.0/C_sys) * np.ones((self.i_SGW_num,self.i_SGW_num))  # -1/C_sys, No need for dynamic SGW number
        Q_mat  = -(1.0/C_sys) * np.ones((re_num,re_num))  # -1/C_sys, No need for dynamic SGW number
        #t_diag = np.diag(1 / self.m_SGW_cap + self.m_sdiag)                    # diag(1/c_i+\tau)
        t_diag = np.diag(1 / re_cap + self.m_sdiag)                    # diag(1/c_i+\tau)
        Q_mat  = Q_mat  + t_diag 
        return Q_mat 

        
    def f_form_qp_lb_mux(self):         
        # for 'global' mode get same Qsrv_mat for all uers since C_sys
        # are the same
        if (self.i_LB_mode=='global'):
            C_sys_mux  = sum(self.m_MUX_cap * self.md_mux_config_new)               # C_sys 
            Qmux_mat = self.f_form_QsrvMat(C_sys_mux,self.i_MUX_num,self.m_MUX_cap)
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # inite self.md_QtildaSgw
        self.md_QtildaMux = np.zeros((self.m_RtNum, self.m_RtNum)) # RxR
        for enb_idx in range(self.m_EnbNum):
            if (self.i_LB_mode=='global'): 
                # diag(self.m_EnbSgwMat(enb_idx,:)) * Qmux_mat # B*Q MxM
                Q_enb_mat = np.dot(np.diag(self.m_EnbMuxMat[enb_idx,:]), Qmux_mat) # MxM
            elif (self.i_LB_mode=='user'):
                # use user specific C_sys,j to calculate Q^srv,j matrix
                Qmux_mat = self.f_form_QsrvMat(self.m_userMuxSysCap[enb_idx],self.i_MUX_num,self.m_MUX_cap) # MxM
                B_diag_j = np.diag(self.m_EnbMuxMat[enb_idx,:])               # MxM
                Q_enb_mat = np.dot(np.dot(B_diag_j, Qmux_mat), B_diag_j) # B*Q*B,MxM
            Q_enb_sq  = np.dot(Q_enb_mat.T, Q_enb_mat) # (MxM)^T * MxM = MxM
            # get A(M)^T B_j Q^2 A(M)
            QtildaMuxEnb = np.dot(np.dot(self.m_MuxRouteMat.T, Q_enb_sq), self.m_MuxRouteMat) # RxR
            # multiplied by weight_sgw
            QtildaMuxEnb = QtildaMuxEnb * self.m_WeightMux[enb_idx]
            self.md_QtildaMux = self.md_QtildaMux + QtildaMuxEnb                                    
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
        # vaiance weight: 1/(M*S^hat)
        var_w = 1 / (self.md_new_MuxNum * self.md_var_norm)
        self.md_QtildaMux = var_w * self.md_QtildaMux  # RxR  
        self.md_Qtilda    = self.md_Qtilda + 2*self.md_QtildaMux 
        print self.i_LB_mode  
        print 'for journal 2', self.md_QtildaMux                                  
        
        
        
        
    def f_form_mux_delay(self,d_route):  
        if not self.m_lbMux_en:
            delay_mux = 0
            return delay_mux        
        # get MUX throughput
        mux_traffic = np.dot(d_route, self.m_MuxRouteMat.T) # 1xR * MxR'=1xM
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
    def f_getjoint_up(self):
        '''get upper bound traffic per route'''
        # 1/ get cap upper bound
        # bsxfun(@times,self.m_RES_cap',self.i_ResRouteMat) # NxR in matlab
        res_cap_map = self.i_ResRouteMat * self.m_RES_cap.flatten()[:,None] # NxR b* Nx1 (python index) --> NxR
#             up = max(res_cap_map) # NxR --> 1xR 
        # musk 0 entity with a big value    
        dump0= max(self.i_traffic_in) * (res_cap_map==0).astype(float)
        cap_up = dump0 + res_cap_map            
        cap_up_vect = np.min(cap_up,0) # NxR --> 1xR,0:first dim
        # 2/ get input traffic upper bound
        # bsxfun(@times,self.i_traffic_in',self.m_EnbRouteMat) # JxR
        traffic_cap_map = self.m_EnbRouteMat * self.i_traffic_in.flatten()[:,None] # JxR b* Jx1 (python index) --> JxR
        traffic_cap_vect = np.sum(traffic_cap_map,0) # JxR --> 1xR, 0: over first dim            
        joint_up = np.minimum(cap_up_vect, traffic_cap_vect)
#        self.m_debug.joint_up = joint_up
#             print 'cap_up_vect'
#             print 'traffic_cap_vect'
#             print 'joint_up'
        return joint_up  # 1xR      
    def f_getx0(self,upbound):
        # bsxfun(@times,upbound,self.m_EnbRouteMat) # JxR
        effect_cap_map = self.m_EnbRouteMat * upbound # JxR b* 1xR (python index) --> JxR
        effect_cap     = np.sum(effect_cap_map,1)[:,None] # JxR --> Jx1 (np.sum always returns a row vetor so need [:,None] for column
        traffic_capWeighted = self.i_traffic_in.flatten()[:,None] / effect_cap # Jx1
        # bsxfun(@times,traffic_capWeighted,effect_cap_map) # JxR
        x0_mat = effect_cap_map * traffic_capWeighted # JxR b* Jx1 (python index) --> JxR        
        x0_w   = np.sum(x0_mat,0)                     # sum over J --> 1xR
        #print x0_mat, x0_w            
        return x0_w   

    def f_adjust_nearZero(self):
        # remove very small negative number
        self.m_rate_update = np.maximum(self.m_rate_update,0)
        mask = (self.m_rate_update > self.mc_minRate).astype(float)
        self.m_rate_update = self.m_rate_update * mask   
        
    def f_update_results(self):
        self.f_update_general_results()
        self.f_update_sgw_results()
        self.f_update_mux_results()
    def f_update_general_results(self):
        #disp(size(self.m_rate_update))disp(size(self.m_EnbRouteMat))
        # bsxfun(@times,self.m_rate_update,self.m_EnbRouteMat)
        #print np.size(self.m_rate_update)
        self.o_traffic = self.m_EnbRouteMat * self.m_rate_update.flatten() # JxR b* Rx1 (python index) --> JxR 
        # bsxfun(@times,self.o_traffic,self.i_traffic_in')
        self.o_Amatrix = self.o_traffic / self.i_traffic_in.flatten()[:,None] # JxR b/ Jx1 (python index) --> JxR  
        #print self.o_Amatrix
        self.o_ratioList =[]
        for j in range(self.m_EnbNum):
            userRatios = list(self.o_Amatrix[j,np.where(self.m_EnbRouteMat[j,:]>0)].flatten())
            self.o_ratioList.append(userRatios)
        #print self.o_ratioList    
        # Calculate load update per enabled RES            
        for n in range(self.m_ResNum):
#             print self.i_ResRouteMat[n,:]
#             print np.size(self.md_res_config_new[n])
#             print np.shape(self.m_rate_update)
            self.m_load_update[n] = sum(self.i_ResRouteMat[n,:]
                                        * self.md_res_config_new[n]
                                        * self.m_rate_update)                                
        # get RES load,load ratio and status
        self.o_ResLoad = self.m_load_update
        #self.o_RES_status = self.m_load_update > self.mc_reOnOff_th 
        self.o_ResLoadRatio = self.o_ResLoad / self.i_RES_cap
        self.o_RES_status = (self.o_ResLoadRatio > self.mc_reOnOff_th).astype(float)
        # calculate RES incremental power
        self.o_PowConsum1 = sum(self.m_RES_peff * (self.o_ResLoad))
        # calculate RES p0
        ipow_sum = sum(self.o_RES_status * (self.m_RES_p0))
        self.o_PowConsum2 = ipow_sum+self.o_PowConsum1
        # calculate power term per ENB and weighted sum
        self.f_update_pow_approx()
    def f_update_pow_approx(self):
        P_max = self.md_pnorm / self.m_normScale #   sum(self.md_max_pow)
        for enb_idx in range(self.m_EnbNum):
            traffic_enb = self.m_rate_update * self.m_EnbRouteMat[enb_idx,:] #1xR
            resTraffic_enb = np.dot(traffic_enb, self.m_ResRouteMat.T) #1xR * NxR^T: 1xN
            # \epsinon / P^max
            weight_enb  = self.m_RES_peff_approx / P_max # 1xN
            #print np.size(self.m_RES_peff_approx), (weight_enb), (resTraffic_enb),enb_idx
            self.o_powRes_enb[enb_idx]  = np.dot(weight_enb, resTraffic_enb) # 1xN * 1xN^T: 1x1         
        self.o_powSum_norm = np.dot(self.m_WeightPow, self.o_powRes_enb)  # inner product            
    def f_update_sgw_results(self):                       
        # get SGW load,load ratio and status
        self.o_SgwLoad = self.m_load_update[self.m_sgw_start:self.m_mux_start] #-1)            
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
            # get A(I)^T B_j Q^2 A(I)
            QtildaSgwEnb = np.dot(np.dot(self.m_SgwRouteMat.T, Q_enb_sq), self.m_SgwRouteMat) # RxR
            self.o_SgwLBVarEnb[enb_idx] = var_w * np.dot(np.dot(self.m_rate_update, QtildaSgwEnb), self.m_rate_update)
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
    def f_update_mux_results(self):
        # get MUX load,load ratio and status
        if self.i_MUX_num > 0 and self.m_lbMux_en:
            self.o_MuxLoad = self.m_load_update[self.m_mux_start:self.m_link_start] # -1)            
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
                           %(j+1,self.m_WeightPow[j],self.m_WeightSgw[j],self.m_WeightMux[j]))
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
            fid.write('eNodB%d,' %(j+1))
            # print out traffic allocation
            for r in range(self.m_RtNum):
                fid.write('%4.5f,' %self.o_traffic[j,r])
#                 fid.write('$,')
#                 % print out traffic matrix
#                 for j=1:self.m_Ndim
#                     fid.write('%4.5f,',self.o_Amatrix(i,j))
#                 end
            fid.write('\n')
        # print out sum per BS
        fid.write('sum,')
        for r in range(self.m_RtNum):
            #print 'self.m_rate_update[r]=',self.m_rate_update[r]
            fid.write('%4.5f,' %(self.m_rate_update[r]))
#             fid.write('$,')
#             for j=1:self.m_Ndim
#                 fid.write('%4.5f,',sum(self.o_Amatrix(:,j)))
#             end
        fid.write('\n')            
        fid.write('SgwIndex,load,load ratio,load status, price, slop, load ratio STD \n')
        for i in range(self.m_SgwNum):
            fid.write('SGW%d, %2.5f, %2.5f, %d,%2.5f,%2.5f,%2.5f\n' %(i+1,self.o_SgwLoad[i],
                    self.o_SgwLoadRatio[i],self.o_SGW_status[i],
                    999,
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
#         print self.o_powSum_norm,self.o_SgwLBVarSum,self.o_DelaySum
        fid.write('%2.5f, %2.5f, %2.5f, %2.5f, %2.5f, %2.5f, %2.5f \n' 
                  %(self.o_PowConsum1,self.o_PowConsum2, self.o_fval, self.f_calculate_min_value(),
                    self.o_powSum_norm,self.o_SgwLBVarSum,self.o_DelaySum))
        fid.close()
    def f_calculate_min_value(self):
        min_value_pow = np.dot(self.m_fvec, self.m_rate_update) # inner product of 2 vectors
        min_value_lb_sgw = np.dot(np.dot(self.m_rate_update, self.md_QtildaSgw), self.m_rate_update)
        min_delay_mux = self.f_form_mux_delay(self.m_rate_update)
        #disp(min_value_pow);disp(min_value_lb_sgw);disp(min_delay_mux);
        eff_min = min_value_pow + min_value_lb_sgw + min_delay_mux
        return eff_min
    
class userSolverTemplate(object):
    def __init__(self,j,D,C,I,H,L,RM,EM,E,pnorm,R_sgw,D_mux,V_norm,sdiag,
                 Weight,numbs,BsAgg_step):
        self.i_SGW_num = I
        self.i_MUX_num = H
        self.i_link_num= L
        self.i_enb_idx=j
        self.i_ResRouteMat=RM
        self.i_EnbRouteMat=EM
        self.i_traffic_in=D
        self.i_RES_peff=E
        self.id_RES_pnorm = pnorm          # RES normalised power cosumption dim: 1
        self.id_load_ratio_sgw = R_sgw    # expected load ratio R=T/S^sgw_sys or T_j/S^srv_sys,j 
        self.id_MUX_ntime = D_mux         # MUX normalised time delay,dim: 1xM
        self.id_sgwVar_norm= V_norm       # SGW variance norming factor   
        self.i_sdiag = sdiag
        self.i_RES_cap=C
        self.i_WeightPow=Weight[0] 
        self.i_WeightSgw=Weight[1] 
        self.i_WeightMux=Weight[2] 
        self.io_numbs=numbs             # number of active BS,used for scaling but may not be needed
        self.io_BsAgg_step = BsAgg_step # BS aggregated traffic gradient step size: \alpha_{\lambda}
        self._f_init_m()
        self.f_check()
    def _f_init_m(self):
        # Optinal inputs   
        self.io_BsAllo_step = None   # BS traffic rate allocation gradient step size: \alpha_{d}
        self.io_max_iteration = None
        # dynamic input
          #innter loop
        self.id_sgw_loads = None       # dynamic loads over all S-GWs dim: I
        self.id_mux_loads = None       # dynamic loads over all MUXs dim: M 
        self.id_price_load = None   # lagragian multiplier for RES n regarding capacity constrain \sum_r A_{n,r}d_r <= S_n
        self._id_res_loads = None       # dim: N
          #outter loop  
#         id_load_ratio_mux   # expected load ratio R=T/S^mux_sys !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! to be removed                             
        self.id_SGW_status = None   # Renewed SGW status 1--> en, 0-->dis dim: 1xI
        self.id_RES_status = None   # updated RES status 1--> en, 0-->dis dim: 1xN  
        self.id_ResRouteMat = None    # dynamic active RES-Route map
        self.id_iteration  = None
#         i_LinkGr_step   # Link constrain gradient step size: \alpha_{\mu}
        # class member
        self.m_SgwNum = None        # Initial number of SGWs as I 
        self.m_ResNum = None        # Initial number of RESs as N 
        self.m_BsAgg_step = None    # actual BS aggregated traffic gradient step size: \alpha_{\lambda}        
        self.m_BsAgg_price = None   # BS aggregated traffic Lagragian multiplier  
        self.m_rescap_price = None   # gradient for RES capacity over d_r
        self.m_init_allo = None     # Initial traffic allocation with static LB: d_r=D_j/ActiveRtNum,dim: 1 * R
        self.m_pre_allo = None      # Rate allocation of the previous literation,dim: 1 * R   
        self.m_enbRt_vect = None     # eNodeB j route map vecter,dim: 1 * R
        self.m_enbRt_num = None     # total route number
        self.m_enbAtRt_num = None   # active route number
        self.m_enbRes_vect = None   # eNodeB-RES map vector,dim: 1 * N
        self.m_enbSgw_vect = None   # eNodeB-SGW map vector,dim: 1 * I
        self.m_enbMux_vect = None   # eNodeB-MUX map vector,dim: 1 * M
        self.m_BsAllo_step = None   # actual BS rate allocation step size: \alpha_{d}
        self.m_sgw_start = None     # SGW start postion
        self.m_mux_start = None     # MUX start postion
        self.m_link_start = None    # Link start postion 
        self.m_RES_peff = None      # dynamic RES peff
        self.m_lbMux_en = None      # MUX LB enabler
        self.m_SGW_cap = None       # SGW capacity
        self.m_MUX_cap = None       # MUX capacity
        self.m_sgwVarNorm_en = None    # LB var norm enable
        self.m_obj_gradient = None  # gradient without lag or pen
        self.m_gradient = None      # completed gradient
        self.m_userSgwSysCap = None      
        self.m_normScale = 1        # constant for level normalisation factors for POW,LB and delay (0.08,0.05,0.02  
        ###############penalty related############        
        self.m_pen_eq_weight  =3.0  #best is 1.3
        self.m_pen_ieq_weight =5.0
          # dynamic member
        self.md_RES_cap = None
        self.md_SgwNum = None         # dynamic active number of SGWs
        self.md_ResNum = None         # dynamic active number of RES
        self.md_MuxNum = None         # dynamic active number of MUXs        
        self.md_SgwRouteMat = None    # dynamic active SGW-Route map
        self.md_MuxRouteMat = None    # dynamic active MUX-Route map, dim:MxR
        self.md_MuxRouteMat_j = None  # dynamic active MUX-Route map for this user, dim:MxR
        self.md_NumRoutePerMux_j = None # number of routes for user j per MUX, dim: 1xM
        self.md_MUX_status = None   # updated RES status 1--> en, 0-->dis dim: 1xM  
        self.md_QtildaSgw  = None   # weighted quadratic matrix for this user dim: R_user x R_user
        self.md_Qtilda     = None   # self.md_QtildaSgw * 2 used for solver  
          # constant
#         mc_bs_agg_gradient=2.0 # constant for BS agg traffic gradient calculation for fast converge but may not at all  
#         mc_bs_agg_gradient=0.5 # constant for BS agg traffic gradient calculation
        self.mc_bs_agg_gradient=0.1 # constant for BS agg traffic gradient calculation for pen
#         mc_link_gradient  =0.1 # constant for link gradient calculation
        self.mc_bs_allo_gradient=  0.1 # 0.05  #  0.2  # 
        self.mc_reOnOff_th= 1.0e-4  # RES on/off threshold for traffic   
        self.mc_minRate   = 1.0e-10  
          # optional member
        self.m_bs_scale = None      # tunning para for /N
        self.m_linkCon_en = None    # logic for link constrain enable
          # debug purpose
        self.m_debug = None             
        # output
        self.o_traffic = []       # output of traffic allocation vector over routes, 1xR
        self.o_Amatrix = []       # output allocation ratio matrix, 1xR      
    def f_check(self):
        [J,self.m_enbRt_num] = np.shape(self.i_EnbRouteMat)
        if (self.i_enb_idx >= J or self.i_enb_idx < 0) :
            raise Exception('Input eNodeB index is out of range')
        self.m_SgwNum = self.i_SGW_num          
        self.m_ResNum = self.i_SGW_num+self.i_MUX_num+self.i_link_num
        self.md_SgwNum = self.i_SGW_num
        self.md_MuxNum = self.i_MUX_num
        [N_res,R] = np.shape(self.i_ResRouteMat)
        if (N_res != len(self.i_RES_peff) or N_res != self.m_ResNum):
            raise Exception('Input RES dim inconsistent')
        if R != self.m_enbRt_num :
            raise Exception('route dim mismatch')
        if self.i_link_num > 0 :
            self.m_linkCon_en=1
        else:
            self.m_linkCon_en=0
        # need put more check
            
        # Form default paras
        self.f_init_default()            
    def f_init_default(self):
        # Initial status
        self.f_init_status()
        # Calculate BS aggregation traffic gradient step and init price
#        self.f_init_user_agg_gradient()
        # Derive route map related paras
        self.f_init_routeMap()
        # Calculate default rate allocation with LB (Proposional fair)
        self.f_init_rate()
        # Init optional paras
        self.f_init_optional()
        # Init output array
        self.f_init_output()      
    def f_init_status(self):
        # Set initial SGW status
        self.id_SGW_status = np.ones((self.i_SGW_num,)) 
        # Set initial RES status
        self.id_RES_status = np.ones((self.m_ResNum,)) 
        # Set initial MUX status
        self.md_MUX_status = np.ones((self.i_MUX_num,))  
    def f_init_user_agg_gradient(self): 
        # get BS aggregated traffic gradient step size
        if not(self.io_BsAgg_step):
#                 self.m_BsAgg_step = 2./self.i_traffic_in
            if self.i_traffic_in > 0:
#                     self.m_BsAgg_step = self.mc_bs_agg_gradient./self.md_SgwNum./self.i_traffic_in
                self.m_BsAgg_step = self.mc_bs_agg_gradient / self.md_SgwNum * self.i_traffic_in
            else:
                self.m_BsAgg_step = 0
        else: # currently from top !!!!!!!!!!!!!!!!!!!!!!!!!
            self.m_BsAgg_step = self.io_BsAgg_step
        # Initial BS aggregated traffic Lagragian multiplier
        self.m_BsAgg_price = self.m_BsAgg_step  
        self.m_BsAgg_price = 0
    def f_init_routeMap(self): #may need dynamic setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # Get eNodeB route mapping
        self.m_enbRt_vect = self.i_EnbRouteMat[self.i_enb_idx,:] # dim:R
        # Get active route number for eNode j
        self.m_enbAtRt_num = sum(self.m_enbRt_vect)            
        # Derive RES paras
        self.m_RES_peff  = self.i_RES_peff                
        # get start postion in res-route map
        self.m_sgw_start = 0 # 1 in matlab
        self.m_mux_start = self.m_SgwNum # +1
        self.m_link_start = self.m_mux_start+self.i_MUX_num            
        # Get dynamic RES-route map
        self.id_ResRouteMat = self.i_ResRouteMat            
        # init VAR norm
        if (self.id_sgwVar_norm):
            self.m_sgwVarNorm_en = 1
        # init SGW, MUX dynamic enable matrices, vectors
        self.f_init_MapEn()         
    def f_init_MapEn(self): #may need dynamic setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # Derive RES enable for this eNodeB
        self.m_enbRes_vect = (np.dot(self.id_ResRouteMat, self.m_enbRt_vect)>0).astype(int) # (NxR b* R)-->N
        # get active RES cap
        active_res_vect = (np.sum(self.id_ResRouteMat,1)>0).astype(int) # dim: N
        self.md_RES_cap = active_res_vect * self.i_RES_cap  # dim: N             
        # Derive SGW paras
        sgw_start = self.m_sgw_start
        sgw_end   = self.m_mux_start #-1
        self.m_SGW_cap   = self.md_RES_cap[sgw_start:sgw_end]
        # get SGW-route map
        self.md_SgwRouteMat = self.id_ResRouteMat[self.m_sgw_start:self.m_mux_start,:] #IxR
        # get eNodeB-SGW map vector
        self.m_enbSgw_vect = (np.dot(self.md_SgwRouteMat, self.m_enbRt_vect)>0).astype(int) # (IxR b* R)-->I             
        # init MUX LB enable
        if (self.id_sgwVar_norm):
            self.m_lbMux_en = 1
            # get active MUX-route from active RES-route mat
            self.md_MuxRouteMat = self.id_ResRouteMat[self.m_mux_start:self.m_link_start,:]
            # get eNodeB-MUX map vector
            self.m_enbMux_vect = (np.dot(self.md_MuxRouteMat, self.m_enbRt_vect)>0).astype(int) # (MxR b* R)-->M
            self.md_MuxNum = sum(self.m_enbMux_vect)
            # get this user's MUX-route from active MUX-route and eNodeB route mapping
            self.md_MuxRouteMat_j = np.dot(self.md_MuxRouteMat, np.diag(self.m_enbRt_vect)) # (MxR * RxR)--> MxR
            # get number of routes for j user per MUX
            self.md_NumRoutePerMux_j =  np.sum(self.md_MuxRouteMat_j,1) # (sum(MxR,1)-->M)
#                 print(self.md_NumRoutePerMux_j)
            # Derive MUX paras
            if self.i_MUX_num > 0: 
                mux_start = self.m_mux_start
                mux_end   = self.m_link_start #-1
                self.m_MUX_cap   = self.md_RES_cap[mux_start:mux_end]         
    def f_init_rate(self):           
        # get initial evenly distributed allocation
        self.m_init_allo = self.i_traffic_in / self.m_enbAtRt_num * self.m_enbRt_vect
        # Assign to internal buffer for next literation
        self.m_pre_allo  = self.m_init_allo
        # get BS rate allocation step size for gradient method
        if not(self.io_BsAllo_step):
            max_allo = max(self.m_pre_allo)
            if max_allo > 0:
#                     self.m_BsAllo_step = 2.0.*self.m_enbAtRt_num./max_allo
#                     self.m_BsAllo_step = 0.1.*self.m_enbAtRt_num./max_allo
#                     self.m_BsAllo_step = 10 ./ self.md_SgwNum ./ self.i_traffic_in
#                     self.m_BsAllo_step = self.mc_bs_allo_gradient ./ self.md_SgwNum .* self.i_traffic_in
                scale = self.m_enbAtRt_num / self.m_enbRt_num
                self.m_BsAllo_step = scale * self.mc_bs_allo_gradient / self.md_SgwNum * self.i_traffic_in
                if (self.i_WeightSgw==1.0 and self.id_sgwVar_norm < 1.0):
                    self.m_BsAllo_step = self.m_BsAllo_step * self.id_sgwVar_norm 
#                     if self.i_WeightSgw==0
#                         self.m_BsAllo_step = 0.0001 .* self.mc_bs_allo_gradient                       
                else:
                    self.m_BsAllo_step = 0 
            else:
                self.m_BsAllo_step = self.io_BsAllo_step  
    def f_init_optional(self):
        if not(self.io_numbs):
            self.m_bs_scale  =  1.0   #    
        else:
            self.m_bs_scale  =  self.io_numbs   #                  
    def f_init_output(self):
        # form output rate allocation array
        self.o_traffic = np.zeros((self.m_enbRt_num,)) # dim: R
        self.o_Amatrix = np.zeros((self.m_enbRt_num,)) # dim: R   

    def f_getjoint_up(self):
        '''get upper bound traffic per route'''
        # 1/ get cap upper bound
        # bsxfun(@times,self.m_RES_cap',self.i_ResRouteMat) # NxR in matlab
        res_cap_map = self.id_ResRouteMat * self.md_RES_cap.flatten()[:,None] # NxR b* Nx1 (python index) --> NxR
        # musk 0 entity with a big value    
#         print np.shape(self.id_ResRouteMat),np.shape(res_cap_map),np.shape(self.md_RES_cap)
        dump0= self.i_traffic_in * (res_cap_map==0).astype(float)
        cap_up = dump0 + res_cap_map            
        cap_up_vect = np.min(cap_up,0) # NxR --> 1xR,0:first dim
        # 2/ get input traffic upper bound
        traffic_cap_vect = self.m_enbRt_vect * self.i_traffic_in # 1xR b* 1  --> 1xR         
        joint_up = np.minimum(cap_up_vect, traffic_cap_vect)
        return joint_up  # 1xR 
    
    def f_getx0(self,upbound):
        effect_cap     = self.m_enbRt_vect * upbound # 1xR b* 1xR (python index) --> 1xR
        effect_cap_sum = sum(effect_cap) # 1x1
        weight = effect_cap / effect_cap_sum # 1xR      
        x0_w   = weight *  self.i_traffic_in # 1x1            
        return x0_w   
            
    def f_form_QsrvMat(self,C_sys_in,re_num,re_cap):
        # get active system capacity based on user/global mode
        C_sys  = C_sys_in               # C_sys
#             Q_mat  = -(1.0/C_sys)*ones(self.md_new_SgwNum,self.md_new_SgwNum)  # -1/C_sys
        #Q_mat  = -(1.0/C_sys) * np.ones((self.i_SGW_num,self.i_SGW_num))  # -1/C_sys, No need for dynamic SGW number
        Q_mat  = -(1.0/C_sys) * np.ones((re_num,re_num))  # -1/C_sys, No need for dynamic SGW number
        #t_diag = np.diag(1 / self.m_SGW_cap + self.m_sdiag)                    # diag(1/c_i+\tau)
        t_diag = np.diag(1 / re_cap + self.m_sdiag)                    # diag(1/c_i+\tau)
        Q_mat  = Q_mat  + t_diag 
        return Q_mat                           

    def f_adjust(self):
        if self.i_traffic_in <= 0:
            self.f_init_output()
            return
        # calculate scale
        scale = self.i_traffic_in / sum(self.m_pre_allo)
        print 'BS-%d: scale is %2.5f ' %(self.i_enb_idx, scale)
        self.m_pre_allo = self.m_pre_allo * scale
        # adjust for w=0 when RES capacity is violated
        self.f_adjustW()
        # derive output
        self.o_traffic   = self.m_pre_allo
        self.o_Amatrix   = self.o_traffic / self.i_traffic_in        
    def f_adjustW(self):
        if self.i_WeightSgw > 0:
            # no need to readjust
            return
        #print(self.m_pre_allo)
        # get min capacity per route
        big_num = max(self.md_RES_cap)+1
        res_cap_r = np.dot(np.diag(self.md_RES_cap),self.i_ResRouteMat) # NxN * NxR --> NxR
        nonezero_mat = big_num * (res_cap_r==0).astype(int)
        res_cap_r = res_cap_r + nonezero_mat # dim: NxR
        #print(res_cap_r)
        res_cap_r_enb = np.dot(res_cap_r, np.diag(self.m_enbRt_vect)) # NxR * RxR --> NxR
        #print(res_cap_r_enb)
        min_cap_r_enb = np.min(res_cap_r_enb,axis=0)
        #print(min_cap_r_enb)
        
        while sum((self.m_pre_allo > min_cap_r_enb).astype(int))>0:
            # find route with overloaded traffic and the extra traffic
            tem_allo = np.minimum(min_cap_r_enb,self.m_pre_allo)
            dif_allo = self.m_pre_allo - tem_allo
            # find route unsaturated and the corresponding ratio
            unadjust_vect = self.m_enbRt_vect - (dif_allo>0).astype(int)
            unadjust_allo = unadjust_vect * self.m_pre_allo
            unadjust_ratio= unadjust_allo / sum(unadjust_allo)
            # distribute the extra traffic on unsaturated routes by ratio
            sum_dif_allo = sum(dif_allo)
            distribut_allo= unadjust_ratio * sum_dif_allo
            # clip the overloaded and fill to unstaturated
            #print(dif_allo) print(distribut_allo)
            self.m_pre_allo = self.m_pre_allo - dif_allo + distribut_allo
        
