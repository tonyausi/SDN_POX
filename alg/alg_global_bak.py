#!/usr/bin/python

'''
Cloud project:
- Software Defined Networking (SDN) 
-- project related algorithm (global solution)

Tony Lin
'''
import re
import numpy as np
from scipy.optimize import minimize

class projSolver(object):
    def __init__(self,D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,LB_mode='user',normScale=None,
                 plot_en=False,srecord_en=False):    
        '''input'''
        self.i_traffic_in=np.array(D)       # input traffic vector dim: J
        self.i_RES_cap   =np.array(C)       # resource capability dim: N=I+H+L
        self.i_RES_maxpow=np.array(PMAX)/1000000    # resource peak power dim: N   # PMAX./1000000
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
        self.i_outloop_en=Outloop
        self.io_method_mode = GameMode      # [] for GR for gradient mode, BR for best response mode 
        self.i_RES_ratio = RES_ratio
        self.i_constraint_mode = constraint_mode
        self.i_LB_mode = LB_mode
        self.i_normScale = normScale
                
        self.m_plot_en=plot_en
        self.m_srecord_en=srecord_en # Enable single results recording 
        '''init class member'''
        self.f_init_m()
        # call member def
        self.f_check()
        self.f_init()
        self.f_simulate()
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
        self.m_iteration=None     # number of iteration ??????????????????????????????????
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
        self.md_res_config_prev  # vector of pre SGW configuration 1-->en, 0-->disable
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
                raise('Input i_RES_cap or i_RES_maxpow or i_RES_idleScale dim inconsistent')
        #check enb-route map and res-route map
        [self.m_EnbNum, self.m_RtNum]=np.shape(self.i_EnbRouteMat)
        t=np.shape(self.i_ResRouteMat)
        if t[0]!=self.m_ResNum:
            raise('Number of resource mismatch for resource-route map')
        if t[1]!=self.m_RtNum:
            raise('Number of routes mismatch for resource-route map')
        # check weighting factor
        w=np.shape(self.i_Weight)
        if (w[0]==1) and (w(2)==1):
            if self.i_Weight>1 or self.i_Weight<0:
                raise('Incorrect range for weighting factor')
        elif (w[0]!=self.m_EnbNum) or (w[1]!=3):
            raise('Input i_Weight dim mismatch number of user')
        # check loop control
        if not (self.i_outloop_en==0 or self.i_outloop_en==1):
            print 'self.i_outloop_en=',self.i_outloop_en
            raise('Out loop control should be 1 or 0')
        # check RES util ratio para
        if not(self.i_RES_ratio):
                self.m_res_appox_en=0
        else:
            self.m_res_appox_en=1
            if len(self.i_RES_ratio)!= len(self.i_RES_idleScale):
                raise('Input i_RES_ratio dim mismatch number of res')
        # check constraint mode
        if not(self.i_constraint_mode):
            self.i_constraint_mode = 'lag' # Lagrangian
        elif (self.i_constraint_mode!='lag'
              and self.i_constraint_mode!='pen'):
            raise('Input i_constraint_mode not supported yet')
        print'constraint mode is %s \n' %self.i_constraint_mode
        # parse route map for RES and BS
        self.f_pars_routeMap()
        # check sgw, mux and links against system throughput
        self.m_throughput = sum(self.i_traffic_in)
        if sum(self.m_SGW_cap) <= self.m_throughput:
            raise('SGW total capacity is less than the system input')
        if sum(self.m_MUX_cap) <= self.m_throughput:
            raise('MUX total capacity is less than the system input')
        if sum(self.m_Link_cap) <= self.m_throughput:
            raise('Link total capacity is less than the system input')
        # check i_LB_mode
        if not(self.i_LB_mode):
            self.i_LB_mode = 'global'
        elif (self.i_LB_mode!='global'
              and self.i_LB_mode!='user'):
            raise('Only support global or user mode for i_LB_mode')
        # check i_normScale
        if (self.i_normScale):
            if self.i_normScale <= 0:
                raise('i_normScale should be a positive numer')
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
    def f_init(self)
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
    def f_init_set(self)   
        # Initial rate update
        self.m_rate_update = np.zeros((1,self.m_RtNum)) # (1,self.m_RtNum)
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
        if not(self.i_sdiag):
            self.m_sdiag = 0.001 / max(1,sum(self.i_traffic_in))
        else:
            self.m_sdiag = self.i_sdiag    
    def f_init_weight(self)
        w=np.shape(self.i_Weight)
        t1 = np.ones((1,self.m_EnbNum))
        if (w[0]==1) and (w[1]==1):
            self.m_WeightPow = (1 - self.i_Weight) * t1
            self.m_WeightSgw = self.i_Weight / 2 * t1
            self.m_WeightMux = self.i_Weight / 2 * t1
        elif (w(1)==self.m_EnbNum) and (w(2)==3):
            self.m_WeightPow = self.i_Weight(:,0)
            self.m_WeightSgw = self.i_Weight(:,1)
            self.m_WeightMux = self.i_Weight(:,2)       
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
            if self.m_WeightPow < (1 - 0.001):                    
                self.md_pnorm = sum(self.md_max_pow * pow_scale) 
            else:
                if self.io_scale_mode ==1:
                    p_weight = 0.1.*sum(self.i_traffic_in)./self.m_EnbNum
#                     p_weight = p_weight + 0.02
                else:                        
                    p_weight = 1.0
                    #print p_weight 
                self.md_pnorm = p_weight*sum(self.md_max_pow)*pow_scale 
        elif self.io_norm_mode=='max':
            self.md_pnorm = self.i_RES_maxpow
        else:
            raise('Incorrect SGW normalisation mode')  
        # scale norm weights to proper level
        self.md_pnorm = self.md_pnorm * self.m_normScale                              
    def f_init_lb_norm(self):            
        m_new_SGW_cap = self.m_RES_cap(self.m_sgw_start : self.m_mux_start)
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
            m_new_MUX_cap = self.m_RES_cap(self.m_mux_start : self.m_link_start)
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

    def f_simulate(self):
        # always run first inner iteration
        self.f_inner_simu()
        # stop further loop if not required
        if self.i_outloop_en!=1:
            return
        # check change of active sgw
        self.f_check_active_SgwRes()
        #if self.md_dif_sgw > 0 
        while self.md_dif > 0:                
            self.f_reconfig()
            self.f_inner_simu()
            # check change of active sgw
            self.f_check_active_SgwRes()
    def f_inner_simu(self):
        self.f_form_fmin()
        self.f_min_cal()
        self.f_update_results()            
        self.f_record_results()
        self.f_plot_results()
        self.f_evaluate_magnitude()

    def f_form_fmin(self):
        # form A,B matrices
        self.f_form_active_map()
        # form system normalised incremental power
        self.f_form_fmin_syspow()
        # form SGW load variance term
        self.f_form_fmin_lb_sgw()  
        self.f_form_eq_pen()        
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
        ActiveSgwRtVect = (sum(self.m_SgwRouteMat)>0).astype(float)            # 1xR
            # get active route path from active link & MUX
        MuxLinkRouteMat = self.m_ResRouteMat[self.m_mux_start:self.m_ResNum,:] #(ML)xR
        ActiveMuxLinkRtVect = (sum(MuxLinkRouteMat)>0).astype(float)           # 1xR
            # get upated active route: Dim: 1 x R
        ActiveRtVect = ActiveSgwRtVect * ActiveMuxLinkRtVect                   # 1xR
            # get B(t) as Aeq, DIM: JxR
            #bsxfun(@times,ActiveRtVect,self.i_EnbRouteMat) in matlab
        self.m_EnbRouteMat = self.i_EnbRouteMat.T * ActiveRtVect # JxR b* 1xR -->JxR (python index)                        
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
                Q_enb_mat = np.diag(np.diag(B_diag_j, Qsrv_mat), B_diag_j) # B*Q*B,IxI
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
    def f_form_eq_pen(self):
        if (self.i_constraint_mode!='pen'):
            return
        # derive equality contraint related para for
        # \lambda.*(d^in-d*B^rt)*(d^in-d*B^rt)
        # =\lambda.*(d^in*d^in'-2d^in*B^rt*d'+d*B^rt'*B^rt*d')
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # get \lambda.*B^rt'*B^rt
        self.m_Bsquare_pen  = self.m_pen_eq_weight * np.dot(self.m_EnbRouteMat.T, self.m_EnbRouteMat) # RxR
        # get -2\lambda.*d^in*B^rt'
        self.m_fvec_eq_pen = -2. * self.m_pen_eq_weight * np.dot(self.i_traffic_in,self.m_EnbRouteMat) # 1xR
        # get \lambda.*d^in*d^in'                
        self.m_const_pen   = self.m_pen_eq_weight * np.dot(self.i_traffic_in,self.i_traffic_in.T) # 1x1   
        #print 'calc eq pen'                    
    def f_min_cal(self):
        lp = np.zeros((self.m_RtNum,))
        # get actual upper bound
#             up = max(self.m_RES_cap) * np.ones(self.m_RtNum,1)
        joint_up = self.f_getjoint_up()
        up = joint_up + 1
        '''        
        f_bound=lambda x,y: [[x[i],y[i]] for i in range(len(x))]
        # get initial value based on upper bound
        x0_w = self.f_getx0(joint_up) #disp(x0_w)
        #T = self.m_throughput
        #x0 = T./self.m_RtNum.*ones(1,self.m_RtNum) # Make a starting guess at the solution
        #x0 = 1 .* T./self.m_RtNum.*ones(1,self.m_RtNum)
        x0 = x0_w .* 0.7 #%%%%%%%%% magic number ?????????????

#        self.m_debug.x0 = x0                              
        if (self.i_constraint_mode=='lag'):
            A  = self.m_ResRouteMat
            b  = self.m_RES_cap                      
            Aeq= self.m_EnbRouteMat
            beq= self.i_traffic_in
            eqconst   = lambda x: np.dot(Aeq,x)-beq  # C(x)=0 in scipy.optimize.minimize
            ineqconst = lambda x: b-np.dot(A,x)      # C(x)>=0 in scipy.optimize.minimize
            cons = ({'type': 'eq',
                     'fun' : eqconst},
                    {'type': 'ineq',
                     'fun' : ineqconst}
                   )
        elif (self.i_constraint_mode=='pen'):
            cons = ()
        else:
            raise('i_constraint_mode mode not supported yet')pause keyboard
        md = 'SLSQP'
        op = {'disp': True} 
        '''
        # [self.m_rate_update,self.o_fval] = fmincon(@(x)f_objfun(self,x),x0,A,b,Aeq,beq,lp,up,[],options)
#        minRes = minimize(self.f_objfun, x0, bounds=f_bound(lp,up), constraints=cons,
#                          method=md,options=op)
        ''' data output '''
#        self.m_rate_update = np.array(minRes.x) 
        ''' min value for target function '''    
#        self.o_fval        = minRes.fun  
        ''' scaling back ''' 
        self.o_fval = self.o_fval * self.m_normScale
        ''' 'remove every tiny numbers (+ and -) '''
        self.f_adjust_nearZero()
        
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
        x0_w   = sum(x0_mat)            
        return x0_w
    def f_objfun(self,d_route):
        sys_pow = np.dot(self.m_fvec, d_route) # 1xR * 1xR^T=1x1
        sys_lb_sgw = np.dot(np.dot(d_route, self.md_QtildaSgw), d_route) #1xR * RxR * 1xR'=1x1
        sys_delay_mux = self.f_form_qp_lb_mux(self,d_route)
#             print sys_delay_mux
        f = sys_pow + sys_lb_sgw + sys_delay_mux
        if (self.i_constraint_mode=='pen'):
            sys_eq_pen = (np.dot(self.m_fvec_eq_pen, d_route)                      # 1xR d* 1xR --> 1x1
                           + np.dot(np.dot(d_route, self.m_Bsquare_pen), d_route)  # 1xR * RxR * Rx1
                           + self.m_const_pen)                       
            f = f + sys_eq_pen + f_calc_rescap_penalty(self,d_route)        
    def f_form_qp_lb_mux(self,d_route):  
        if not self.m_lbMux_en:
            delay_mux = 0
            return         
        # get MUX throughput
        mux_traffic = np.dot(d_route, self.m_MuxRouteMat) # 1xR * MxR'=1xM
        # get active MUX entity delay
        delay_mux_active = self.md_mux_config_new / (self.m_MUX_cap-mux_traffic) # 1xM
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
    def f_calc_rescap_penalty(self,d_route):
        # get d * A' - C --> 1xR * (NxR)'
        cap_dif   = np.dot(d_route, self.m_ResRouteMat.T) - self.m_RES_cap # 1xN
        cap_dif_sq= cap_dif**2 # 1xN
        res_cap_indic = (cap_dif > 0).astype(float) # 1xN
        self.m_rescap_pen = 0
        for enb_idx in range(self.m_EnbNum):
            # get B_j^res .* penalty_enable, dim: 1xN
            enb_res_en    = self.m_EnbResMat[enb_idx,:] * res_cap_indic # remove not j related RES and inactive penalty
            # get weighted and active penalty dif
            pen_dif = self.m_pen_ieq_weight * np.dot(enb_res_en, cap_dif_sq) # dim: 1xN * 1xN'
            # accumulate over j
            self.m_rescap_pen = self.m_rescap_pen + pen_dif
        rescap_pen = self.m_rescap_pen
        #print max(self.m_rescap_pen)
        #print res_cap_indic
        return rescap_pen    
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
        self.o_traffic = self.m_EnbRouteMat * self.m_rate_update.flatten()[:,None] # JxR b* Jx1 (python index) --> JxR 
        # bsxfun(@times,self.o_traffic,self.i_traffic_in')
        self.o_Amatrix = self.o_traffic / self.i_traffic_in.flatten()[:,None] # JxR b/ Jx1 (python index) --> JxR  
        # Calculate load update per enabled RES            
        for n in range(self.m_ResNum):
            self.m_load_update[n] = sum(self.i_ResRouteMat[n,:]
                                        * self.md_res_config_new(n)
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
        for enb_idx in range((self.m_EnbNum):
            traffic_enb = self.m_rate_update * self.m_EnbRouteMat[enb_idx,:] #1xR
            resTraffic_enb = np.dot(traffic_enb, self.m_ResRouteMat.T) #1xR * NxR^T: 1xN
            # \epsinon / P^max
            weight_enb  = self.m_RES_peff_approx / P_max # 1xN
            self.o_powRes_enb[enb_idx]  = np.dot(weight_enb, resTraffic_enb.T) # 1xN * 1xN^T: 1x1           
        self.o_powSum_norm = np.dot(self.m_WeightPow, self.o_powRes_enb)  # inner product            
    def f_update_sgw_results(self):                       
        # get SGW load,load ratio and status
        self.o_SgwLoad = self.m_load_update[self.m_sgw_start:self.m_mux_start] #-1)            
        #self.o_SGW_status = self.o_SgwLoad > self.mc_reOnOff_th 
        self.o_SgwLoadRatio = self.o_ResLoadRatio[self.m_sgw_start:self.m_mux_start] #-1)
        self.o_SGW_status = (self.o_SgwLoadRatio > self.mc_reOnOff_th).astype(int) 
        SgwLR_var           = sum((self.o_SgwLoadRatio - self.m_ideal_lb_sgw)**2 / self.md_new_SgwNum)
        self.o_SgwLR_std    = np.sqrt(SgwLR_var)
#             # get SGW LB output
#             self.f_update_lb_sgw(self.m_sdiag)
            #disp(self.m_sdiag)
            #self.f_update_lb_sgw(0.00001)
            # get actual SGW LB for enbs
        self.f_calculate_lb_actual()
    def f_update_lb_sgw(self,sdiag): # similar to f_form_fmin_lb_sgw            
        # get active system capacity
        C_sys  = self.md_C_sys_sgw               # C_sys
        Q_mat  = -(1.0/C_sys)*np.ones(self.i_SGW_num,self.i_SGW_num)  # -1/C_sys, No need for dynamic SGW number (IxI)
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
        if self.i_MUX_num > 0 && self.m_lbMux_en:
            self.o_MuxLoad = self.m_load_update[self.m_mux_start:self.m_link_start] # -1)            
            #self.o_MUX_status = self.o_MuxLoad > self.mc_reOnOff_th 
            self.o_MuxLoadRatio = self.o_ResLoadRatio[self.m_mux_start:self.m_link_start] #-1)
            self.o_MUX_status = (self.o_MuxLoadRatio > self.mc_reOnOff_th).astype(float)
            #disp(self.md_new_MuxNum)
            MuxLR_var           = sum((self.o_MuxLoadRatio - mean(self.o_MuxLoadRatio))**2 / self.md_new_MuxNum)
            self.o_MuxLR_std    = np.sqrt(MuxLR_var)
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

#if __name__ == "__main__": 
