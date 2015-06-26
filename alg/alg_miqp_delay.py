#!/usr/bin/python

'''
Cloud project:
- Software Defined Networking (SDN) 
-- project related algorithm (global solution)

Tony Lin
'''
#import re
import numpy as np
#from scipy.optimize import minimize
#import cplex
#import scipy.io

from alg.alg_miqp import miqpSolver
from utils.proj_util import f_load_mat

TEM_OUT = 'miqp_delay_out.txt'


class miqpSolverDelay(miqpSolver):
    def __init__(self,D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,RES_delay,delayTh,
                 LB_mode='user',i_sdiag = None, normScale=None, i_fileName=TEM_OUT,                 
                 plot_en=False,srecord_en=False):
        self.i_RES_delay = np.array(RES_delay)
        self.i_delayTh = np.array(delayTh)
        # call projSolverTemplate's constructor    
        super(miqpSolverDelay,self).__init__(D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,LB_mode,
                 i_sdiag,normScale,i_fileName,
                 plot_en,srecord_en)
        #print 'm_res_appox_en=',self.m_res_appox_en

    ''' overloaded method '''
    def f_init_pow_norm(self):  
        #print 'get child method' 
        if not(self.io_norm_mode):
            self.io_norm_mode = 'mean'
        # get power normalisation term
#         self.md_max_pow     = self.i_RES_maxpow * self.md_res_config_new
#         print 'child overload f_init_pow_norm'
        if self.m_res_appox_en:
            self.md_max_pow = (self.m_RES_peff_approx * self.md_res_config_new * 
                                 self.i_RES_cap * self.i_RES_ratio)
        else:
            self.md_max_pow = self.i_RES_maxpow * self.md_res_config_new
        #print 'self.md_max_pow=',self.md_max_pow,'self.m_res_appox_en=',self.m_res_appox_en
        if self.io_norm_mode=='mean':
            self.m_Pratio = 1.0 #       0.8  #       
#                  self.md_pnorm = sum(self.md_max_pow) / sum(self.md_res_config_new) * self.m_Pratio
            pow_scale = 1.0
            # scale norm weights to proper level
            pow_scale = pow_scale * self.m_normScale
            if min(self.m_WeightPow.flatten()) < (1 - 0.001):                    
                self.md_pnorm = sum(self.md_max_pow * pow_scale) 
            else:
                if self.io_scale_mode ==1:
                    p_weight = 0.1 * sum(self.i_traffic_in) / self.m_EnbNum
#                     p_weight = p_weight + 0.02
                else:                        
                    p_weight = 1.0
                print p_weight, pow_scale
                self.md_pnorm = p_weight*sum(self.md_max_pow)*pow_scale 
        elif self.io_norm_mode=='max':
            self.md_pnorm = self.i_RES_maxpow
        else:
            raise Exception('Incorrect SGW normalisation mode')  
        print 'True pow normaliser = %2.5f ' %self.md_pnorm
    ''' overloaded method '''  
#     def f_init_lb_norm(self):            
#         m_new_SGW_cap = self.m_RES_cap[self.m_sgw_start : self.m_mux_start]
#         self.md_C_sys_sgw = sum(m_new_SGW_cap)
#         self.m_ideal_lb_sgw = self.m_throughput / self.md_C_sys_sgw #m_throughput as T calculated in f_check
#         if self.m_varNorm_en == 0:
#             return
#         self.md_var_norm = (((1.0 - self.m_ideal_lb_sgw)**2 
#                        + (self.md_new_SgwNum-1)*self.m_ideal_lb_sgw**2) 
#                        / self.md_new_SgwNum)
#         ''' remove MUX norm'''
#         print 'True var normaliser = %2.5f ' %self.md_var_norm
                               
    def f_init_extensible(self):
        self.f_init_routeDelay()
        
    def f_init_routeDelay(self):
        #print 'extensible'#, self.i_delayTh 
        D_shape = np.shape(self.i_traffic_in)
        delayThShape = np.shape(self.i_delayTh)
        if delayThShape:
            assert (delayThShape == D_shape)
            self.m_delayThIn = self.i_delayTh
        else:
            self.m_delayThIn = self.i_delayTh * np.ones(D_shape)
        #print 'self.m_delayThIn=',self.m_delayThIn
        #print 'self.m_EnbRouteMat shape =',np.shape(self.m_EnbRouteMat)
        # Route delay constraint
        self.m_delayThConstraint = np.dot(self.m_delayThIn,self.m_EnbRouteMat) # J * JxR --> R
        #print 'self.m_delayThConstraint=',np.shape(self.m_delayThConstraint)   
        # self.m_ResRouteMat: NxR
        #print 'self.m_ResRouteMat.shape=',np.shape(self.m_ResRouteMat)
        self.m_routeDelay = np.dot(self.i_RES_delay, self.m_ResRouteMat) # N * NxR --> R
        #print  self.m_routeDelay
        self.m_userRouteDelay = self.m_EnbRouteMat * self.m_routeDelay
        #print self.m_userRouteDelay
    def f_check_delayPara(self):
        if not self.i_delayTh:
            raise Exception('missing delay threshold for the mode with delay constraints')
        if not self.i_RES_delay.all():
            raise Exception('some delay parameter is missing with self.i_RES_delay=%s' 
                            %str(self.i_RES_delay))
    

#     def f_form_fmin(self):
#         # form A,B matrices
#         self.f_form_active_map()
#         # form system normalised incremental power
#         self.f_form_fmin_syspow()  # self.m_fvec
#         # form SGW load variance term
#         self.f_form_fmin_lb_sgw_p1()  # self.md_QtildaSgw
#         
#         self.f_form_qp_lb_mux_p1()
# 
        
    ''' overloaded method ''' 
    def f_form_qp_lb_mux_p1(self):
        pass                             
    def f_miqp_constraints(self):
        self.m_var_num = self.m_RtNum+self.m_ResNum
        Aineq_continue  = np.concatenate((self.m_ResRouteMat,np.zeros((self.m_ResNum,self.m_ResNum))),axis=1)  # dim: NxR cat2 NxN --> Nx(R+N)
        #print np.shape(Aineq_continue)
        Aineq_integer   = np.concatenate((self.m_ResRouteMat,-self.mc_U * np.eye(self.m_ResNum)),axis=1)           # dim: NxR cat2 NxN
        #print np.shape(Aineq_integer)
        Aineq_delay    = np.concatenate((np.diag(self.m_routeDelay-self.m_delayThConstraint),
                                        np.zeros((self.m_RtNum,self.m_ResNum))),axis=1)     # dim: RxR cat2 RxN --> Rx(R+N)
        Aineq           = np.concatenate((Aineq_continue,Aineq_integer),axis=0)              # dim: Nx(R+N) cat1 Nx(R+N) --> 2Nx(R+N)        
        bineq  = np.concatenate((self.m_RES_cap,np.zeros(self.m_ResNum,)),axis=0);           # dim 1xN' cat1 Nx1 --> 2Nx1
        # append delay related
        # append delay related
        Aineq           = np.concatenate((Aineq,Aineq_delay),axis=0)              # dim: 2Nx(R+N) cat1 Rx(R+N) --> (2N+R)x(R+N)
        bineq  = np.concatenate((bineq,np.zeros(self.m_RtNum,)),axis=0);           # dim 2N cat1 R --> (2N+R)
        #print np.shape(Aineq) , np.shape(bineq)
        #print np.shape(self.m_EnbRouteMat)
        Aeq    = np.concatenate((self.m_EnbRouteMat,np.zeros((self.m_EnbNum,self.m_ResNum))),axis=1) # dim: JxR cat2 JxN -->Jx(R+N)
        beq    = self.i_traffic_in   # dim: Jx1
        #print np.shape(Aeq) , np.shape(beq)
        Amat = np.concatenate((Aineq,Aeq),axis=0)
        self.m_miqp_rhs = np.concatenate((bineq,beq),axis=1)
        #print np.shape(Amat), np.shape(self.m_miqp_rhs),self.m_miqp_rhs        
        self.m_miqp_cols = self.f_get_cols(Amat)
        #print self.m_miqp_cols
         
        sense_L = 'L'*(2*self.m_ResNum+self.m_RtNum) # dim: (2N+R)
        sense_E = 'E'*self.m_EnbNum
        self.m_miqp_sense = sense_L + sense_E
        #print self.m_miqp_sense
#         super(miqpSolverDelay, self).f_miqp_constraints()          
            

if __name__ == "__main__": 
    #matIn = f_load_mat('USNET_2.mat')
    #matIn = f_load_mat('case2a.mat')
    matIn = f_load_mat('USNET_11.mat')
#    matIn = f_load_mat('USNET_12.mat')
    #matIn = f_load_mat('case2.mat')
    '''
    D,C,PMAX,Beta,I,H,L,EM,RM,
    Weight,Threshold,Iteration,NormMode,Outloop,
    GameMode,RES_ratio,constraint_mode,    
    i_sdiag = None, normScale=None,
    i_fileName=TEM_OUT,                 
    plot_en=False,srecord_en=False
    '''
    #print matIn
    D = matIn['D'].flatten();C = matIn['S'].flatten();PMAX = matIn['PMAX'].flatten();Beta = matIn['Beta'].flatten();    
    I = matIn['I'].flatten()[0];H = matIn['H'].flatten()[0];L = matIn['L'].flatten()[0];EM = matIn['EM'];RM = matIn['RM'];
    Weight = matIn['Weight']  #  0.5 #   
    Threshold = matIn['Threshold'].flatten()[0];Iteration = matIn['Iteration'];
    NormMode = [];Outloop = 0;
    o_file = 'fmin_iteration_out_sdn.txt'
    GameMode = matIn['Mode'][0]
    RES_ratio = None #RES_ratio = matIn['res_ratio'].flatten()
    constraint_mode = 'lag'
    # scale for USNET test cases
    #PMAX = PMAX/1000000.0
    #D = np.array([0.5,0.9])
    link_delay = [7.6,9.44,8.52,4.38,7.6,8.52,8.52,8.52,8.06,9.9,
                  8.52,9.44,14.04,9.9,8.52,8.06,8.52,10.82,8.52,8.52,
                  8.06,8.06,8.06,6.68]
    RES_delay = [10]*I+[0.0]*H+link_delay#[10]*L
    dShape = np.shape(D)
    delayTh = [350,350,25,20] #30*np.ones(dShape)#;print delayTh # [555,666] # 
#    delayTh = [350,180,350,350] #30*np.ones(dShape)#;print delayTh # [555,666] # 
    solver  = miqpSolverDelay(D,C,PMAX,Beta,I,H,L,EM,RM,
                         Weight,Threshold,Iteration,NormMode,Outloop,
                         GameMode,RES_ratio,constraint_mode,RES_delay,delayTh,
                         i_sdiag = None, normScale=18.0,
                         i_fileName=o_file,LB_mode='global') 
    #print solver.m_normScale
    print 'solver.m_userRouteDelay=',solver.m_userRouteDelay
    print 'solver.o_PowConsum2=',solver.o_PowConsum2   
    print D
    solver.i_traffic_in = np.array([4.523E+01,5.470E+01,7.395E+00,7.483E+01]) # np.array([0.7,1.1])
    solver.f_recalculate()  
    solver.i_traffic_in = np.array([4.309E+01,5.211E+01,7.052E+00,7.143E+01]) # np.array([0.9,1.3])
    solver.f_recalculate()
    print 'solver.o_PowConsum2=',solver.o_PowConsum2 
#     solver.i_traffic_in = np.array([0.9,1.3])
#     solver.f_recalculate()
    #print solver.io_norm_mode,solver.md_max_pow,solver.m_normScale,solver.i_normScale
    #print solver.m_miqp_obj
