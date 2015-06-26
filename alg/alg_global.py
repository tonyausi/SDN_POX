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
import scipy.io

from alg.alg_template import projSolverTopTemplate
from utils.proj_util import f_load_mat

TEM_OUT = 'fmin_iteration_out.txt'

'''
['EM', 'res_ratio', 'D', 'Weight', 'I', 'H', 'Beta', 'Iteration', 'S', 'L', 
'NormMode', 'BsAggC', 'RM', 'Threshold', 'Outloop', 'ResCapC', 'PMAX', 'Mode']
'''

class projSolver(projSolverTopTemplate):
    def __init__(self,D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,LB_mode='user',
                 i_sdiag = None, normScale=None, i_fileName=TEM_OUT,
                 plot_en=False,srecord_en=False):
        # call projSolverTemplate's constructor    
        super(projSolver,self).__init__(D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,LB_mode,
                 i_sdiag,normScale,i_fileName,
                 plot_en,srecord_en)
        self.f_simulate()
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
        #self.f_plot_results()
        #self.f_evaluate_magnitude()

    def f_form_fmin(self):
        # form A,B matrices
        self.f_form_active_map()
        # form system normalised incremental power
        self.f_form_fmin_syspow()
        # form SGW load variance term
        self.f_form_fmin_lb_sgw()  
        self.f_form_eq_pen()        
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
                
        f_bound=lambda x,y: [[x[i],y[i]] for i in range(len(x))]
        # get initial value based on upper bound
        x0_w = self.f_getx0(joint_up) #disp(x0_w)
        #print x0_w
        #T = self.m_throughput
        #x0 = T./self.m_RtNum.*ones(1,self.m_RtNum) # Make a starting guess at the solution
        #x0 = 1 .* T./self.m_RtNum.*ones(1,self.m_RtNum)
#         x0 = x0_w * 0.7 #%%%%%%%%% magic number ?????????????
#        x0 = lp
        x0 = x0_w * 0.1 #%%%%%%%%% magic number ?????????????

#        self.m_debug.x0 = x0                              
        if (self.i_constraint_mode=='lag'):
            A  = self.m_ResRouteMat
            b  = self.m_RES_cap                      
            Aeq= self.m_EnbRouteMat
            beq= self.i_traffic_in
            #print x0,np.shape(x0),np.shape(Aeq),np.shape(beq)
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
            raise Exception('i_constraint_mode mode not supported yet')
        md = 'SLSQP' # 'L-BFGS-B' # 'Newton-CG' # 'COBYLA' # 'TNC' # 'SLSQP' # 
        #op = {'maxiter':3000,'ftol': 1e-10, 'disp': True}  #'ftol': 1e-10, 'maxiter':3000,
        op = {'disp': True}  #'ftol': 1e-10, 'maxiter':3000,
        # debug
        #print x0 
        #print up
        #print self.m_RES_peff_approx,self.md_pnorm
        #print self.m_fvec
        
        # [self.m_rate_update,self.o_fval] = fmincon(@(x)f_objfun(self,x),x0,A,b,Aeq,beq,lp,up,[],options)
        minRes = minimize(self.f_objfun, x0, bounds=f_bound(lp,up), constraints=cons,
                          method=md,options=op)
        ''' data output '''
        self.m_rate_update = np.array(minRes.x) 
        ''' min value for target function '''    
        self.o_fval        = minRes.fun  
        ''' scaling back ''' 
        self.o_fval = self.o_fval * self.m_normScale
        ''' 'remove every tiny numbers (+ and -) '''
        self.f_adjust_nearZero()
    def f_objfun(self,d_route):
        sys_pow = np.dot(self.m_fvec, d_route) # 1xR * 1xR^T=1x1
        sys_lb_sgw = np.dot(np.dot(d_route, self.md_QtildaSgw), d_route) #1xR * RxR * 1xR'=1x1
        sys_delay_mux = self.f_form_mux_delay(d_route)
#             print sys_delay_mux
        f = sys_pow + sys_lb_sgw + sys_delay_mux
        if (self.i_constraint_mode=='pen'):
            sys_eq_pen = (np.dot(self.m_fvec_eq_pen, d_route)                      # 1xR d* 1xR --> 1x1
                           + np.dot(np.dot(d_route, self.m_Bsquare_pen), d_route)  # 1xR * RxR * Rx1
                           + self.m_const_pen)                       
            f = f + sys_eq_pen + self.f_calc_rescap_penalty(d_route)
        return f
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


    def f_recalculate(self):
        # called by external entity
        self.f_check();
        self.f_init();
        self.f_simulate();

if __name__ == "__main__": 
    #matIn = f_load_mat('USNET_2.mat')
    matIn = f_load_mat('case2a.mat')
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
    Weight = 0.1 # matIn['Weight']  #  0.5 #   
    Threshold = matIn['Threshold'].flatten()[0];Iteration = matIn['Iteration'];
    NormMode = [];Outloop = 0;
    o_file = 'fmin_iteration_out_sdn.txt'
    GameMode = matIn['Mode'][0];RES_ratio = matIn['res_ratio'].flatten();constraint_mode = 'lag'
    # scale for USNET test cases
    #PMAX = PMAX/1000000.0
    Gsolver = projSolver(D,C,PMAX,Beta,I,H,L,EM,RM,
                         Weight,Threshold,Iteration,NormMode,Outloop,
                         GameMode,RES_ratio,constraint_mode,
                         i_sdiag = None, normScale=None,
                         i_fileName=o_file)
    Gsolver.i_traffic_in = D
    Gsolver.f_recalculate()
    