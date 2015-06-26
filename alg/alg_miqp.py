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
import cplex
import scipy.io

from alg.alg_template import projSolverTopTemplate
from utils.proj_util import f_load_mat

TEM_OUT = 'miqp_out.txt'


class miqpSolver(projSolverTopTemplate):
    def __init__(self,D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,LB_mode='user',
                 i_sdiag = None, normScale=None, i_fileName=TEM_OUT,
                 plot_en=False,srecord_en=False):
        # call projSolverTemplate's constructor    
        super(miqpSolver,self).__init__(D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,LB_mode,
                 i_sdiag,normScale,i_fileName,
                 plot_en,srecord_en)
        #print 'm_res_appox_en=',self.m_res_appox_en
        self.f_init_miqp_m()
        self.f_simulate()
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
        if self.io_norm_mode=='mean':
            self.m_Pratio = 1.0 #       0.8  #       
#                  self.md_pnorm = sum(self.md_max_pow) / sum(self.md_res_config_new) * self.m_Pratio
            pow_scale = sum(self.md_sgw_config_new) / sum(self.md_res_config_new) # ???? not clear scale by sum(self.md_sgw_config_new)
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
    def f_init_lb_norm(self):            
        m_new_SGW_cap = self.m_RES_cap[self.m_sgw_start : self.m_mux_start]
        self.md_C_sys_sgw = sum(m_new_SGW_cap)
        self.m_ideal_lb_sgw = self.m_throughput / self.md_C_sys_sgw #m_throughput as T calculated in f_check
        if self.m_varNorm_en == 0:
            return
        self.md_var_norm = (((1.0 - self.m_ideal_lb_sgw)**2 
                       + (self.md_new_SgwNum-1)*self.m_ideal_lb_sgw**2) 
                       / self.md_new_SgwNum)
        if self.m_lbMux_en == 1:
            if self.md_new_MuxNum < 1:
                raise Exception('Warning: number of MUX is 0')
            m_new_MUX_cap = self.m_RES_cap[self.m_mux_start : self.m_link_start]
            self.md_C_sys_mux = sum(m_new_MUX_cap);
            self.m_ideal_lb_mux = self.m_throughput / self.md_C_sys_mux;
            mux_var_norm = (((1.0 - self.m_ideal_lb_mux)**2 
                       + (self.md_new_MuxNum-1)*self.m_ideal_lb_mux**2) 
                       / self.md_new_MuxNum)
            self.md_var_norm = self.md_var_norm + mux_var_norm
        print 'True var normaliser = %2.5f ' %self.md_var_norm
    ''' overloaded method '''    
    def f_init_sdiag(self):
        if not(self.i_sdiag):
            self.m_sdiag = 0.0000
        else:
            self.m_sdiag = self.i_sdiag
    ''' overloaded method '''
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
                               
    def f_init_miqp_m(self):
        self.mc_U      = 100000.0
        self.m_var_num = None
        self.m_miqp_cols  = None
        self.m_miqp_sense = None
        self.m_miqp_type  = None
        self.m_miqp_name = None
        self.m_miqp_rhs  = None
        self.m_miqp_qmat = None
        self.m_miqp_up   = []
        self.m_miqp_lb   = []
        self.md_idle_powNorm = None
        self.m_miqp_obj  = []
        self.m_miqpObj = cplex.Cplex()
        self.m_miqpObj.set_results_stream(None)
        self.m_miqpObj.set_log_stream(None)
        # extensible method for child class
        self.f_init_extensible()
    def f_init_extensible(self):
        pass
        
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
        self.f_miqp_cal()
        self.f_update_results()            
        self.f_record_results()
        #self.f_plot_results()
        #self.f_evaluate_magnitude()
    def f_form_fmin(self):
        # form A,B matrices
        self.f_form_active_map()
        # form system normalised incremental power
        self.f_form_fmin_syspow()  # self.m_fvec
        # form SGW load variance term
        self.f_form_fmin_lb_sgw_p1()  # self.md_QtildaSgw
        
        self.f_form_qp_lb_mux_p1()

    def f_miqp_cal(self):
        self.setproblemdata()
        self.m_miqpObj.solve()
        self.f_debug_miqp()
        sol = self.m_miqpObj.solution
        allOut = sol.get_values()
        ''' data output '''
        self.m_rate_update = np.array(allOut[0:self.m_RtNum]) 
        #print self.m_rate_update
        ''' min value for target function '''    
        self.o_fval        = sol.get_objective_value()  
        ''' scaling back ''' 
        self.o_fval = self.o_fval * self.m_normScale
        ''' 'remove every tiny numbers (+ and -) '''
        self.f_adjust_nearZero()
    def f_debug_miqp(self):
        if not self.m_debug:
            return
        sol = self.m_miqpObj.solution
        # solution.get_status() returns an integer code
        print "Solution status = " , sol.get_status(), ":",
        # the following line prints the corresponding string
        print sol.status[sol.get_status()]
        print "Solution value  = ", sol.get_objective_value()
        numrows = self.m_miqpObj.linear_constraints.get_num()
        numcols = self.m_miqpObj.variables.get_num()
        for j in range(numcols):
            print "Column %d:  Value = %10f" % (j, sol.get_values(j))
        print self.m_miqpObj.problem_type[self.m_miqpObj.get_problem_type()]        
    def setproblemdata(self): 
        # set up constraints
        self.f_miqp_constraints()
        # set up obj and variables
        self.f_miqp_obj()
        # set up up and lp 
        self.f_miqp_boundary()
        # set up qmat
        self.f_miqp_qmat()         
        self.m_miqpObj.set_problem_name ("miqp solution")
        self.m_miqpObj.objective.set_sense(self.m_miqpObj.objective.sense.minimize)  
        self.m_miqpObj.linear_constraints.add(rhs = list(self.m_miqp_rhs), senses = self.m_miqp_sense)
        self.m_miqpObj.variables.add(obj = self.m_miqp_obj, ub = self.m_miqp_up, lb = self.m_miqp_lb, 
                                     columns = self.m_miqp_cols, types=self.m_miqp_type, names = self.m_miqp_name)
        self.m_miqpObj.objective.set_quadratic(self.m_miqp_qmat)                               
    def f_miqp_constraints(self):
        self.m_var_num = self.m_RtNum+self.m_ResNum
        Aineq_continue  = np.concatenate((self.m_ResRouteMat,np.zeros((self.m_ResNum,self.m_ResNum))),axis=1)  # dim: NxR cat2 NxN --> Nx(R+N)
        #print np.shape(Aineq_continue)
        Aineq_integer   = np.concatenate((self.m_ResRouteMat,-self.mc_U * np.eye(self.m_ResNum)),axis=1)           # dim: NxR cat2 NxN
        #print np.shape(Aineq_integer)
        Aineq           = np.concatenate((Aineq_continue,Aineq_integer),axis=0)              # dim: Nx(R+N) cat1 Nx(R+N) --> 2Nx(R+N)
        bineq  = np.concatenate((self.m_RES_cap,np.zeros(self.m_ResNum,)),axis=0);           # dim 1xN' cat1 Nx1 --> 2Nx1
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
        
        sense_L = 'L'*(2*self.m_ResNum)
        sense_E = 'E'*self.m_EnbNum
        self.m_miqp_sense = sense_L + sense_E
        #print self.m_miqp_sense
    def f_get_cols(self,constraint_mat):
        [rowNum,colNum] = np.shape(constraint_mat)
        #print rowNum, colNum
        cols = []
        for c_idx in range(colNum):
            row_key = []
            row_value = []
            for r_idx in range(rowNum):
                if constraint_mat[r_idx,c_idx] != 0:
                    row_key.append(r_idx)
                    row_value.append(constraint_mat[r_idx,c_idx])
            cols.append([row_key,row_value])
        return cols        
    def f_miqp_obj(self):
        name_d  = ['d%i' %i for i in range(self.m_RtNum)]
        type_d  = 'C'*self.m_RtNum
        name_re = ['n%i' %n for n in range(self.m_ResNum)]
        type_re = 'I'*self.m_ResNum
        self.m_miqp_name = name_d + name_re
        self.m_miqp_type = type_d+type_re
        #print self.m_miqp_name,self.m_miqp_type
        self.md_idle_powNorm = np.average(self.m_WeightPow) / self.md_pnorm * self.i_RES_idleScale * self.i_RES_maxpow
        #print np.shape(self.m_fvec),np.shape(self.md_idle_powNorm)
        #f  = np.concatenate((self.m_fvec,self.md_idle_powNorm),axis=0)  # dim: (R+N)x1
        self.m_miqp_obj = list(self.m_fvec.flatten()) + list(self.md_idle_powNorm)
        #print self.m_miqp_obj
    def f_miqp_boundary(self):
        self.m_miqp_up = np.ones((self.m_var_num,))
        # set up upper boundary for traffic per route
        upbound = self.f_getjoint_up()
        self.m_miqp_up[0:self.m_RtNum] = upbound
        # set up upper for binary RES auxilary integer variable
        self.m_miqp_up[self.m_RtNum:self.m_var_num] = self.md_res_config_new
        # set up lower
        self.m_miqp_lb = np.zeros((self.m_var_num,))
    def f_miqp_qmat(self):         
        sdiag  = self.m_sdiag*np.eye(self.m_RtNum,self.m_RtNum)
        H_sub  = self.md_Qtilda + sdiag   # dim: RxR
        #print np.max(H_sub-H_sub.T)
        H_sub  = (H_sub+H_sub.T) / 2 # silly numpy had rounding issue
        self.m_miqp_qmat = self.f_get_qmat(H_sub,self.m_var_num)
        #print self.m_miqp_qmat       
    def f_get_qmat(self,qmat_in,var_num):
        [rowNum,colNum] = np.shape(qmat_in)
        qmat = []
        key = [r for r in range(rowNum)]
        for c_idx in range(colNum):
            qmat.append([key,list(qmat_in[:,c_idx])])
        for c_idx in range(colNum,var_num):
            qmat.append([[c_idx],[0.0]])
        return qmat
                        
    def f_recalculate(self):
        # called by external entity
        self.f_check();
        self.f_init_miqp_m()
        self.f_init();
        self.f_simulate();        
            

if __name__ == "__main__": 
    #matIn = f_load_mat('USNET_2.mat')
    matIn = f_load_mat('case2a.mat')
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
    Weight = 0.75 #matIn['Weight']  #  0.5 #   
    Threshold = matIn['Threshold'].flatten()[0];Iteration = matIn['Iteration'];
    NormMode = [];Outloop = 0;
    o_file = 'fmin_iteration_out_sdn.txt'
    GameMode = matIn['Mode'][0]
    RES_ratio = None #RES_ratio = matIn['res_ratio'].flatten()
    constraint_mode = 'lag'
    # scale for USNET test cases
    #PMAX = PMAX/1000000.0
    D = np.array([0.5,0.9])
    solver  = miqpSolver(D,C,PMAX,Beta,I,H,L,EM,RM,
                         Weight,Threshold,Iteration,NormMode,Outloop,
                         GameMode,RES_ratio,constraint_mode,
                         i_sdiag = None, normScale=18.0,
                         i_fileName=o_file,LB_mode='global') 
    #print solver.m_normScale   
    print D
    solver.i_traffic_in = np.array([0.7,1.1])
    solver.f_recalculate()  
    solver.i_traffic_in = np.array([0.9,1.3])
    solver.f_recalculate()
#     solver.i_traffic_in = np.array([0.9,1.3])
#     solver.f_recalculate()
    #print solver.io_norm_mode,solver.md_max_pow,solver.m_normScale,solver.i_normScale
    #print solver.m_miqp_obj
