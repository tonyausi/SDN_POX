#!/usr/bin/python

'''
Cloud project:
- Software Defined Networking (SDN) 
-- project related algorithm (best response)

Tony Lin
'''
import re
import numpy as np
from scipy.optimize import minimize
#import scipy.io

from alg.alg_template import projSolverTopTemplate, userSolverTemplate
from utils.proj_util import f_load_mat

TEM_OUT = 'bestResponse_iteration_out.txt'                      

class sysSolver(projSolverTopTemplate):
    def __init__(self,D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,LB_mode='user',
                 i_sdiag = None, normScale=None,
                 i_fileName=TEM_OUT,
                 plot_en=False,srecord_en=False):
        # call projSolverTopTemplate's constructor    
        super(sysSolver,self).__init__(D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,LB_mode,
                 i_sdiag,normScale,i_fileName,
                 plot_en,srecord_en)
        # get system self.m_fvec for logging
        self.f_form_fmin_syspow()
        # form SGW load variance term for logging
        self.f_form_fmin_lb_sgw()
        self.f_init_userSolver()
        self.f_simulate()
    def f_init_userSolver(self):
        ''' initial user solver '''
        self.m_BS = []
        self.m_userRtVect = []
#         self.m_sdiag = 0.7
        for j in range(self.m_EnbNum):
            Weight_user = [self.m_WeightPow[j],self.m_WeightSgw[j],self.m_WeightMux[j]]
            userObj = userBestResponseSolver(j,D[j],self.i_RES_cap,self.i_SGW_num,self.i_MUX_num,self.i_link_num,self.i_ResRouteMat,self.i_EnbRouteMat,
                                             self.m_RES_peff_approx,self.md_pnorm,self.md_MUX_ntime,self.md_var_norm,
                                             self.m_sdiag,Weight_user,self.m_EnbNum)
            self.m_BS.append(userObj) 
            self.m_userRtVect.append(userObj.m_enbRt_vect)
        
    def f_simulate(self):
        ''' main simulator '''
        print 'get to f_simulate(self)'
        iter = 0
        while iter <=21:
            #print np.shape(self.m_ResRouteMat), np.shape(self.m_rate_update)
            load_all = np.dot(self.m_ResRouteMat,self.m_rate_update.flatten())
            #print np.shape(load_all)
            for j in range(self.m_EnbNum):
                self.m_BS[j].id_res_loads = load_all
#                 print self.m_BS[j].m_rate_update
                #self.m_rate_update[:,self.m_userRtVect[j]>0] = self.m_BS[j].m_rate_update
                self.m_rate_update[self.m_userRtVect[j]>0] = self.m_BS[j].m_rate_update
            print 'At iter %d, self.m_rate_update=%s' %(iter, str(self.m_rate_update))
            iter += 1
#         print 'self.m_fvec=',self.m_fvec
        
        self.f_update_results() 
        self.o_fval = sum([self.m_BS[j].o_fval for j in range(self.m_EnbNum)])           
        self.f_record_results()
        
class userBestResponseSolver(userSolverTemplate):
    def __init__(self,j,D,C,I,H,L,RM,EM,E,pnorm,D_mux,V_norm,sdiag,
                 Weight,numbs):
        BsAgg_step = None
        R_sgw      = None
        # call projSolverTemplate's constructor    
        super(userBestResponseSolver,self).__init__(j,D,C,I,H,L,RM,EM,E,pnorm,R_sgw,D_mux,V_norm,sdiag,
                 Weight,numbs,BsAgg_step)
        self.f_int()
    def f_int(self):
        self.f_ini_m()
        # self._id_res_loads
        # init traffic distribution
        self.f_init_rate()
        self.f_init_userResRouteMat()
    def f_ini_m(self):
        if not(self.i_sdiag):
            self.m_sdiag = 0.001 / max(1,sum(self.i_traffic_in))
        else:
            self.m_sdiag = self.i_sdiag
        self.md_resLoadOther = None
        self.md_sgwLoadOther = None   # aggregated load on sgw from other users
        self.md_muxLoadOther = None   # aggregated load on mux from other users                   
    def f_init_rate(self): # overloaded funtion
        self.m_user_pre_allo  = np.zeros((self.m_enbAtRt_num,)) # dim: R_user
        # assume no one route traffic to all RES at begining
        self._id_res_loads     = np.zeros((self.m_ResNum,)) # dim: N        
    def f_init_userResRouteMat(self): #self.m_enbRt_vect,c[np.where(b>1)]=[22,33]
        self.md_userResRouteMat = self.id_ResRouteMat[:,self.m_enbRt_vect>0] #dim: NxR_user
        self.md_userSgwRouteMat = self.md_userResRouteMat[self.m_sgw_start:self.m_mux_start,:] #IxR_user
        self.md_userMuxRouteMat = self.md_userResRouteMat[self.m_mux_start:self.m_link_start,:] #MxR_user
#        self.f_calculateUserRes()

        
    @property
    def id_res_loads(self):
        return self._id_res_loads
    @id_res_loads.setter
    def id_res_loads(self, load_in): 
        assert(len(load_in)==len(self._id_res_loads))
        self._id_res_loads = np.array(load_in)
        # re-evaluate other users load
        self.f_calculateUserRes()
        # get best response based on other users' decision
        ''' global solver but not MIP solution'''
        self.f_min_cal() 
    def f_calculateUserRes(self):
        self.m_userResLoad = np.dot(self.md_userResRouteMat,self.m_user_pre_allo) # NxR_user b* R_user --> N
        # aggregated load from other users
        self.md_resLoadOther = np.maximum((self._id_res_loads - self.m_userResLoad),0.0)
#         print np.shape(self.m_userResLoad),np.shape(self.md_resLoadOther)
        self.md_sgwLoadOther = self.md_resLoadOther[self.m_sgw_start:self.m_mux_start] 
        self.md_muxLoadOther = self.md_resLoadOther[self.m_mux_start : self.m_link_start]
        # available RES cap for this user
        self.md_RES_cap_user = np.maximum((self.md_RES_cap - self.md_resLoadOther),0.0)
        # aggregated load from other users on allocated SGWs          
    def f_min_cal(self):        
        joint_up = self.f_getjoint_up() # dim: 1xR
        #up = joint_up + 1
        x0_w = self.f_getx0(joint_up)   # dim: 1xR
        
        lp = np.zeros((self.m_enbAtRt_num,))    # dim: 1xR_user
        up = joint_up[self.m_enbRt_vect>0] + 1  # dim: 1xR_user
        # get initial value based on upper bound        
#        x0 = lp
        x0 = x0_w[self.m_enbRt_vect>0] * 0.1 # dim: 1xR_user #%%%%%%%%% magic number ?????????????                     
        f_bound=lambda x,y: [[x[i],y[i]] for i in range(len(x))]
#        self.m_debug.x0 = x0                              
        A  = self.md_userResRouteMat  # dim: NxR_user
        b  = self.md_RES_cap_user # self.m_RES_cap                      
        Aeq= self.m_enbRt_vect[self.m_enbRt_vect>0] #self.i_EnbRouteMat
        beq= self.i_traffic_in
#         print x0,np.shape(x0),np.shape(Aeq),np.shape(beq)
        eqconst   = lambda x: np.dot(Aeq,x)-beq  # C(x)=0 in scipy.optimize.minimize
        ineqconst = lambda x: b-np.dot(A,x)      # C(x)>=0 in scipy.optimize.minimize
        cons = ({'type': 'eq',
                 'fun' : eqconst},
                {'type': 'ineq',
                 'fun' : ineqconst}
                )
        md = 'SLSQP' # 'L-BFGS-B' # 'Newton-CG' # 'COBYLA' # 'TNC' # 'SLSQP' # 
        #op = {'maxiter':3000,'ftol': 1e-10, 'disp': True}  #'ftol': 1e-10, 'maxiter':3000,
        op = {'disp': False}  #'ftol': 1e-10, 'maxiter':3000,
        # debug
        #print x0 
        #print up
        #print self.m_RES_peff_approx,self.md_pnorm
        #print self.m_fvec        
        # [self.m_rate_update,self.o_fval] = fmincon(@(x)f_objfun(self,x),x0,A,b,Aeq,beq,lp,up,[],options)
        minRes = minimize(self.f_objfun, x0, bounds=f_bound(lp,up), constraints=cons,
                          method=md,options=op)
        ''' data output '''
        self.m_rate_update = np.array(minRes.x) # dim: R_user
        ''' min value for target function '''    
        self.o_fval        = minRes.fun  
        ''' scaling back ''' 
        self.o_fval = self.o_fval * self.m_normScale
        ''' 'remove every tiny numbers (+ and -) '''
        self.f_adjust_nearZero()
        self.m_user_pre_allo = self.m_rate_update
    def f_adjust_nearZero(self):
        # remove very small negative number
        self.m_rate_update = np.maximum(self.m_rate_update,0)
        mask = (self.m_rate_update > self.mc_minRate).astype(float)
        self.m_rate_update = self.m_rate_update * mask           
    def f_objfun(self,d_user):
        self.f_form_fmin_userpow()
        self.f_form_fmin_lb_sgw_user()
#         print np.shape(self.m_fvec),np.shape(d_user)
        sys_pow = np.dot(self.m_fvec, d_user) # 1xR_user * 1xR_user^T=1x1
        sys_lb_sgw_1st= np.dot(self.md_lbLinearVect, d_user)
        sys_lb_sgw_qp = np.dot(np.dot(d_user, self.md_QtildaSgw), d_user) #1xR_user * R_userXR_user * 1xR_user'=1x1
        sys_delay_mux = self.f_form_mux_delay(d_user,self.md_muxLoadOther)
#             print sys_delay_mux
        f = sys_pow + sys_lb_sgw_1st + sys_lb_sgw_qp + sys_delay_mux
        return f        
    def f_form_fmin_userpow(self):
        slop_norm = self.m_RES_peff / self.id_RES_pnorm #1xN
        # form eff vector
        self.m_Evec = self.i_WeightPow * slop_norm # 1xN
        # form vector as f for first order coefficient in slover
        # get EAB_j=E*A*B_j
        Bj_diag = np.diag(self.m_enbRt_vect) # RxR
        # 1xN * NxR * RxR --> R  %%%%%%%%%%need to confirm to use i_ResRouteMat or m_ResRouteMat
        #self.m_fvec = np.dot(np.dot(self.m_Evec, self.i_ResRouteMat), Bj_diag) 
        self.m_fvec = np.dot(self.m_Evec, self.md_userResRouteMat) # 1xN * NxR_user        
    def f_form_fmin_lb_sgw_user(self):
        self.m_userSgwSysCap = np.dot(self.m_SGW_cap, self.m_enbSgw_vect) #1xI b* 1xI-->1
        # use user specific C_sys,j to calculate Q^srv,j matrix
        Qsrv_mat = self.f_form_QsrvMat(self.m_userSgwSysCap,self.i_SGW_num,self.m_SGW_cap) # IxI
        B_diag_j = np.diag(self.m_enbSgw_vect)               # IxI
        Q_enb_mat = np.dot(np.dot(B_diag_j, Qsrv_mat), B_diag_j) # B*Q*B,IxI
#         Q_enb_sq  = np.dot(Q_enb_mat.T, Q_enb_mat) # (IxI)^T * IxI = IxI
#         # get A_j(I)^T B_j Q^2 A_j(I): IxR_user^T * IxI * IxR_user  
#         QtildaSgwEnb = np.dot(np.dot(self.md_userSgwRouteMat.T, Q_enb_sq), self.md_userSgwRouteMat) # R_user x R_user
        # Q_j * A_j(I): IxI * IxR_user --> IxR_user
        Q_A_mat  = np.dot(Q_enb_mat,self.md_userSgwRouteMat)
        # A_j(I)^T * Q_j^T * Q_j * A_j(I) --> IxI
        self.md_QtildaSgw = np.dot(Q_A_mat.T, Q_A_mat)
        # 2 D_{-j}^T Q A_j(I) d_j
        userSgwLoadOther = self.md_sgwLoadOther * self.m_enbSgw_vect # dim: 1xI
        # 2 D_{-j}^T Q A_j(I), x d_j
        lbOtherVector    = 2 * np.dot(userSgwLoadOther, Q_A_mat)     # dim: 1xR_user
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
        # vaiance weight: 1/(I*S^hat)
        var_w = self.i_WeightSgw / (self.md_SgwNum * self.id_sgwVar_norm)
        # linear (first order) term
        self.md_lbLinearVect = var_w * lbOtherVector                 # dim: 1xR_user
        # quadratic term
        self.md_QtildaSgw = var_w * self.md_QtildaSgw  # R_user x R_user
        self.md_Qtilda    = 2*self.md_QtildaSgw
        # 
    def f_form_mux_delay(self,d_user,mux_trafficOther):  
        if not self.m_lbMux_en:
            delay_mux = 0
            return delay_mux        
        # get MUX throughput from this user only
        mux_trafficUser = np.dot(d_user, self.md_userMuxRouteMat.T) # 1xR_user * MxR_user'=1xM
        # get active MUX entity delay
        mux_dif = np.maximum(1e-10,self.m_MUX_cap-mux_trafficUser-mux_trafficOther)                
        delay_mux_active = self.m_enbMux_vect / mux_dif # 1xM 
        # get 1/(M*md_mux_norm)
        mux_weight = self.i_WeightMux / (self.id_MUX_ntime * self.md_MuxNum) # 1xM
        # get weighted mux delay
        delay_mux_norm = delay_mux_active * mux_weight # 1xM               
        # get MUX delay sum per route per user
        delay_mux_route = np.dot(delay_mux_norm, self.md_userMuxRouteMat) # 1xM * MxR_user = 1xR_user
        # get MUX delay sum per user
        delay_mux_user = sum(delay_mux_route)
        return delay_mux_user
        
        
if __name__ == "__main__": 
    #matIn = f_load_mat('USNET_2.mat')
    matIn = f_load_mat('case2.mat')
    #print matIn
    D = matIn['D'].flatten();C = matIn['S'].flatten();PMAX = matIn['PMAX'].flatten();Beta = matIn['Beta'].flatten();    
    I = matIn['I'].flatten()[0];H = matIn['H'].flatten()[0];L = matIn['L'].flatten()[0];EM = matIn['EM'];RM = matIn['RM'];
    Weight = 0.5 # matIn['Weight']  #  0.5 #   
    Threshold = matIn['Threshold'].flatten()[0];Iteration = matIn['Iteration'];
    NormMode = [];Outloop = 0;
    GameMode = matIn['Mode'][0];RES_ratio = matIn['res_ratio'].flatten();constraint_mode = 'lag'   
    '''
    for individual solver
    j,D,C,I,H,L,RM,EM,E,pnorm,D_mux,V_norm,sdiag,Weight_user,numbs
    '''
#     j = 0;D_user = D[j]
#     E = np.array([6335.33333,957.46667,1269.93333,   37.26667,28.66667,    
#          11.66667,11.66667,11.66667,11.66667,3.33333,3.33333,3.33333,3.33333,3.33333,3.33333])
#     pnorm = 4233.04
#     D_mux = np.array([ 0.85714286,  0.85714286])
#     V_norm = 0.02650
#     sdiag = 0.0001
#     Weight_user = Weight_user = [0.9,0.05,0.05] #  [1.0,0.0,0.0] # 
#     numbs = 2
#     userObj = userBestResponseSolver(j,D_user,C,I,H,L,RM,EM,E,pnorm,D_mux,V_norm,sdiag,Weight_user,numbs)
#     userObj.id_res_loads = np.zeros((len(E),))
#     #print 'self.md_QtildaSgw=',userObj.md_QtildaSgw
#     print 'With no other traffic, u0 rate decision:',userObj.m_rate_update
#     EM_u1 = EM[1]
#     RM_1 = RM[:,EM_u1>0] #;print RM_1
#     rate_u1 = np.array([0.00000,0.00000,0.00000,0.00000,0.55503,0.34497])
#     load_u1 = np.dot(RM_1,rate_u1)#; print load_u1
#     userObj.m_user_pre_allo  = np.zeros((userObj.m_enbAtRt_num,))
#     userObj.id_res_loads = load_u1
#     print 'other user load:', userObj.md_muxLoadOther
#     print 'With u1 traffic, u0 rate decision:',userObj.m_rate_update
#     ##################
#     rate_new = np.array([0.,0.,0.,0.,0.25649599,0.24350401,      0.,0.,0.,0.,0.55503,0.34497])
#     load_all = np.dot(RM,rate_new)
#     userObj.id_res_loads = load_all
#     print 'After 1 iteration, u0 rate decision:',userObj.m_rate_update
#     ##################
#     rate_new = np.array([0.,0.,0.,0.,0.25649599,0.24350401,      0.,0.,0.,0.,0.55503,0.34497])
#     load_all = np.dot(RM,rate_new)
#     userObj.id_res_loads = load_all
#     print 'After 2 iteration, u0 rate decision:',userObj.m_rate_update
#     ##################
#     rate_new = np.array([0.,0.,0.,0.,0.25649599,0.24350401,      0.,0.,0.,0.,0.55503,0.34497])
#     load_all = np.dot(RM,rate_new)
#     userObj.id_res_loads = load_all
#     print 'After 3 iteration, u0 rate decision:',userObj.m_rate_update
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    '''
    self,D,C,PMAX,Beta,I,H,L,EM,RM,
                 Weight,Threshold,Iteration,NormMode,Outloop,
                 GameMode,RES_ratio,constraint_mode,LB_mode='user',i_fileName=TEM_OUT,normScale=None,
                 plot_en=False,srecord_en=False
    '''
    o_file = 'bestResponse_iteration_out_sdn.txt'
    sysObj = sysSolver(D,C,PMAX,Beta,I,H,L,EM,RM,
                       Weight,Threshold,Iteration,NormMode,Outloop,
                       GameMode,RES_ratio,constraint_mode,
                       i_sdiag = None, normScale=None,
                       i_fileName=o_file)

    
    
    
    
    
    
    
    
    
    