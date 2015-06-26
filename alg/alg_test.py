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

def func(y, sign=1.0):
    return sign*(2*y[0]*y[1] + 2*y[0] - y[0]**2 - 2*y[1]**2)

eqconst = lambda x: np.array([x[0]**3 - x[1]])
def eqfun(x):
    return np.dot(x,[3,-1])

cons = ({'type': 'eq',
         'fun' : eqconst#eqfun # ,
         #'jac' : lambda x: np.array([3.0*(x[0]**2.0), -1.0])
         }#,
        #{'type': 'ineq',
        # 'fun' : lambda x: np.array([x[1] - 1])}
        )
#bnds = ((-2, 1.1), (1, 6))
bnds = [[-2, 1.1], [1, 6]]
x0_w = np.array([-1.0,1.0])

if __name__ == "__main__": 
    res = minimize(func, x0_w, args=(-1.0,), 
                   constraints=(),method='SLSQP',options={'disp': True})
    print res.x, res.fun
    res = minimize(func, x0_w, args=(-1.0,), bounds=bnds, 
                   constraints=cons,method='SLSQP',options={'disp': True})
    print res.x, res.fun
