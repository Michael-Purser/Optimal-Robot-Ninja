import sys
sys.path.append("/home/michael/Downloads/casadi/install-python")

from casadi import *
import numpy as np

SQP = Function.load("SQP.casadi")

[x,u,T] = SQP(np.vstack((np.linspace(0,10,201),np.linspace(0,10,201),np.linspace(0,np.pi,201))),np.vstack((np.linspace(0,0.1,200),np.linspace(0,0.1,200))),70,np.hstack((20*np.ones((1000,1)),20*np.ones((1000,1)))),0.2,-0.2,0.2,-0.1,0.1,-1,1,2,0.2)

print x
print u
print T
