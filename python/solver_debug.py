from glob import glob
from casadi import *

s = glob("solver.*.mtx")

args = {}

for e in s:
  args[e.split(".")[-2]] = DM.from_file(e)

solver = Function.load("solver.casadi")
solver(**args)


