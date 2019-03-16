import numpy as np
from ctypes import *

class CasadiSparsity:
  """
  Compressed Columun storage sparsity pattern
  """
  def __init__(self,sp):
    self.n_row = sp[0]
    self.n_col = sp[1]
    self.colind = np.array(sp[2:2+self.n_col+1])
    self.nnz = self.colind[-1]
    self.row = np.array(sp[2+self.n_col+1:2+self.n_col+1+self.nnz])
  def is_dense(self):
   return self.nnz == self.n_row*self.n_col

class CasadiCodegenWrapper:
  def __init__(self,library,name):
    self.name = name
    self.lib = cdll.LoadLibrary("./f.so")
    self._f("n_in").restype = c_longlong
    self.n_in = self._f("n_in")()
    self._f("n_out").restype = c_longlong
    self.n_out = self._f("n_out")()

    f_name_in = self._f("name_in")
    f_name_in.argtypes = [c_longlong]
    f_name_in.restype = c_char_p

    self.names_in = [str(f_name_in(i)) for i in range(self.n_in)]

    f_name_out = self._f("name_out")
    f_name_out.argtypes = [c_longlong]
    f_name_out.restype = c_char_p

    self.names_out = [str(f_name_out(i)) for i in range(self.n_out)]

    sparsity_in = self._f("sparsity_in")
    sparsity_in.argtypes = [c_longlong]
    sparsity_in.restype = POINTER(c_longlong)

    self.sparsity_in = [CasadiSparsity(sparsity_in(i)) for i in range(self.n_in)]

    sparsity_out = self._f("sparsity_out")
    sparsity_out.argtypes = [c_longlong]
    sparsity_out.restype = POINTER(c_longlong)
    
    self.sparsity_out = [CasadiSparsity(sparsity_out(i)) for i in range(self.n_out)]

    self.default_in = self._f("default_in")
    self.default_in.argtypes = [c_longlong]
    self.default_in.restype = c_double

    self.incref = self._f("incref")
    self.decref = self._f("decref")

    self.f = self._f()
    self.f.argtypes = [POINTER(POINTER(c_double)),POINTER(POINTER(c_double)),POINTER(c_longlong),POINTER(c_double),c_int]
    self.f.restype = c_int

    self.checkout = self._f("checkout")
    self.checkout.restype = c_int
    self.release = self._f("release")
    self.release.argtypes = [c_int]
    self.release.restype = c_int
    mem = self.checkout()
    self.release(mem)
    self.incref()

    work = self._f("work")
    work.restype = c_int
    work.argtypes = [POINTER(c_longlong)]*4
    sz_arg = c_longlong()
    sz_res = c_longlong()
    sz_iw  = c_longlong()
    sz_w   = c_longlong()
    work(byref(sz_arg), byref(sz_res), byref(sz_iw), byref(sz_w))

    self.arg = (POINTER(c_double)*sz_arg.value)()
    self.res = (POINTER(c_double)*sz_res.value)()
    self.iw = (c_longlong*sz_iw.value)()
    self.w  = (c_double*sz_w.value)()

  def __del__(self):
    self.decref()

  def _f(self,name=None):
    name = self.name if name is None else self.name + "_" + name
    
    return getattr(self.lib,name)

  def __call__(*arg,**kwargs):
    assert len(arg)==0 or len(kwargs==0), "Cannot mix positional and keyword arguments"
    assert len(arg)==self.n_in
    mem = self.checkout()
    for i in range(self.n_in):
      self.arg[i] = arg[i].ctypes.data_as(POINTER(c_double))
    self.f(self.arg,self.res,self.iw,self.w,mem)
    self.release(mem)

  def allocate_outputs(self):
    r = []
    for i in range(self.n_out):
      sp = self.sparsity_out[i]
      assert sp.is_dense()
      r.append(np.zeros((sp.n_row, sp.n_col),dtype=np.double,order="F"))
    return r

  def call_efficient(self,arg,res):
    assert len(arg)==self.n_in
    assert len(res)==self.n_out
    mem = self.checkout()
    for i in range(self.n_in):
      self.arg[i] = arg[i].ctypes.data_as(POINTER(c_double))
    for i in range(self.n_out):
      self.res[i] = res[i].ctypes.data_as(POINTER(c_double))
    self.f(self.arg,self.res,self.iw,self.w,mem)
    self.release(mem)



if __name__ == "__main__":
  wrapper = CasadiCodegenWrapper("./f.so","f")

  output_buffer = wrapper.allocate_outputs()

  print(output_buffer)

  arg = np.array([[1,-1],[-1,2]],dtype=c_double,order="F")
  wrapper.call_efficient([arg],output_buffer)

  print(output_buffer)

