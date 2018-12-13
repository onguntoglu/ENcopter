import numpy as np
import rst

Bminus = [0, 0, 0, -1.788424918525570e+03]
Bplus = [1]
Aminus = [1, 39.999999999999990, -1.684671300000000e+03, -6.738685199999998e+04]
Aplus = [1]
Rd = [1, -1]
Sd = [1]
Acl_dash = [1, -1.704287577932423, 0.726149037073691, 0, 0, 0, 0]
Ao = [1, 0, 0, 0]

A = [1,39.999999999999990,-1.684671300000000e+03,-6.738685199999998e+04]
B = [0, 0, 0, -1.788424918525570e+03]

rst = rst.RST()
[R, S, T] = rst.rst_design(Bminus, Bplus, Aminus, Aplus, Rd, Sd, Acl_dash, Ao)

print("Closed-loop poles:")
Acl = rst.poly_add(np.convolve(R, A), np.convolve(B, S))
print(Acl)

