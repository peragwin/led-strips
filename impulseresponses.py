import math
import numpy as np
import matplotlib.pyplot as plt

length = 128

def U(n1, n2):
   def u(x):
      if x >= n1 and x < n2:
         return 1.0
      else:
         return 0
   return u

def x(n):
   return math.sin(2*math.pi*n/12)

"""
poles = [-.5]
zeros = [1] # first one is gain

X = u
Y = np.zeros(length)
Yl = np.zeros(length)
Yd = np.zeros(length)
y = 0.0
longy = 0.0
diffy = 0.0
sdy = 0.0
for r in range(length):
   ny = 0.0
   for i, p in enumerate(poles):
      ny -= p * y #Y[r - (i+1)]
   for i, z in enumerate(zeros):
      ny += z * X(r-i)
   longy = ny*.15 + .85*longy - .05
   Yl[r] = longy
   ny -= longy
   Y[r] = ny

   diff = ny - y
   diffy = diff * .5 + .5*diffy
   Yd[r] = diffy
   sdy += diffy

   y = ny


plt.plot(Y)
plt.plot(Yl)
plt.plot(Yd)
print(sdy)
"""

# void applyIIRFilter(
#   float (*filterEq)[2], // [filterOrder] x [2]
#   float input,
#   float *output, // [filterOrder]
#   int filterOrder,
#   float *dInput
# ) {
#   int order;
#   float ae;
#   for (order = 0; order < filterOrder; order++) {
#     ae = filterEq[order][0] * input + filterEq[order][1] * output[order];
#     if (dInput != NULL)
#       dInput[order] = ae - output[order];

#     input = ae;
#     output[order] = ae;
#   }
#   for (order = filterOrder - 1; order > 0; order--) {
#     output[order - 1] += output[order] - 0.0001; // correct for fp precision in negative feedback
#   }
# }

def applyIIRFilter(filterParams, input, output, order, dInput=None):
   fp = filterParams
   for o in range(order):
      ae = fp[o][0] * input + fp[o][1] * output[o]
      if dInput is not None:
         dInput[o] = ae - output[o]

      input = ae
      output[o] = ae

   for o in range(order-1, 0, -1):
     output[o-1] += output[o] - 0.0001

filterParams = [
   [ 0.5,  0.5 ],
   [ -.05, .95 ],
   [ -0.01, 0 ],
]

Y = np.zeros(length)
Yl = np.zeros(length)
Yll = np.zeros(length)
X = U(2, 64)
output = [0,0,0]

for r in range(length):
   applyIIRFilter(filterParams, X(r), output, 3)
   Y[r] = output[0]
   Yl[r] = output[1]
   Yll[r] = output[2]

plt.plot(Y)
plt.plot(Yl)
plt.plot(Yll)
plt.show()