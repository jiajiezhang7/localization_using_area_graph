import cubic_spline
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
import matplotlib.pyplot as plt 
import numpy as np
import math
# error  = [0,    0.4,  0.5,   0.8,  1.1,  1.5,    2,     2.5,   3]
# weight = [1, 0.98,0.95, 0.8,  0.2,  0.15, 0.1, 0.05, 0.01]
# spline = cubic_spline.Spline(error,weight)
# rx = np.arange(0,3,0.01)
# ry = [spline.calc(i) for i in rx]
# plt.plot(error,weight,"og")
# plt.plot(rx,ry,"-r")
# plt.grid(True)
# plt.show()



# corridorness=[0,0.2,0.5,0.8,1.0]
# downSampleRate=[0,0.05,2,5,6]
# spline = cubic_spline.Spline(corridorness,downSampleRate)
# rx = np.arange(0,1,0.01)
# ry = [spline.calc(i) for i in rx]
# print(spline.a,spline.b,spline.c,spline.d)
# plt.plot(corridorness,downSampleRate,"og")
# plt.plot(rx,ry,"-r")
# plt.grid(True)
# plt.show()

import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
#  = np.array([0,0.2,0.6,0.8,1.0])  # sort data points by increasing x value
# y = np.array([0,0.05,1.0,3,5])
x=[0,0.2,0.5,0.8,1.0]
y=[0,0.05,1.5,5,6]
# x = np.array([0,0.2,0.5,0.8,1.0])  # sort data points by increasing x value
# y = np.array([0,0.05,1.5,5,6])
arr = np.arange(np.amin(x), np.amax(x), 0.0001)
s = interpolate.CubicSpline(x, y)
print(s.c)
rx = np.arange(0.2,0.5,0.01)
ry = [s.c[0,1]*(i-0.2)**3+s.c[1,1]*(i-0.2)*(i-0.2)+s.c[2,1]*(i-0.2)+s.c[3,1] for i in rx]


# plt.plot(x, y, 'bo', label='Data Point')
# plt.plot(arr, s(arr), 'r-', label='Cubic Spline')
y_=s(arr)
for i in range(y_.shape[0]):
    if y_[i]<1:
        y_[i]=1
        
plt.plot(arr,y_)

# plt.plot(rx,ry)
plt.xlabel("Corridorness")  
plt.ylabel("DownSampleRate")
# plt.legend()
plt.show()

maxPercentage = symbols('maxPercentage')
expression1=12.88194444*(maxPercentage-0)*(maxPercentage-0)*(maxPercentage-0)+0.14930556*(maxPercentage-0)*(maxPercentage-0)+-0.29513889*(maxPercentage-0)+0.00000000
s_expr1=simplify(expression1)
expression2=- 12.88194444*(maxPercentage-0.2)*(maxPercentage-0.2)*(maxPercentage-0.2)+7.87847222*(maxPercentage-0.2)*(maxPercentage-0.2)+1.31041667*(maxPercentage-0.2)+ 0.05
s_expr2=simplify(expression2)
expression3=-41.00694444*(maxPercentage-0.5)*(maxPercentage-0.5)*(maxPercentage-0.5)+19.47222222*(maxPercentage-0.5)*(maxPercentage-0.5)+9.515625*(maxPercentage-0.5)+1.5
s_expr3=simplify(expression3)
expression4=-41.00694444*(maxPercentage-0.8)*(maxPercentage-0.8)*(maxPercentage-0.8)-17.43402778*(maxPercentage-0.8)*(maxPercentage-0.8)+10.12708333*(maxPercentage-0.8)+5
s_expr4=simplify(expression4)
print(s_expr1)
print(s_expr2)
print(s_expr3)
print(s_expr4)
