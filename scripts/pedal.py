from sympy import *
import matplotlib.pyplot as plt 
import numpy as np
import math
x1= symbols('x1')
y1 = symbols('y1')
x2= symbols('x2')
y2 = symbols('y2')
x3= symbols('x3')
y3 = symbols('y3')
x4= symbols('x4')
y4 = symbols('y4')

f1=(x3-x4)*(x1-x2)+(y3-y4)*(y1-y2)
f2=(y2-y4)/(x2-x4)-(y4-y1)/(x4-x1)
c=solve([f1,f2],[x4,y4])
# # print("x4= ",c)
# # print("y4 = ",y4)
# # print(pretty(c[0]))
# print((c[0]))
# print((c[1]))
print(c)

# {x4: (x1**2*x3 - 2*x1*x2*x3 - x1*y1*y2 + x1*y1*y3 + x1*y2**2 - x1*y2*y3 + x2**2*x3 + x2*y1**2 - x2*y1*y2 - x2*y1*y3 + x2*y2*y3)/(x1**2 - 2*x1*x2 + x2**2 + y1**2 - 2*y1*y2 + y2**2), y4: (x1**2*y2 - x1*x2*y1 - x1*x2*y2 + x1*x3*y1 - x1*x3*y2 + x2**2*y1 - x2*x3*y1 + x2*x3*y2 + y1**2*y3 - 2*y1*y2*y3 + y2**2*y3)/(x1**2 - 2*x1*x2 + x2**2 + y1**2 - 2*y1*y2 + y2**2)}

