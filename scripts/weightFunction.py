import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np 
import math
from cmath import e
# thred_1=1.1
thred_1=2.9

thred_2=2

x = np.linspace(0, thred_1, 100)
x_2= np.linspace(thred_1, thred_2, 100)

cauchy = 1/(1+(x/thred_2)*(x/thred_2))
turkey = (1-x*x/(thred_2*thred_2))*(1-x*x/(thred_2*thred_2))
sigmoid_inverse =[2*(thred_2-(1/(1+math.exp(-4*i))))-4 for i in x]
sigmoid_inverse_2=[2*(thred_2-(1/(1+math.exp(-2*i))))-4 for i in x]
sigmoid_inverse_15=[2*(thred_2-(1/(1+math.exp(-1.5*i))))-4 for i in x]

error  = [0,    0.4,  0.5,   0.8,1.1]
weight = [1, 0.98,0.95, 0.8,0.2]
error_2=[1.1,  1.5,    2,     2.5,   3]
weight_2=[0.2,0.15, 0.1, 0.05, 0.01]
z1 = np.polyfit(error, weight,3) #用3次多项式拟合，输出系数从高到0
p1 = np.poly1d(z1) #使用次数合成多项式
z2 = np.polyfit(error_2, weight_2,3) #用3次多项式拟合，输出系数从高到0
p2 = np.poly1d(z1) #使用次数合成多项式
y_pre = p1(x)
poly_1=[z1[0]*i**3+z1[1]*i**2+z1[2]*i+z1[3] for i in x]
poly_2=[z2[0]*i**3+z2[1]*i**2+z2[2]*i+z2[3] for i in x_2]

# plt.plot(error,weight,'.')
# plt.plot(x,poly)
# plt.show()

print(z1)
print(z2)


# fig, ax = plt.subplots()
# ax.plot(x,cauchy, label='cauchy')
# ax.plot(x, turkey,label='turkey')
# ax.plot(x, sigmoid_inverse,label='sigmoid_inverse')
# ax.plot(x, sigmoid_inverse_2,label='sigmoid_inverse_2')
# ax.plot(x, sigmoid_inverse_15,label='sigmoid_inverse_15')
# ax.plot(x, poly_1,label='poly')
# ax.plot(x_2, poly_2,label='poly')

# ax.plot(error, weight,label='point')




# ax.legend()

threshold_outside=5
x_inverse=np.linspace(0, threshold_outside+3, 100)
a=threshold_outside/9
f_inverse= [a/(i+a) for i in x_inverse]
threshold_outside_=threshold_outside/2
f_tukey=[(1-i*i/(threshold_outside_*threshold_outside_))*(1-i*i/(threshold_outside_*threshold_outside_)) for i in x_inverse]
f_tukey_first=[ threshold_outside*threshold_outside/6*(1-(1-i*i/(threshold_outside*threshold_outside))**3) for i in x_inverse]
thres_inside=1
x_inside=np.linspace(-thres_inside,0, 100)
# 2*(k-(1/(1+std::exp(-1.5*r))))
f_inside=[2*(thres_inside-(1/(1+math.exp(-1.5*i)))) for i in x_inside]
f_inside_reverse=[thres_inside/(1.5*abs(i)+thres_inside) for i in x_inside]
plt.plot(x_inverse,f_inverse,label='f_inverse',color='orange')
# plt.plot(x_inverse,f_tukey,label='f_tukey',color='blue')

# plt.plot(x_inverse,f_tukey_first,label='f_tukey_first',color='green')
# plt.plot(x_inside,f_inside,label='f_inside',color='black')
plt.plot(x_inside,f_inside_reverse,label='f_inside',color='orange')
plt.xlabel("Distance to intersection")
plt.ylabel("Weight")
# plt.ylim((-1,2))
plt.show()