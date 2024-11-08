import cubic_spline
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
import matplotlib.pyplot as plt 
import numpy as np
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
def turkey():
    plt.rc('font',family='Times New Roman')
    x =np.arange(-2,6,0.5)
    y=[]
    z=[]
    for i in np.arange(-2,6,0.5):
        if i<-1:
            y.append(0)
            z.append(0)
        elif i>5:
            z.append(0)
            y.append(0)
            
        elif  i>3:
            y.append(0)
            z.append(5/(9*i+5))
            
        elif  i<=0:
            y.append(1/(1.5*(-i)+1))
            z.append(1/(1.5*(-i)+1))
        else:
            y.append(3/(9*i+3))
            z.append(5/(9*i+5))
            
    x1= np.arange(-2,-0.9,1)
    y1=np.zeros_like(x1)
    
    x2= np.arange(-1,0.05,0.05)
    y2=[1/(1.5*(-i)+1) for i in x2]
    x3= np.arange(0,3,0.05)
    y3=[3/(9*i+3) for i in x3]
    
    x4= np.arange(3,5,1)
    y4=np.zeros_like(x4)
    plt.figure(dpi=300)
    # plt.scatter(x, y,marker='s',s=1)
    plt.plot(x1, y1,color="dodgerblue")
    plt.plot(x2, y2,color="dodgerblue")
    plt.plot(x3, y3,color="dodgerblue")
    plt.plot(x4, y4,color="dodgerblue")

    # plt.scatter(x, z,marker='s',s=1)

    # plt.plot(rx,ry)
    plt.xlabel("$sd_j$ [m]",fontsize=13)  
    plt.ylabel("Weight")
    # plt.legend()
    plt.show()
def corridor():
    x =np.arange(0,1,0.01)
    y=[]
    for i in np.arange(0,1,0.01):
        if i<0.5:
            y.append(1)
        else:
            y.append(10*i-4)
    
    plt.yticks(fontproperties = 'Times New Roman',fontsize=35)
    plt.xticks(fontproperties = 'Times New Roman',fontsize=35)
    
    plt.plot(x, y, linewidth=3)
    plt.xlabel("Corridorness  $Cor$",fontdict={'family' : 'Times New Roman','size' :33})  
    plt.ylabel("Downsample Rate  $R_{cor}$",fontdict={'family' : 'Times New Roman','size' :33})
    plt.xlim(0,1)
    ax = plt.gca()
    ax.set_aspect(0.15)
    plt.show()
    
    

if __name__ == '__main__':
    # corridor()
    turkey()