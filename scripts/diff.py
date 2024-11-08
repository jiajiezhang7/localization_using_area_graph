from sympy import *
import matplotlib.pyplot as plt 
import numpy as np
import math
x = symbols('x')
y = symbols('y')
p1x= symbols('p1x')
p1y= symbols('p1y')
p2x= symbols('p2x')
p2y= symbols('p2y')
p3x= symbols('p3x')
p3y= symbols('p3y')
p4x= symbols('p4x')
p4y= symbols('p4y')
constant_x= symbols('constant_x')
constant_y= symbols('constant_y')
p2x=p1x+constant_x
p2y=p1y+constant_y

g=y*x**2
#  intersectionx=(p3y*p4x*p2x-p4y*p3x*p2x-p3y*p4x*p1x+p4y*p3x*p1x-p1y*p2x*p4x+p2y*p1x*p4x+p1y*p2x*p3x-p2y*p1x*p3x)/(p4x*p2y-p4x*p1y-p3x*p2y+p3x*p1y-p2x*p4y+p2x*p3y+p1x*p4y-p1x*p3y+0000000000001);
#  intersectiony=(-p3y*p4x*p2y+p4y*p3x*p2y+p3y*p4x*p1y-p4y*p3x*p1y+p1y*p2x*p4y-p1y*p2x*p3y-p2y*p1x*p4y+p2y*p1x*p3y)/(p4y*p2x-p4y*p1x-p3y*p2x+p1x*p3y-p2y*p4x+p2y*p3x+p1y*p4x-p1y*p3x+0000000000001);
intersectionx=(p3y*p4x*p2x-p4y*p3x*p2x-p3y*p4x*p1x+p4y*p3x*p1x-p1y*p2x*p4x+p2y*p1x*p4x+p1y*p2x*p3x-p2y*p1x*p3x)/(p4x*p2y-p4x*p1y-p3x*p2y+p3x*p1y-p2x*p4y+p2x*p3y+p1x*p4y-p1x*p3y);
intersectiony=(-p3y*p4x*p2y+p4y*p3x*p2y+p3y*p4x*p1y-p4y*p3x*p1y+p1y*p2x*p4y-p1y*p2x*p3y-p2y*p1x*p4y+p2y*p1x*p3y)/(p4y*p2x-p4y*p1x-p3y*p2x+p1x*p3y-p2y*p4x+p2y*p3x+p1y*p4x-p1y*p3x);
print(intersectionx.diff('p1x'))
print(intersectionx.diff('p1y'))
print(intersectiony.diff('p1x'))
print(intersectiony.diff('p1y'))

# (p1y*p3x - p1y*p4x - p3x*(constant_y + p1y) + p4x*(constant_y + p1y))/(-p1x*p3y + p1x*p4y + p1y*p3x - p1y*p4x - p3x*(constant_y + p1y) + p3y*(constant_x + p1x) + p4x*(constant_y + p1y) - p4y*(constant_x + p1x))
# (-p1x*p3x + p1x*p4x + p3x*(constant_x + p1x) - p4x*(constant_x + p1x))/(-p1x*p3y + p1x*p4y + p1y*p3x - p1y*p4x - p3x*(constant_y + p1y) + p3y*(constant_x + p1x) + p4x*(constant_y + p1y) - p4y*(constant_x + p1x))
# (-p1y*p3y + p1y*p4y + p3y*(constant_y + p1y) - p4y*(constant_y + p1y))/(p1x*p3y - p1x*p4y - p1y*p3x + p1y*p4x + p3x*(constant_y + p1y) - p3y*(constant_x + p1x) - p4x*(constant_y + p1y) + p4y*(constant_x + p1x))
# (p1x*p3y - p1x*p4y - p3y*(constant_x + p1x) + p4y*(constant_x + p1x))/(p1x*p3y - p1x*p4y - p1y*p3x + p1y*p4x + p3x*(constant_y + p1y) - p3y*(constant_x + p1x) - p4x*(constant_y + p1y) + p4y*(constant_x + p1x))
