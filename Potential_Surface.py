# -*- coding: utf-8 -*-
"""
Created on Sun May 24 03:05:24 2020

@author: JuanMC
"""


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

def makeGaussian(X,Y,Amp,CenterX,CenterY,DX,DY,Slope):
    return Amp*np.exp(-Slope*np.log(2) * (((X-CenterX)**2)/ DX**2 + ((Y-CenterY)**2)/ DY**2) )


#g1=makeGaussian(20, 2, [10,10])

x_min=-25
x_max=25
y_min=-25
y_max=25


fig = plt.figure()
fig.set_size_inches(10, 5)
ax = fig.gca(projection='3d')


# Make data.
X = np.arange(x_min, x_max, 0.5)
Y = np.arange(y_min,y_max, 0.5)
X, Y = np.meshgrid(X, Y)


R1 =makeGaussian(X,Y,1,0,0,2,2,0.8)
R2 =makeGaussian(X,Y,-1,10,10,5,5,0.1)
R3 =makeGaussian(X,Y,0,-10,10,5,5,0.5)
Z=R1+R2+R3

# Plot the surface.
ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,linewidth=1, antialiased=False,alpha=0.8)
#ax.plot_wireframe(X, Y, Z, rstride=2, cstride=2)
ax.contour(X, Y, Z, zdir='z', offset=-15, cmap=cm.coolwarm)

PosRobot=[-20, -20]
step=2
angles=np.arange(0,2*np.pi,np.pi/16)

xRobot=np.sin(angles)*step+PosRobot[0]
yRobot=np.cos(angles)*step+PosRobot[1]


plt.figure()
plt.plot(xRobot,yRobot,'o')


R1 =makeGaussian(xRobot,yRobot,1,0,0,5,5,0.8)
R2 =makeGaussian(xRobot,yRobot,-1,10,10,5,5,0.1)
R3 =makeGaussian(xRobot,yRobot,0,-10,10,5,5,0.5)
ZR=R1+R2+R3
posMin=np.where(np.min(ZR)==ZR)
DirXRob=xRobot[posMin]
DirYRob=yRobot[posMin]

print(ZR)
print(np.where(np.min(ZR)==ZR))
print(np.min(ZR))
print(xRobot)
print(DirXRob)
print(DirYRob)
