# -*- coding: utf-8 -*-
"""
Created on Sat Feb 27 21:00:14 2021

@author: JuanMC
"""

import vrep
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import math
from matplotlib.pyplot import figure
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches
import time
import pickle
X,Y,Z,HistPosPar,HistPosTarget,epocas,Number_Robots,Part_Enjambre=pickle.load(open("Caso2_Tunnel.p","rb"))

fig = plt.figure(figsize=(10,10))
ax = fig.add_subplot(1,1,1)
#ax.set_title("Swarm navigation and mean robot position")
ax.set_xlabel("X(m)")
ax.set_ylabel("Y(m)")
ax.set_xlim([-20,30])
ax.set_ylim([-50,28])


#ax.scatter(HistPosPar[0,1,:,0],HistPosPar[0,1,:,1], s=50, c='c', marker='o',zorder=10, label="Particles Init Position")
#ax.plot(HistPosTarget[0,:,0],HistPosTarget[0,:,1],zorder=5, c='k', linewidth=3, label="Mean Position")
#ax.scatter(HistPosPar[0,epocas-1,0,0],HistPosPar[0,epocas-1,0,1], s=50, c='g', marker='o', zorder=10, label="Particles Final Position")

ax.contour(X,Y,Z,25,linewidths=0.25,colors='k')
ax.contourf(X,Y,Z,25,cmap=cm.coolwarm,alpha=0.5)


for Robot in range(Number_Robots):
    ax.scatter(HistPosPar[Robot,1,:,0],HistPosPar[Robot,1,:,1], s=50, c='g', marker='o',zorder=100, label="Particles Init Position")
    ax.plot(HistPosTarget[Robot,:epocas-2000,0],HistPosTarget[Robot,:epocas-2000,1],zorder=25,c='k', linewidth=1,label="Robot Path")
    ax.scatter(HistPosPar[Robot,3700,:,0],HistPosPar[Robot,3700,:,1], s=150,c='r', marker='o', zorder=300, label="Particles Final Position")#c='#df6f2f'
    
    for particula in range(Part_Enjambre):
        ax.plot(HistPosPar[Robot,:epocas-2000,particula,0],HistPosPar[Robot,:epocas-2000,particula,1],c='#1166aa')#2737AE
    
    for Pos_Med in range(epocas):
        #print(Pos_Med)
        if Pos_Med % 600 == 0:            
            ax.scatter(HistPosTarget[Robot,Pos_Med,0],HistPosTarget[Robot,Pos_Med,1], s=50, c='c', marker='o', zorder=100)
        
ax.scatter(HistPosTarget[:,epocas-1,0],HistPosTarget[:,epocas-1,1], s=80, c='m', marker='o',zorder=200, label="Robot Final Position")       
ax.legend()       
ax.grid()



cont=0
distancias=np.zeros((epocas,20))
for pos in range(0,epocas,100):
    for robot in range(0,10):
        distancias[cont,robot]=np.linalg.norm(HistPosPar[0,pos,robot,:]-HistPosTarget[0,pos,:])
    cont=cont+1


plt.figure()
plt.plot(distancias[0:87,1])
plt.plot([0,87],[1,1],'--m')

plt.grid()
plt.xlabel('Epochs')
plt.ylabel('Distance(m)')
plt.ylim([0,5])


