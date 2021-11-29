from actions.utilities import constants
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.patches as mpatches
import numpy as np

def close_all_figures():
    plt.close("all")

def generate_surface(pos_obstacle,
                     pos_victim,
                     pos_robot):
    
    fig = plt.figure()
    fig.set_size_inches(10, 5)
    ax = fig.gca(projection='3d')
    X = np.arange(constants.x_min, constants.x_max, 0.1)
    Y = np.arange(constants.y_min, constants.y_max, 0.1)
    X, Y = np.meshgrid(X, Y)
    
    Z = 0
    
    Z = add_obstacles_to_surface(X,Y,Z, pos_obstacle)
    Z = add_victims_to_surface(X,Y,Z, pos_victim)
    ax.plot_surface(X,
                Y,
                Z,
                cmap=cm.coolwarm,
                linewidth=1,
                antialiased=False,
                alpha=0.8)

    ax.contour(X, Y, Z, zdir='z', offset=-1.5, cmap=cm.coolwarm)
    add_robots_to_surface(ax, pos_robot)
    plt.show()
    return X,Y,Z

def makeGaussian(X,
                 Y,
                 Amp,
                 CenterX,
                 CenterY,
                 DX,
                 DY,
                 Slope):
    
    return Amp*np.exp(-Slope*np.log(2) * (((X-CenterX)**2)/ DX**2 + ((Y-CenterY)**2)/ DY**2) )



def add_obstacles_to_surface(X,
                             Y,
                             Z,
                             pos_obstacle):
    
    for obstacle in range(constants.Number_Obstacles):
        Z_obstacle = makeGaussian(X,
                                  Y,
                                  constants.Alt_Obsta,
                                  pos_obstacle[obstacle][0],
                                  pos_obstacle[obstacle][1],
                                  2,
                                  2,
                                  constants.Std_Obsta)        
        Z = Z_obstacle + Z
    return Z

def add_victims_to_surface(X,
                           Y,
                           Z,
                           pos_victim):
    
    for victim in range(constants.Number_Victims):
        Z_victim = makeGaussian(X,
                                Y,
                                constants.Alt_Victim,
                                pos_victim[victim][0],
                                pos_victim[victim][1],
                                2,
                                2,
                                constants.Std_Victim)  #  Victima atractor
        Z = Z_victim + Z
    return Z

def add_robots_to_surface(ax, pos_robot):
    for Robot in range(constants.Number_Robots):
        ax.scatter(pos_robot[Robot][0],
                   pos_robot[Robot][1],
                   0.5,
                   s=100,
                   c='r',
                   marker="o")

def generate_contour(X,Y,Z):
    plt.figure()
    plt.contour(X,
                Y,
                Z,
                15,
                linewidths=0.25,
                colors='k')
    
    
    CS = plt.contourf(X,
                      Y,
                      Z,
                      15,
                      cmap=cm.coolwarm)
    
    plt.show()
    
    plt.figure(2)
    plt.title("Repulsion")
    Part_Legend = mpatches.Patch(color='orange', label='Particulas')
    Enja_Legend = mpatches.Patch(color='blue', label='Enjambres')
    plt.legend(handles=[Part_Legend,Enja_Legend])
    Espacio = np.arange(-20,20,0.1)
    Repulsion_Enjambre_plt = 1/(1+((abs(((Espacio) - 0 )
                                        /(constants.Dist_Enjambre)))**
                                   (2*constants.Pendiente_Enjambre)))
    
    Repulsion_Particula_plt = 1/(1+((abs(((Espacio) - 0 )
                                         /(constants.Dist_Particula)))**
                                    (2*constants.Pendiente_Particula)))
    
    plt.plot(Espacio,
             Repulsion_Enjambre_plt,
             'b')
    
    plt.plot(Espacio,
             Repulsion_Particula_plt,
             'orange')
    
    
    
    
def results(HistVal, HistPosPar, HistPosTarget, X, Y, Z):
    plt.figure()
    for Robot in range(constants.Number_Robots):
        plt.plot(HistVal[Robot])  
    plt.show()


    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(1,1,1)
    ax.set_title("Swarm navigation and mean robot position")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")


    ax.scatter(HistPosPar[0,1,:,0],
               HistPosPar[0,1,:,1],
               s=50,
               c='r',
               marker='o',
               zorder=10,
               label="Particles Init Position")


    ax.plot(HistPosTarget[0,:,0],
            HistPosTarget[0,:,1],
            zorder=5,
            c='k',
            linewidth=1.5,
            label="Mean Position")

    ax.scatter(HistPosPar[0,constants.epocas-1,0,0],
               HistPosPar[0,constants.epocas-1,0,1],
               s=50,
               c='k',
               marker='o',
               zorder=10,
               label="Particles Final Position")


    ax.contour(X,
               Y,
               Z,
               25,
               linewidths = 0.25,
               colors='k')

    ax.contourf(X,
                Y,
                Z,
                25,
                cmap=cm.coolwarm,
                alpha=0.5)



    for Robot in range(constants.Number_Robots):
        ax.scatter(HistPosPar[Robot,1,:,0],
                   HistPosPar[Robot,1,:,1],
                   s=50,
                   c='r',
                   marker='o',
                   zorder=10)
        
        ax.plot(HistPosTarget[Robot,:,0],
                HistPosTarget[Robot,:,1],
                zorder=25,
                c='k',
                linewidth=1.5)
        
        ax.scatter(HistPosPar[Robot,constants.epocas-1,:,0],
                   HistPosPar[Robot,constants.epocas-1,:,1],
                   s=50,
                   c='k',
                   marker='o',
                   zorder=10)
        
        #ax.plot(HistPosPar[Robot,:,:,0],HistPosPar[Robot,:,:,1])
        
        for particula in range(constants.Part_Enjambre):
            ax.plot(HistPosPar[Robot,:,particula,0],
                    HistPosPar[Robot,:,particula,1],
                    c='#2737AE')
            
        
        for Pos_Med in range(constants.epocas):
            print(Pos_Med)
            if Pos_Med % 200 == 0:            
                ax.scatter(HistPosTarget[Robot,Pos_Med,0],
                           HistPosTarget[Robot,Pos_Med,1],
                           s=10,
                           c='w',
                           marker='o',
                           zorder=10)
                
            
    ax.scatter(HistPosTarget[:,constants.epocas-1,0],
               HistPosTarget[:,constants.epocas-1,1],
               s=10,
               c='b',
               marker='o',
               zorder=20,
               label="Mean Final Position")  
         
    ax.legend()       
    ax.grid()
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(1,1,1)
    ax.contour(X,
               Y,
               Z,
               25,
               linewidths=0.25,
               colors='k')
    
    ax.contourf(X,
                Y,
                Z,
                25,
                cmap=cm.coolwarm,
                alpha=0.5)
    
    
    
    ax.scatter(HistPosTarget[:,1,0],
               HistPosTarget[:,1,1],
               s=50,
               c='b',
               marker='o')
    
    for Robot in range(constants.Number_Robots):
        ax.plot(HistPosTarget[Robot,:,0],HistPosTarget[Robot,:,1])
        
    ax.scatter(HistPosTarget[:,constants.epocas-1,0],
               HistPosTarget[:,constants.epocas-1,1],
               s=50,
               c='r',
               marker='o')
    
    ax.grid()
