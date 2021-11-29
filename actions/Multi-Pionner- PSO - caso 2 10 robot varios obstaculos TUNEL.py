# -*- coding: utf-8 -*-
"""
Created on Tue May 12 19:25:54 2020

@author: Brian David Noriega
         Juan Pablo Romero Camacho
         David Paez Ramirez
        
        
MINIMO GLOBAL QUE MUEVE A LOS SWARM HACIA EL HORIZONTE    
    
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


# ---------------------------------- Clase Particula ----------------------------------
 
class Particula(object):
    def __init__(self,x,v):
        
        
        
        self.posicion = x
        self.velocidad = v
        self.mejorValor = np.inf
        self.mejorPosicion = x
        self.repulsionPart = np.zeros(3)
        self.repulsionSwr = np.zeros(3)
        self.valor = 0
        
        
    def evaluar(self, funcion):        
        valor = funcion(self.posicion)
        if valor < self.mejorValor:
            self.mejorValor=valor
            self.mejorPosicion=self.posicion
        self.valor=valor
        
        
    def actualizarVel(self, bestPosSwr, C):
        Pos = self.posicion
        PosSwrm = bestPosSwr
        PosPart = self.mejorPosicion
        Norm_PosSwrm = normalizar(PosSwrm,Pos)
        Norm_PosPart = normalizar(PosPart,Pos)       
        
        for i in range(len(bestPosSwr)):
             self.velocidad[i]= C[0]*self.velocidad[i]  +  C[1]*((np.random.random()/4)+0.75)*(Norm_PosPart[i])  + C[2]*((np.random.random()/4)+0.75)*(Norm_PosSwrm[i])  + C[3]*((np.random.random()/4)+0.75)*(self.repulsionPart[i]) + C[4]*((np.random.random()/4)+0.75)*(self.repulsionSwr[i])
             
    def actualizarPos(self,dt):
        self.posicion = self.posicion + np.array(self.velocidad)*dt




# --------------------------------- Funciones del Algoritmo -----------------------------------------
        

def normalizar(Vect,Pos):
    Vector = np.zeros(3)
    if Pos[0] == Vect[0] and Pos[1] == Vect[1]:
        return [0,0,0]
    else:
        Norm = math.sqrt((Vect[0] - Pos[0])**2+(Vect[1] - Pos[1])**2+(Vect[2] - Pos[2])**2)
        
        Vector = np.zeros(3)
        Vector[0] = (Vect[0] - Pos[0])/Norm
        Vector[1] = (Vect[1] - Pos[1])/Norm
        Vector[2] = (Vect[2] - Pos[2])/Norm
        return Vector
    
    

# --------------------------------- Funcion de Evaluacion ----------------------------------------------------




def funEval3D(posicion):
    xRobot = posicion[0]
    yRobot = posicion[1]
    
    """
    victim = [] # lista de victimas
    pos_victim = [] # Lista de posiciones de la victima
    obstacle = [] # Lista de Obstaculos
    pos_obstacle = [] # Lista de posiciones de los Obstaculos
    """
    z_ = 0
    zv_ = 0
    
    Z_Global_ = makeGaussian(xRobot,yRobot,-1,0,45,200,40,0.5)
    for obstacle in range(Number_Obstacles):
        #Leer de Vrep la Posicion de los Obstaculos
        """
        _obstacle_name = obstacle_name + '#' + str(Obstacle)
        errorCode, Fire_handle =vrep.simxGetObjectHandle(clientID,_obstacle_name,vrep.simx_opmode_oneshot_wait)
        errorCode, PosFire = vrep.simxGetObjectPosition(clientID,Fire_handle,-1,vrep.simx_opmode_oneshot)    
        obstacle.append(Fire_handle)
        pos_obstacle.append(PosFire)
        """
        
        
        R_Obstacle = makeGaussian(xRobot,yRobot,Alt_Obsta,pos_obstacle[obstacle][0],pos_obstacle[obstacle][1],Std_Obsta,Std_Obsta,Slope_Obsta)
        #R_Obstacle = makeGaussian(xRobot,yRobot,Alt_Obsta,pos_obstacle[obstacle][0],pos_obstacle[obstacle][1],0.1,0.2,10)
        
        z_ = R_Obstacle + z_
    for victim in range(Number_Victims):
        #Leer de Vrep la Posicion de las Victimas
        """
        _victim_name = victim_name + '#' + str(Victim)
        errorCode, Victim_handle =vrep.simxGetObjectHandle(clientID,_victim_name,vrep.simx_opmode_oneshot_wait)
        errorCode, PosVictim = vrep.simxGetObjectPosition(clientID,Victim_handle,-1,vrep.simx_opmode_oneshot)    
        victim.append(Victim_handle)
        pos_victim.append(PosVictim)
        """   
        
        R_Victim = makeGaussian(xRobot,yRobot,Alt_Victim,pos_victim[victim][0],pos_victim[victim][1],Std_Victim,Std_Victim,Slope_Victim) #  Victima atractor
        zv_ =np.minimum(R_Victim, zv_)
    z_=zv_+z_+Z_Global_
        
    return z_


def makeGaussian(X,Y,Amp,CenterX,CenterY,DX,DY,Slope):
    return Amp*np.exp(-Slope*np.log(2) * (((X-CenterX)**2)/ DX**2 + ((Y-CenterY)**2)/ DY**2) )



# ----------------------------- Funcion para el movimiento del Robot ------------------------------------------



def goTo(robot, target, RobAng):
    Kr=10  #20
    Kp=4   #8
    i=0.5  # Integrador Falso
    MaxVel=12
    dist= ((target[0]-robot[0])**2 + (target[1]-robot[1])**2)**0.5
    TarRobAng =np.arctan2((target[1]-robot[1]), (target[0]-robot[0]))
   
    if abs(TarRobAng-RobAng)>np.pi:
        Rotacion=(TarRobAng-RobAng)-2*np.pi*np.sign(TarRobAng-RobAng)
    else:
        Rotacion=(TarRobAng-RobAng)
        
    LeftWheel=-Kr*(Rotacion/np.pi)+Kp*dist +i
    RightWheel=Kr*(Rotacion/np.pi)+Kp*dist +i
    
    if np.abs(LeftWheel) > MaxVel:
        RightWheel=  (RightWheel/np.abs(LeftWheel))*MaxVel
        LeftWheel=np.sign(LeftWheel)*MaxVel
    if np.abs(RightWheel) > MaxVel:
        LeftWheel=  (LeftWheel/np.abs(RightWheel))*MaxVel
        RightWheel=np.sign(RightWheel)*MaxVel
    return LeftWheel, RightWheel, dist



 # ----------------- Conexion con Vrep ---------------
 
 

vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print("Conexion no exitosa")
    sys.exit("No se pudo conectar")
time.sleep(1)




# --------------- Paramatros de configuracion del PSO ----------------



Number_Robots = 10 #Numero de Robots
Number_Victims = 1 #Numero de Victima        Bill#0 y Bill#1
Number_Obstacles = 10 #Numero de Obstaculos  Fire#0 a Fire#11
Part_Enjambre = 10# Particulas por Enjambre

C = [0.1,0.3,1.5,0.5,3] # Coeficientes PSO   #  0=velocdad  1=particula   2=swarm   3=repulsion part   4= repulsion swarm
dt = 0.2#0.2 # Paso o Step en la actualizacion PSO

Dist_Enjambre = 4 # 3  Distancia Minima - Desviacion estandar
Dist_Particula = 0.7#0.7   Distancia Minima - Desviacion estandar   
Vect_DistPar = np.zeros(Number_Robots) + Dist_Particula
Vect_Guardar = np.zeros(Number_Robots) + Dist_Particula

Pendiente_Enjambre = 5 # Pendiente, Ganancia 
Pendiente_Particula = 10 # Pendiente, Ganancia

Alt_Victim = -6 #-2,  -8  # Altura de la Gaussiana en la construccion de la Superficie - Victimas
Alt_Obsta = 12 #4   # Altura de la Gaussiana en la construccion de la Superficie - Obstaculos

Std_Victim = 10   # 10 Grosor de la Gaussiana en la construccion de la Superficie - Victimas
Std_Obsta = 8 #4   # Grosor de la Gaussiana en la construccion de la Superficie - Obstaculos

Slope_Obsta=30  #10
Slope_Victim=1

x_min=y_min = -50 # Dimensiones del area de busqueda
x_max=y_max = 50 
epocas = 6000 # Numero de Iteraciones
mejorValEnj = 0 # Mejor Valor del enjambre


# --------------------- Funciones de Repulsion Enjambre y Particula ----------------------------------------  
    
def funRepulsionSwr(MejoresPosiciones,Enjambre,particula):
    x_ = np.zeros(len(MejoresPosiciones))
    y_ = np.zeros(len(MejoresPosiciones))
    z_ = np.zeros(len(MejoresPosiciones))
    
    for enjambre_ in range(len(MejoresPosiciones)):
        if enjambre_ == Enjambre:
            
            x_[enjambre_] = 0
            y_[enjambre_] = 0
            z_[enjambre_] = 0
            
        else:
            distancia_minima_ = Dist_Enjambre
            pendiente_ = Pendiente_Enjambre
            
            distancia_ = math.sqrt((MejoresPosiciones[enjambre_][0] - particula.posicion[0])**2+(MejoresPosiciones[enjambre_][1] - particula.posicion[1])**2+(MejoresPosiciones[enjambre_][2] - particula.posicion[2])**2)
            Repulsion_Gaussiana_ = 1/(1+((abs(((distancia_) - 0 )/(distancia_minima_)))**(2*pendiente_)))
            
            nVx = (MejoresPosiciones[enjambre_][0] - particula.posicion[0])/distancia_
            nVy = (MejoresPosiciones[enjambre_][1] - particula.posicion[1])/distancia_
            #nVz = (MejoresPosiciones[enjambre_][2] - particula.posicion[2])/distancia_
            
            fVx = - Repulsion_Gaussiana_ * nVx
            fVy = - Repulsion_Gaussiana_ * nVy
            #fVz = - Repulsion_Gaussiana_ * nVz
            
            x_[enjambre_] = fVx
            y_[enjambre_] = fVy
            z_[enjambre_] = 0
    
            
    x_ = np.sum(x_)
    y_ = np.sum(y_)
    z_ = np.sum(z_)                   
    
    return[x_,y_,z_]




        
def funRepulsionPart(particulas,particula, distancia_minima, indexBestPart, indexActPart):
    x_ = np.zeros(len(particulas))
    y_ = np.zeros(len(particulas))
    z_ = np.zeros(len(particulas))

    if distancia_minima != Dist_Particula and indexBestPart == indexActPart:
        return [0,0,0]
    else:
        for part in particulas:
            if part.posicion[0] == particula.posicion[0] and part.posicion[1] == particula.posicion[1] and part.posicion[2] == particula.posicion[2]:
                x_[particulas.index(part)] = 0
                y_[particulas.index(part)] = 0
                z_[particulas.index(part)] = 0
                
    
                
    
            else:
                #distancia_minima = Dist_Particula
                pendiente = Pendiente_Particula
                
                distancia = math.sqrt((particula.posicion[0]- part.posicion[0])**2 + (particula.posicion[1] - part.posicion[1])**2 +(particula.posicion[2] - part.posicion[2])**2)
                Repulsion_Gaussiana = 1/(1+((abs(((distancia) - 0 )/(distancia_minima)))**(2*pendiente)))
                            
                nVx = (part.posicion[0] - particula.posicion[0])/distancia
                nVy = (part.posicion[1] - particula.posicion[1])/distancia
                #nVz = (part.posicion[2] - particula.posicion[2])/distancia
                
                fVx = - Repulsion_Gaussiana * nVx
                fVy = - Repulsion_Gaussiana * nVy
                #fVz = - Repulsion_Gaussiana * nVz
                
                x_[particulas.index(part)] = fVx
                y_[particulas.index(part)] = fVy
                z_[particulas.index(part)] = 0
                
    
        x_ = np.sum(x_)
        y_ = np.sum(y_)
        z_ = np.sum(z_)
        
        
                
        return [x_,y_,z_]


# ------------------------- Configuración de Elementos de Vrep ---------------------------


left_motor_name = 'Pioneer_p3dx_leftMotor'
right_motor_name = 'Pioneer_p3dx_rightMotor'
visible_name = 'Pioneer_p3dx_visible'
victim_name = 'Bill'
plane_name = 'Plane'
obstacle_name = 'fire'
planetarget_name = 'Plane_Target'


left_motor_handle = [] # Lista de Velocidades del motor izquierdo
right_motor_handle = [] # Lista de Velocidades del motor derecho
visible_handle = [] # Lista de Pioneers
pos_robot = [] # Lista de posiciones del robot
target_plane = [] # Lista de posiciones del target
victim = [] # lista de victimas
pos_victim = [] # Lista de posiciones de la victima
obstacle = [] # Lista de Obstaculos
pos_obstacle = [] # Lista de posiciones de los Obstaculos
planes = [] # Lista de planos


# ------------------------ Conexion Dinamica de Elementos Vrep -------------------------------


for Robot in range(Number_Robots):
    _left_motor = left_motor_name + '#' + str(Robot)
    _right_motor = right_motor_name + '#' + str(Robot)
    _visible = visible_name + '#' + str(Robot)
    _planetarget = planetarget_name + '#' + str(Robot)     
    
    errorCode, left_motor_handle_ = vrep.simxGetObjectHandle(clientID,_left_motor,vrep.simx_opmode_oneshot_wait)
    errorCode, right_motor_handle_ = vrep.simxGetObjectHandle(clientID,_right_motor,vrep.simx_opmode_oneshot_wait)
    errorCode, pioneer_handle_ = vrep.simxGetObjectHandle(clientID,_visible,vrep.simx_opmode_oneshot_wait)
    errorCode, target_handle = vrep.simxGetObjectHandle(clientID,_planetarget,vrep.simx_opmode_oneshot_wait)
    errorCode, PosRobot  = vrep.simxGetObjectPosition(clientID,pioneer_handle_,-1,vrep.simx_opmode_oneshot_wait) #vrep.simx_opmode_blocking    
    
    
    
    left_motor_handle.append(left_motor_handle_)
    right_motor_handle.append(right_motor_handle_)
    visible_handle.append(pioneer_handle_)
    pos_robot.append(PosRobot)
    #target_plane.append(target_handle)
    """
    for Plane in range(Part_Enjambre):
        _plane = plane_name + str( Plane + (Part_Enjambre * Robot))
        errorCode, plane_handle = vrep.simxGetObjectHandle(clientID,_plane,vrep.simx_opmode_oneshot_wait)
        planes.append(plane_handle)
    """
    
for Victim in range(Number_Victims):
    _victim_name = victim_name + '#' + str(Victim)
    errorCode, Victim_handle =vrep.simxGetObjectHandle(clientID,_victim_name,vrep.simx_opmode_oneshot_wait)#vrep.simx_opmode_blocking
    errorCode, PosVictim = vrep.simxGetObjectPosition(clientID,Victim_handle,-1,vrep.simx_opmode_oneshot_wait)#vrep.simx_opmode_blocking
    
    victim.append(Victim_handle)
    pos_victim.append(PosVictim)

for Obstacle in range(Number_Obstacles):
    _obstacle_name = obstacle_name + '#' + str(Obstacle)
    errorCode, Fire_handle =vrep.simxGetObjectHandle(clientID,_obstacle_name,vrep.simx_opmode_oneshot_wait)
    errorCode, PosFire = vrep.simxGetObjectPosition(clientID,Fire_handle,-1,vrep.simx_opmode_oneshot_wait)#vrep.simx_opmode_blocking
    
    obstacle.append(Fire_handle)
    pos_obstacle.append(PosFire)
    


# --------------------- Plot de la Superficie Resultante ------------------------------------

plt.close("all")

fig = plt.figure()
fig.set_size_inches(10, 5)
ax = fig.gca(projection='3d')
X = np.arange(x_min, x_max, 0.1)
Y = np.arange(y_min,y_max, 0.1)
X, Y = np.meshgrid(X, Y)

Z = 0
Zv = 0


Z_Global = makeGaussian(X,Y,-10,0,50,200,40,0.5)

for obstacle in range(Number_Obstacles):
    Z_obstacle = makeGaussian(X,Y,Alt_Obsta,pos_obstacle[obstacle][0],pos_obstacle[obstacle][1],Std_Obsta,Std_Obsta,Slope_Obsta)
    Z = Z_obstacle + Z    
for victim in range(Number_Victims):
    Z_victim = makeGaussian(X,Y,Alt_Victim,pos_victim[victim][0],pos_victim[victim][1],Std_Victim,Std_Victim,Slope_Victim)  #  Victima atractor
    Zv =np.minimum( Z_victim , Zv)
Z = Z + Zv +Z_Global
ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,linewidth=1, antialiased=False,alpha=0.8)
#fig.colorbar(ax, shrink=0.5, aspect=10)
ax.contour(X, Y, Z, zdir='z', offset=-1.5, cmap=cm.coolwarm)

for Robot in range(Number_Robots):
    ax.scatter(pos_robot[Robot][0], pos_robot[Robot][1], 0.5, s=100, c='r', marker="o")
    
ax.set_zlim(-10, 4)
plt.show()

plt.figure(4)
plt.contour(X,Y,Z,15,linewidths=0.25,colors='k')
CS = plt.contourf(X,Y,Z,15,cmap=cm.coolwarm)
plt.show()

plt.figure(2)
plt.title("Repulsion")
Part_Legend = mpatches.Patch(color='orange', label='Particulas')
Enja_Legend = mpatches.Patch(color='blue', label='Enjambres')
plt.legend(handles=[Part_Legend,Enja_Legend])
Espacio = np.arange(-20,20,0.1)
Repulsion_Enjambre_plt = 1/(1+((abs(((Espacio) - 0 )/(Dist_Enjambre)))**(2*Pendiente_Enjambre)))
Repulsion_Particula_plt = 1/(1+((abs(((Espacio) - 0 )/(Dist_Particula)))**(2*Pendiente_Particula)))
plt.plot(Espacio,Repulsion_Enjambre_plt,'b')
plt.plot(Espacio,Repulsion_Particula_plt,'orange')
plt.show()


# ---------------------------- Loop Principal del Programa -------------------------------

    
HistVal = np.empty([Number_Robots, epocas])  # Lista de valores para graficar
HistPosPar = np.empty([Number_Robots, epocas,Part_Enjambre,3 ]) # Lista de valores para graficar
HistPosTarget = np.empty([Number_Robots, epocas, 3]) #Lista de posiciones del Robot

Enjambre_Robot = [[] for i in range(Number_Robots)] # Creacion de Lista de Listas para cada uno de los enjambres

for Robot in range(Number_Robots): #Declaracion de las particulas con posicion[x,y,z] y velocidad [x,y,z]
    for particula_ in range(Part_Enjambre): 
        posicion = np.random.randint(-4,4, size = 2) # Valores entre -5 y 5 de un arreglo de dos dimensiones
        posicion = np.append(posicion,[0]) # Se le agrega al arreglo anterior un tercer elemento que vale 0
        Enjambre_Robot[Robot].append(Particula(posicion + pos_robot[Robot],[0,0,0])) # Creacion de las particulas
        
# Loop Principal    
for i in range(epocas):
    if (np.remainder(i,100)==0):
        print("Epoca: ", i)
    Enjambre_valores = [[] for i in range(Number_Robots)] # Lista de cada uno de los valores de los enjambres
    MejoresPosEnjs = [] # Lista de las mejores posiciones de cada enjambre
    
    for Robot in range(Number_Robots):
        Pos = []
        for particula in Enjambre_Robot[Robot]:
            particula.evaluar(funEval3D) # Evlauacion de la particula                  
            Enjambre_valores[Robot].append(particula.valor) # Se guardan los valores en una lista
            Pos.append(particula.posicion)
            
        Pos = np.array(Pos)
        x = np.sum(Pos[:,0])/Part_Enjambre
        y = np.sum(Pos[:,1])/Part_Enjambre
        
        #MejoresPosEnjs.append(Enjambre_Robot[Robot][Enjambre_valores[Robot].index(min(Enjambre_valores[Robot]))].posicion)
        MejoresPosEnjs.append([x,y,0])
        
        mejorValEnj = min(Enjambre_valores[Robot])
                      
        if mejorValEnj <= min(HistVal[Robot]):
            Vect_DistPar[Robot] = Dist_Particula
            Vect_Guardar[Robot] = Dist_Particula
        else:
            Vect_DistPar[Robot] += 0.01
            Vect_Guardar[Robot] += 0.01
            if Vect_Guardar[Robot] >= 20:
                Vect_Guardar[Robot] = 20
                Vect_DistPar[Robot] = Dist_Particula         
              
        HistVal[Robot][i] = mejorValEnj
        
        
            
        
        for n in range (Part_Enjambre):
            HistPosPar[Robot,i,n,:]=Enjambre_Robot[Robot][n].posicion
            
    for enjambre_ in range(Number_Robots):
        postarget_ = np.zeros(3) # La posicion del target sera la posicion media del enjambre
        posiciones = [] # cada una de las posiciones del enjambre actual
        
        for index, particula_ in enumerate(Enjambre_Robot[enjambre_]):
            
            particula_.repulsionPart = funRepulsionPart(Enjambre_Robot[enjambre_],particula_, Vect_DistPar[enjambre_],Enjambre_valores.index(min(Enjambre_valores)),index) # Respulsion Individual      
            particula_.repulsionSwr = funRepulsionSwr(MejoresPosEnjs,enjambre_,particula_) # Respulsion Enjambre
            posiciones.append(particula_.posicion)
            
            #posplane = [(particula_.posicion[0] + particula_.velocidad[0]*dt),(particula_.posicion[1] + particula_.velocidad[1]*dt),(particula_.posicion[2] + particula_.velocidad[2]*dt)] # Posicion de los planos
            #errorCode = vrep.simxSetObjectPosition(clientID,planes[Enjambre_Robot[enjambre_].index(particula_)+(Part_Enjambre*enjambre_)],-1,[posplane[0],posplane[1],0.2], vrep.simx_opmode_streaming) # Ubicar plano (particula) en Vrep
            
            particula_.actualizarVel(MejoresPosEnjs[enjambre_],C)
            particula_.actualizarPos(dt)
             
        posiciones = np.array(posiciones)
        
        #Ponderado
        """
        Metodo_Target = "Ponderado"
        valores = np.array(Enjambre_valores[enjambre_])        
        valores = 1/(1+((abs(((valores) - min(valores) )/(0.5)))**(2*5)))
        
        postarget_[0] = np.sum(posiciones[:,0]*valores)/np.sum(valores)
        postarget_[1] = np.sum(posiciones[:,1]*valores)/np.sum(valores)  
        
        HistPosTarget[enjambre_,i,:] = postarget_
        
        #postarget_[2] = np.sum(posiciones[:,2]*valores)/np.sum(valores)
        """
        #Promedio
        
        Metodo_Target = "Promedio"
        postarget_[0] = np.sum(posiciones[:,0])/Part_Enjambre
        postarget_[1] = np.sum(posiciones[:,1])/Part_Enjambre
        
        HistPosTarget[enjambre_,i,:] = postarget_
        
        #postarget_[2] = np.sum(posiciones[:,2]*valores)/Part_Enajmbre
        
        
        #Mejor Valor
        """
        Metodo_Target = "Mejor Valor"
        
        
        postarget_[0] = MejoresPosEnjs[enjambre_][0]
        postarget_[1] = MejoresPosEnjs[enjambre_][1]  
        
        HistPosTarget[enjambre_,i,:] = postarget_
        
        #postarget_[2] = MejoresPosEnjs[enjambre_][2]
        """
        
# ------------------------------- Movimiento del Robot ---------------------------------------------------
        
        #errorCode = vrep.simxSetObjectPosition(clientID,target_plane[enjambre_],-1,[postarget_[0],postarget_[1],0.2], vrep.simx_opmode_streaming)
        
        
        
        # Conexion Vrep Robot
        errorCode, PosRobot  = vrep.simxGetObjectPosition(clientID,visible_handle[enjambre_],-1,vrep.simx_opmode_oneshot)
        errorCode, RobAng    = vrep.simxGetObjectOrientation(clientID,visible_handle[enjambre_],-1,vrep.simx_opmode_oneshot)
        LeftVel,RightVel, Distancia = goTo(PosRobot, postarget_, RobAng[2])
            
            
        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle[enjambre_],LeftVel, vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle[enjambre_],RightVel, vrep.simx_opmode_streaming)
        
# ------------------------------------- Detener Motores --------------------------------------------------       

for enjambre_ in range(Number_Robots):    
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle[enjambre_],0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle[enjambre_],0, vrep.simx_opmode_streaming)
          

# ---------------------------------- Grafica de desplazamiento del Enjambre -----------------------


       
        


plt.figure(3)
for Robot in range(Number_Robots):
    plt.plot(HistVal[Robot])  
plt.show()


fig = plt.figure(figsize=(10,10))
ax = fig.add_subplot(1,1,1)
ax.set_title("Swarm navigation and mean robot position")
ax.set_xlabel("X")
ax.set_ylabel("Y")


ax.scatter(HistPosPar[0,1,:,0],HistPosPar[0,1,:,1], s=50, c='g', marker='o',zorder=10, label="Particles Init Position")
ax.plot(HistPosTarget[0,:,0],HistPosTarget[0,:,1],zorder=5, c='k', linewidth=3, label="Mean Position")
ax.scatter(HistPosPar[0,epocas-1,0,0],HistPosPar[0,epocas-1,0,1], s=50, c='g', marker='o', zorder=10, label="Particles Final Position")

ax.contour(X,Y,Z,25,linewidths=0.25,colors='k')
ax.contourf(X,Y,Z,25,cmap=cm.coolwarm,alpha=0.5)


for Robot in range(Number_Robots):
    ax.scatter(HistPosPar[Robot,1,:,0],HistPosPar[Robot,1,:,1], s=50, c='r', marker='o',zorder=10)
    ax.plot(HistPosTarget[Robot,:,0],HistPosTarget[Robot,:,1],zorder=25,c='k', linewidth=1)
    ax.scatter(HistPosPar[Robot,epocas-1,:,0],HistPosPar[Robot,epocas-1,:,1], s=50, c='k', marker='o', zorder=10)
    #ax.plot(HistPosPar[Robot,:,:,0],HistPosPar[Robot,:,:,1])
    
    for particula in range(Part_Enjambre):
        ax.plot(HistPosPar[Robot,:,particula,0],HistPosPar[Robot,:,particula,1],c='#3f99cc')#2737AE
    
    for Pos_Med in range(epocas):
        #print(Pos_Med)
        if Pos_Med % 200 == 0:            
            ax.scatter(HistPosTarget[Robot,Pos_Med,0],HistPosTarget[Robot,Pos_Med,1], s=30, c='g', marker='o', zorder=10)
        
ax.scatter(HistPosTarget[:,epocas-1,0],HistPosTarget[:,epocas-1,1], s=80, c='r', marker='o',zorder=20, label="Mean Final Position")       
ax.legend()       
ax.grid()




fig = plt.figure(figsize=(10,10))
ax = fig.add_subplot(1,1,1)
ax.contour(X,Y,Z,25,linewidths=0.25,colors='k')
ax.contourf(X,Y,Z,25,cmap=cm.coolwarm,alpha=0.5)


ax.scatter(HistPosTarget[:,1,0],HistPosTarget[:,1,1], s=50, c='b', marker='o')
for Robot in range(Number_Robots):
    ax.plot(HistPosTarget[Robot,:,0],HistPosTarget[Robot,:,1])
ax.scatter(HistPosTarget[:,epocas-1,0],HistPosTarget[:,epocas-1,1], s=100, c='r', marker='o')
ax.grid()
            

    

    
################################


fig = plt.figure(figsize=(10,10))
ax = fig.add_subplot(1,1,1)
ax.set_title("Swarm navigation and mean robot position")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_xlim([-20,40])
ax.set_ylim([-50,30])


#ax.scatter(HistPosPar[0,1,:,0],HistPosPar[0,1,:,1], s=50, c='c', marker='o',zorder=10, label="Particles Init Position")
#ax.plot(HistPosTarget[0,:,0],HistPosTarget[0,:,1],zorder=5, c='k', linewidth=3, label="Mean Position")
#ax.scatter(HistPosPar[0,epocas-1,0,0],HistPosPar[0,epocas-1,0,1], s=50, c='g', marker='o', zorder=10, label="Particles Final Position")

ax.contour(X,Y,Z,25,linewidths=0.25,colors='k')
ax.contourf(X,Y,Z,25,cmap=cm.coolwarm,alpha=0.5)


for Robot in range(Number_Robots):
    ax.scatter(HistPosPar[Robot,1,:,0],HistPosPar[Robot,1,:,1], s=50, c='g', marker='o',zorder=100, label="Particles Init Position")
    ax.plot(HistPosTarget[Robot,:epocas-2000,0],HistPosTarget[Robot,:epocas-2000,1],zorder=25,c='k', linewidth=1,label="Robot Path")
    ax.scatter(HistPosPar[Robot,epocas-2000,:,0],HistPosPar[Robot,epocas-2000,:,1], s=50,c='#df6f2f', marker='o', zorder=10, label="Particles Final Position")
    
    for particula in range(Part_Enjambre):
        ax.plot(HistPosPar[Robot,:epocas-2000,particula,0],HistPosPar[Robot,:epocas-2000,particula,1],c='#1166aa')#2737AE
    
    for Pos_Med in range(epocas):
        #print(Pos_Med)
        if Pos_Med % 600 == 0:            
            ax.scatter(HistPosTarget[Robot,Pos_Med,0],HistPosTarget[Robot,Pos_Med,1], s=50, c='c', marker='o', zorder=100)
        
ax.scatter(HistPosTarget[:,epocas-1,0],HistPosTarget[:,epocas-1,1], s=80, c='m', marker='o',zorder=200, label="Mean Final Position")       
ax.legend()       
ax.grid()



import pickle

pickle.dump([X,Y,Z,HistPosPar,HistPosTarget,epocas,Number_Robots,Part_Enjambre], open("Caso2_Tunnel.p", "wb"))


"""
X,Y,Z,HistPosPar,HistPosTarget,epocas,Number_Robots,Part_Enjambre=pickle.load(open("trial.p","rb"))



import pickle

pickle.dump([X,Y,Z,HistPosPar,HistPosTarget,epocas,Number_Robots,Part_Enjambre], open("Caso2.p", "wb"))
    
"""    
    
    




