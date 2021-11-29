#import vrep
import numpy as np

from actions.utilities import constants
from actions.classes.particle import Particle
from actions.utilities import repulsion_forces
from actions.utilities import robot_control
from actions.utilities import plots
from actions.utilities import coppelia_conection
from actions.utilities import map_elements
from actions.utilities import evaluation_function


 # ----------------- Conexion con Vrep ---------------

clientID = coppelia_conection.connect()

# ------------------------- Configuración de Elementos de Vrep ---------------------------

#target_plane = []                                                                      # Lista de posiciones del target
#planes = []                                                                            # Lista de planos

# ------------------------ Conexion Dinamica de Elementos Vrep -------------------------------
    
left_motor_handle, right_motor_handle, visible_handle, pos_robot = map_elements.map_robots(clientID)
victim, pos_victim = map_elements.map_victims(clientID)
obstacle, pos_obstacle = map_elements.map_obstacles(clientID)

# --------------------- Plot de la Superficie Resultante ------------------------------------

plots.close_all_figures()
X,Y,Z = plots.generate_surface(pos_obstacle,
                     pos_victim,
                     pos_robot)

plots.generate_contour(X,Y,Z)

# ------------------------- Variables de Interes - Graficar -------------------------------

Vect_DistPar = np.zeros(constants.Number_Robots) + constants.Dist_Particula
Vect_Guardar = np.zeros(constants.Number_Robots) + constants.Dist_Particula   

HistVal = np.empty([constants.Number_Robots,
                    constants.epocas])                                                  # Lista de valores para graficar

HistPosPar = np.empty([constants.Number_Robots,
                       constants.epocas,
                       constants.Part_Enjambre,
                       3 ])                                                             # Lista de valores para graficar

HistPosTarget = np.empty([constants.Number_Robots,
                          constants.epocas,
                          3])                                                           #Lista de posiciones del Robot

Enjambre_Robot = [[] for i in range(constants.Number_Robots)]                           # Creacion de Lista de Listas para cada uno de los enjambres


# ----------------------- Loop Principal del programa ----------------------------------

for Robot in range(constants.Number_Robots):                                            #Declaracion de las particulas con posicion[x,y,z] y velocidad [x,y,z]
    for particula_ in range(constants.Part_Enjambre): 
        posicion = np.random.randint(-5,5, size = 2)                                    # Valores entre -5 y 5 de un arreglo de dos dimensiones
        posicion = np.append(posicion,[0])                                              # Se le agrega al arreglo anterior un tercer elemento que vale 0
        Enjambre_Robot[Robot].append(Particle(posicion + pos_robot[Robot],[0,0,0]))     # Creacion de las particulas
        
# Loop Principal    
for i in range(constants.epocas):
    print("Epoca: ", i)
    Enjambre_valores = [[] for i in range(constants.Number_Robots)]                     # Lista de cada uno de los valores de los enjambres
    MejoresPosEnjs = []                                                                 # Lista de las mejores posiciones de cada enjambre
    
    for Robot in range(constants.Number_Robots):
        Pos = []
        for particula in Enjambre_Robot[Robot]:            
            particula.evaluar(evaluation_function.funEval3D(particula.posicion,
                                                            pos_obstacle,
                                                            pos_victim))                # Evlauacion de la particula          
            
            Enjambre_valores[Robot].append(particula.valor)                             # Se guardan los valores en una lista
            Pos.append(particula.posicion)
            
        Pos = np.array(Pos)
        x = np.sum(Pos[:,0])/constants.Part_Enjambre
        y = np.sum(Pos[:,1])/constants.Part_Enjambre
        
        #MejoresPosEnjs.append(Enjambre_Robot[Robot][Enjambre_valores[Robot].index(min(Enjambre_valores[Robot]))].posicion)
        MejoresPosEnjs.append([x,y,0])
        
        mejorValEnj = min(Enjambre_valores[Robot])
                      
        if mejorValEnj <= min(HistVal[Robot]):
            Vect_DistPar[Robot] = constants.Dist_Particula
            Vect_Guardar[Robot] = constants.Dist_Particula
        else:
            Vect_DistPar[Robot] += 0.01
            Vect_Guardar[Robot] += 0.01
            if Vect_Guardar[Robot] >= 20:
                Vect_Guardar[Robot] = 20
                Vect_DistPar[Robot] = constants.Dist_Particula         
              
        HistVal[Robot][i] = mejorValEnj       
        for n in range (constants.Part_Enjambre):
            HistPosPar[Robot,i,n,:]=Enjambre_Robot[Robot][n].posicion
            
    for enjambre_ in range(constants.Number_Robots):
        postarget_ = np.zeros(3) # La posicion del target sera la posicion media del enjambre
        posiciones = [] # cada una de las posiciones del enjambre actual
        
        for index, particula_ in enumerate(Enjambre_Robot[enjambre_]):            
            particula_.repulsionPart = repulsion_forces.funRepulsionPart(Enjambre_Robot[enjambre_],
                                                                         particula_,
                                                                         Vect_DistPar[enjambre_],
                                                                         Enjambre_valores.index(min(Enjambre_valores)),
                                                                         index) # Respulsion Individual            
            particula_.repulsionSwr = repulsion_forces.funRepulsionSwr(MejoresPosEnjs,
                                                                       enjambre_,
                                                                       particula_) # Respulsion Enjambre
            posiciones.append(particula_.posicion)            
            #posplane = [(particula_.posicion[0] + particula_.velocidad[0]*dt),(particula_.posicion[1] + particula_.velocidad[1]*dt),(particula_.posicion[2] + particula_.velocidad[2]*dt)] # Posicion de los planos
            #errorCode = vrep.simxSetObjectPosition(clientID,planes[Enjambre_Robot[enjambre_].index(particula_)+(Part_Enjambre*enjambre_)],-1,[posplane[0],posplane[1],0.2], vrep.simx_opmode_streaming) # Ubicar plano (particula) en Vrep            
            particula_.actualizarVel(MejoresPosEnjs[enjambre_],
                                     constants.C)            
            particula_.actualizarPos(constants.dt)
            
             
        posiciones = np.array(posiciones)
        # Target Promedio         
        postarget_[0] = np.sum(posiciones[:,0])/constants.Part_Enjambre
        postarget_[1] = np.sum(posiciones[:,1])/constants.Part_Enjambre
        
        HistPosTarget[enjambre_,i,:] = postarget_
        
# ------------------------------- Movimiento del Robot ---------------------------------------------------
        
        #errorCode = vrep.simxSetObjectPosition(clientID,target_plane[enjambre_],-1,[postarget_[0],postarget_[1],0.2], vrep.simx_opmode_streaming)        
        # Conexion Vrep Robot        
        PosRobot = robot_control.getPosicion(clientID,
                                             visible_handle,
                                             enjambre_)
        AngRobot = robot_control.getOrientation(clientID,
                                                visible_handle,
                                                enjambre_)
        
        
        LeftVel, RightVel, Distancia = robot_control.goTo(PosRobot,
                                                          postarget_,
                                                          AngRobot[2])           
            
        robot_control.targetVelocity(clientID,
                                     LeftVel,
                                     RightVel,
                                     left_motor_handle,
                                     right_motor_handle,
                                     enjambre_)
        
# ------------------------------------- Detener Motores --------------------------------------------------               

robot_control.stop(clientID,
                   left_motor_handle,
                   right_motor_handle)
plots.results(HistVal,
              HistPosPar,
              HistPosTarget,
              X,
              Y,
              Z)
       
        



            

    

    
    
    
    
    




