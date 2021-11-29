import numpy as np
from actions.utilities import constants
from actions.utilities import map_elements


def makeGaussian(X,
                 Y,
                 Amp,
                 CenterX,
                 CenterY,
                 DX,
                 DY,
                 Slope):
    
    return Amp*np.exp(-Slope*np.log(2) * (((X-CenterX)**2)/ DX**2 + ((Y-CenterY)**2)/ DY**2) )




def funEval3D(pos_robot, pos_obstacle, pos_victim): 
    
    """
    victim = [] # lista de victimas
    pos_victim = [] # Lista de posiciones de la victima
    obstacle = [] # Lista de Obstaculos
    pos_obstacle = [] # Lista de posiciones de los Obstaculos
    """
    z_ = 0
    for obstacle in range(constants.Number_Obstacles):
        #Leer de Vrep la Posicion de los Obstaculos
        """
        _obstacle_name = obstacle_name + '#' + str(Obstacle)
        errorCode, Fire_handle =vrep.simxGetObjectHandle(clientID,_obstacle_name,vrep.simx_opmode_oneshot_wait)
        errorCode, PosFire = vrep.simxGetObjectPosition(clientID,Fire_handle,-1,vrep.simx_opmode_oneshot)    
        obstacle.append(Fire_handle)
        pos_obstacle.append(PosFire)
        """
        
        
        R_Obstacle = makeGaussian(pos_robot[0],
                                  pos_robot[1],
                                  constants.Alt_Obsta,
                                  pos_obstacle[obstacle][0],
                                  pos_obstacle[obstacle][1],
                                  constants.Std_Obsta,
                                  constants.Std_Obsta,
                                  constants.Slope_Obsta)
        z_ = R_Obstacle + z_
        
    for victim in range(constants.Number_Victims):
        #Leer de Vrep la Posicion de las Victimas
        """
        _victim_name = victim_name + '#' + str(Victim)
        errorCode, Victim_handle =vrep.simxGetObjectHandle(clientID,_victim_name,vrep.simx_opmode_oneshot_wait)
        errorCode, PosVictim = vrep.simxGetObjectPosition(clientID,Victim_handle,-1,vrep.simx_opmode_oneshot)    
        victim.append(Victim_handle)
        pos_victim.append(PosVictim)
        """   
        
        R_Victim = makeGaussian(pos_robot[0],
                                pos_robot[1],
                                constants.Alt_Victim,
                                pos_victim[victim][0],
                                pos_victim[victim][1],
                                constants.Std_Victim,
                                constants.Std_Victim,
                                constants.Slope_Victim) #  Victima atractor
        
        z_ = R_Victim + z_
        
    return z_
