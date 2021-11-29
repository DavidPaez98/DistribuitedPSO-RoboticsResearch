import numpy as np
from actions.utilities import constants
from actions.utilities import vrep 


def goTo(robot,
         target,
         RobAng):
    
    Kr=6
    Kp=3
    i=0.5  # Integrador Falso
    MaxVel=10
    dist= ((target[0]-robot[0])**2 + (target[1]-robot[1])**2)**0.5
    TarRobAng =np.arctan2((target[1]-robot[1]), (target[0]-robot[0]))
   
    if abs(TarRobAng-RobAng)>np.pi:
        Rotacion=(TarRobAng-RobAng)-2*np.pi*np.sign(TarRobAng-RobAng)
    else:
        Rotacion=(TarRobAng-RobAng)
        
    LeftWheel=-Kr*(Rotacion/np.pi)+Kp*dist +i
    RightWheel=Kr*(Rotacion/np.pi)+Kp*dist +i
    
    if LeftWheel > MaxVel:
        RightWheel=  (RightWheel/LeftWheel)*MaxVel
        LeftWheel=MaxVel
    if RightWheel > MaxVel:
        LeftWheel=  (LeftWheel/RightWheel)*MaxVel
        RightWheel=MaxVel
    return LeftWheel, RightWheel, dist

def stop(clientID, left_motor_handle, right_motor_handle):    
    
    for enjambre_ in range(constants.Number_Robots):    
        errorCode = vrep.simxSetJointTargetVelocity(clientID,
                                                left_motor_handle[enjambre_],
                                                0,
                                                vrep.simx_opmode_blocking)
    
        errorCode = vrep.simxSetJointTargetVelocity(clientID,
                                                right_motor_handle[enjambre_],
                                                0,
                                                vrep.simx_opmode_blocking)
    

def getPosicion(clientID, visible_handle, enjambre_):
    errorCode, PosRobot  = vrep.simxGetObjectPosition(clientID,
                                                      visible_handle[enjambre_],
                                                      -1,
                                                      vrep.simx_opmode_oneshot)
    return PosRobot

def getOrientation(clientID, visible_handle, enjambre_):
    errorCode, AngRobot = vrep.simxGetObjectOrientation(clientID,
                                                      visible_handle[enjambre_],
                                                      -1,
                                                      vrep.simx_opmode_oneshot)
    
    return AngRobot

def targetVelocity(clientID, LeftVel, RightVel, left_motor_handle, right_motor_handle, enjambre_):
    errorCode = vrep.simxSetJointTargetVelocity(clientID,
                                                    left_motor_handle[enjambre_],
                                                    LeftVel,
                                                    vrep.simx_opmode_streaming)
        
    errorCode = vrep.simxSetJointTargetVelocity(clientID,
                                                right_motor_handle[enjambre_],
                                                RightVel,
                                                vrep.simx_opmode_streaming) 
    