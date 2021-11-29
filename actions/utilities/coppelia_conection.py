from actions.utilities import vrep
import time

def connect():
    vrep.simxFinish(-1)
    
    clientID = vrep.simxStart('127.0.0.1',
                              19999,
                              True,
                              True,
                              5000,
                              5) # Connect to V-REP
    
    if clientID!=-1:
        print ('Connected to remote API server')
        return clientID
    else:
        print("Conexion no exitosa")
        return 0
    
    time.sleep(1)
    
    