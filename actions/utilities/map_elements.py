
from actions.utilities import constants
from actions.utilities import vrep



target_plane = [] # Lista de posiciones del target
planes = [] # Lista de planos



def map_robots(clientID):    
    
    
    left_motor_handle = [] # Lista de Velocidades del motor izquierdo
    right_motor_handle = [] # Lista de Velocidades del motor derecho
    visible_handle = [] # Lista de Pioneers
    pos_robot = [] # Lista de posiciones del robot
    
    
    for Robot in range(constants.Number_Robots):
        _left_motor = constants.left_motor_name + '#' + str(Robot)
        _right_motor = constants.right_motor_name + '#' + str(Robot)
        _visible = constants.visible_name + '#' + str(Robot)
        _planetarget = constants.planetarget_name + '#' + str(Robot)     
        
        errorCode, left_motor_handle_ = vrep.simxGetObjectHandle(clientID,
                                                                 _left_motor,
                                                                 vrep.simx_opmode_oneshot_wait)
        
        errorCode, right_motor_handle_ = vrep.simxGetObjectHandle(clientID,
                                                                  _right_motor,
                                                                  vrep.simx_opmode_oneshot_wait)
        
        errorCode, pioneer_handle_ = vrep.simxGetObjectHandle(clientID,
                                                              _visible
                                                              ,vrep.simx_opmode_oneshot_wait)
        
        errorCode, target_handle = vrep.simxGetObjectHandle(clientID,
                                                            _planetarget,
                                                            vrep.simx_opmode_oneshot_wait)
        
        errorCode, PosRobot  = vrep.simxGetObjectPosition(clientID,
                                                          pioneer_handle_,
                                                          -1,
                                                          vrep.simx_opmode_blocking) 
        
        
        
        
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
    return left_motor_handle, right_motor_handle, visible_handle, pos_robot

def map_victims(clientID):
    
    victim = [] # lista de victimas
    pos_victim = [] # Lista de posiciones de la victima
    
    for Victim in range(constants.Number_Victims):
        _victim_name = constants.victim_name + '#' + str(Victim)
        errorCode, Victim_handle = vrep.simxGetObjectHandle(clientID,
                                                            _victim_name,
                                                            vrep.simx_opmode_oneshot_wait)
        
        errorCode, PosVictim = vrep.simxGetObjectPosition(clientID,
                                                          Victim_handle,
                                                          -1,
                                                          vrep.simx_opmode_blocking)
        
        
        victim.append(Victim_handle)
        pos_victim.append(PosVictim)
        
    return victim, pos_victim

def map_obstacles(clientID):
    
    obstacle = [] # Lista de Obstaculos
    pos_obstacle = [] # Lista de posiciones de los Obstaculos
    
    for Obstacle in range(constants.Number_Obstacles):
        _obstacle_name = constants.obstacle_name + '#' + str(Obstacle)
        errorCode, Fire_handle = vrep.simxGetObjectHandle(clientID,
                                                          _obstacle_name,
                                                          vrep.simx_opmode_oneshot_wait)
        
        errorCode, PosFire = vrep.simxGetObjectPosition(clientID,
                                                        Fire_handle,
                                                        -1,
                                                        vrep.simx_opmode_blocking)
        
        
        obstacle.append(Fire_handle)
        pos_obstacle.append(PosFire)
        
    return obstacle, pos_obstacle

def map_particles(clientID):
    pass
