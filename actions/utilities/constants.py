Number_Robots = 8 #Numero de Robots
Number_Victims = 2 #Numero de Victima        Bill#0 y Bill#1
Number_Obstacles = 12 #Numero de Obstaculos  Fire#0 a Fire#11
Part_Enjambre = 10 # Particulas por Enjambre

C = [0.25,0.25,0.25,0.45,0.65] # Coeficientes PSO
dt = 0.5 # Paso o Step en la actualizacion PSO

Dist_Enjambre = 2.5 # Distancia Minima - Desviacion estandar
Dist_Particula = 2 # Distancia Minima - Desviacion estandar


Pendiente_Enjambre = 5 # Pendiente, Ganancia 
Pendiente_Particula = 10 # Pendiente, Ganancia

Alt_Victim = -2  # Altura de la Gaussiana en la construccion de la Superficie - Victimas
Alt_Obsta = 2    # Altura de la Gaussiana en la construccion de la Superficie - Obstaculos

Std_Victim = 0.1   # Grosor de la Gaussiana en la construccion de la Superficie - Victimas
Std_Obsta = 1     # Grosor de la Gaussiana en la construccion de la Superficie - Obstaculos

Slope_Obsta=40
Slope_Victim=0.01

x_min = y_min = -25 # Dimensiones del area de busqueda
x_max = y_max = 25 
epocas = 1000 # Numero de Iteraciones
mejorValEnj = 0 # Mejor Valor del enjambre


left_motor_name = 'Pioneer_p3dx_leftMotor'
right_motor_name = 'Pioneer_p3dx_rightMotor'
visible_name = 'Pioneer_p3dx_visible'
victim_name = 'Bill'
plane_name = 'Plane'
obstacle_name = 'fire'
planetarget_name = 'Plane_Target'

