import numpy as np
from actions.utilities import constants

#Ponderado

def getWeightedTarget(posiciones, Enjambre_valores, enjambre_):       
        
    valores = np.array(Enjambre_valores[enjambre_])        
    valores = 1/(1+((abs(((valores) - min(valores) )/(0.5)))**(2*5)))
    postarget_ = np.zeros(3)
    postarget_[0] = np.sum(posiciones[:,0]*valores)/np.sum(valores)
    postarget_[1] = np.sum(posiciones[:,1]*valores)/np.sum(valores)  

    return postarget_
        
def getAverageTarget(posiciones:list):
    posiciones = np.array(posiciones)
     
    
    #postargetx = np.sum(posiciones[:,0])/constants.Part_Enjambre
    #postargety = np.sum(posiciones[:,1])/constants.Part_Enjambre
    postargetx = 0
    postargety = 0
    postargetz = 0
    
    return [0,0,0]
        
def getBestTarget(MejoresPosEnjs, enjambre_):       
        
    postarget_ = np.zeros(3)
    postarget_[0] = MejoresPosEnjs[enjambre_][0]
    postarget_[1] = MejoresPosEnjs[enjambre_][1]  

    return postarget_
