import math
import numpy as np
from actions.utilities import constants

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
            distancia_minima_ = constants.Dist_Enjambre
            pendiente_ = constants.Pendiente_Enjambre
            
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

    if distancia_minima != constants.Dist_Particula and indexBestPart == indexActPart:
        return [0,0,0]
    else:
        for part in particulas:
            if part.posicion[0] == particula.posicion[0] and part.posicion[1] == particula.posicion[1] and part.posicion[2] == particula.posicion[2]:
                x_[particulas.index(part)] = 0
                y_[particulas.index(part)] = 0
                z_[particulas.index(part)] = 0
                
    
                
    
            else:
                #distancia_minima = Dist_Particula
                pendiente = constants.Pendiente_Particula
                
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