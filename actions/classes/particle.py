import numpy as np
import math

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
    

class Particle(object):
    def __init__(self,x,v):
        
        
        
        self.posicion = x
        self.velocidad = v
        self.mejorValor = np.inf
        self.mejorPosicion = x
        self.repulsionPart = np.zeros(3)
        self.repulsionSwr = np.zeros(3)
        self.valor = 0
        
        
    def evaluar(self, valor):        
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
             self.velocidad[i]= C[0]*self.velocidad[i]  +  C[1]*np.random.random()*(Norm_PosPart[i])  + C[2]*np.random.random()*(Norm_PosSwrm[i])  + C[3]*np.random.random()*(self.repulsionPart[i]) + C[4]*np.random.random()*(self.repulsionSwr[i])
             
    def actualizarPos(self,dt):
        self.posicion = self.posicion + np.array(self.velocidad)*dt

