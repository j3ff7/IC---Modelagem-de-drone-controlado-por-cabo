import numpy as np
from scipy.optimize import fsolve
import math as m

def calcular_tensao(vao, comprimento, diametro, densidade):
    g = 9.81
    raio = diametro / 2
    area_secao = m.pi *(raio**2)
    
    massa_l = area_secao * densidade
    peso_l = massa_l * g
    
    def erro_comprimento(a):
        return 2 * a * np.sinh(vao/(2*a)) - comprimento
    
    a_solucao = fsolve(erro_comprimento, 1)
    a = a_solucao[0]
    
    y_max = a * np.cosh(vao / (2*a))
    forca_max = peso_l * y_max
    
    tensao = (forca_max/area_secao) / 1e6 #Saí em MPa
    
    return forca_max, tensao

vao = 1
comprimento = 2
diametro = 0.015
densidade = 1000

forca_r, tensao_r = calcular_tensao(vao,comprimento,diametro,densidade)

if forca_r is not None:
    print("Resultados dos Cálculos:")
    print(f"Força Máxima na ponta: {forca_r:.2f} Newtons")
    print(f"Tensão:    {tensao_r:.2f} MPa")
    