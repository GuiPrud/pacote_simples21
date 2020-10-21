#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 11 17:39:38 2020

@author: yohas
"""
#Importando as bibliotécas e as mensagens que serão usadas
import rospy
import time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math
#
    
gapArray = [0] #definindo a variavel global que será compartilhada, assim
bestGapTu = ()
erro_anterior = 0
mux_switch_count = 0
estado = 0

def corrigeGap(lista, valorMin, x = 3):
    """
    Entradas são a lista de valores lidos pelo lidar o valorMinimo que é um float, e x que é um inteiro.
    Retorna a lista transformando todos os valores menores do que o valor minimo, e x de seus adjacentes em 0
    Ordem de grandeza máximo (2x+1)n+n -> O(n)
    Funciionamento: Varre a lista guardando o index dos valoress menores ou iguais ao valorMin em uma lista. Depois,
    usando essa lista de indexamentos, a função modifica o valor indexado e x valores, para mais e menos, para zero.
    """
    newlista= []
    for c in range(len(lista)):
        if lista[c] <= valorMin:
            newlista.append(c)
    for n in newlista:
        for i in range(2*x+1):
            a=(n + x - i)
            if   a >= 0 and a < len(lista):
                lista[a] = 0

    #print str(lista) #print para testes
    return lista

def callback(msg):
    '''
    Função que é ativada a cada leitura do scan e atualiza a "gap array"
    aumentando os obstáculos proximos para auxiliar a escolha da melhor direção.
    '''
    global estado
    global gapArray
    #define-se o arco de visão onde estamos procurando os obstáculos, e a menor distância entre o carro e os obstáculos.
    rangeMax=int(270*1080/360)
    rangeMin=int(90*1080/360)
    
    tempList=list(msg.ranges[rangeMin:rangeMax])


    #atualiza o gapArray com a leitura atual
    gapArray=tempList[:]

def melhorGap(listaGaps):
    """
    Entrada é uma lista de valores de lacunas e zeros.
    Retornar um tuple com o primeiro valor sendo o index central do melhor lacuna de não zeros dessa lista,
    o segundo valor sendo o numero de valores tem essa lacuna, o terceiro o valor da média desses numeros, e o
    quarto o numero de valores que tem a lista.
    Ordem de grandeza -> O(n²)
    """

   # print str(listaGaps)
    melhorTupla=(0, 0, 0, 0)
    tamLista=len(listaGaps)
    for v in range(len(listaGaps)):
        listaNova=[]
        med = 0
        centro=0
        tamLac=0
        if listaGaps[v] > 0:
            for i in range(len(listaGaps)-v):
                if listaGaps[v+i]>0:
                    listaNova.append(listaGaps[v+i])
                else:
                    break
            tamLac=len(listaNova)
            med=sum(listaNova)/len(listaNova)
            centro=v+int(len(listaNova)/2)
            if tamLac>melhorTupla[1] or (tamLac==melhorTupla[1] and med>melhorTupla[2]):
                melhorTupla=(centro, tamLac, med, tamLista)

    #print str(melhorTupla)     #usado para teste
    return melhorTupla


class FollowTheGap(object):

    direc_msg = AckermannDriveStamped()

    def __init__(self):

        rospy.init_node('lego_team_node', anonymous = False)

        #self.sub = rospy.Subscriber('/scan', LaserScan, callback)
	self.sub = rospy.Subscriber('/lego_team_id/scan', LaserScan, callback)

        
        rate = rospy.Rate(40)

        while not rospy.is_shutdown():
            global gapArray
            global estado
            
	    #rospy.loginfo(str(estado))
            
            #print gapArray
            self.behaviourControll(gapArray)
            if estado == 1: 
                direc_msg = self.GapFollow(gapArray)
            else:
                direc_msg = self.WallFollow(gapArray)
           
            gap_pub.publish(direc_msg)
            rate.sleep()
            
            
    def GapFollow(self, listaScan):
        
        minDist = 4.0
        #função para retornar a lista de leituras com as lacunas sendo valores diferentes de zero
        GPArray=corrigeGap(listaScan, minDist, 8)
        #rospy.loginfo(str(dist)) #usado para teste
        direc_msg = AckermannDriveStamped()
        direc_msg.drive.steering_angle = 0


        direc_msg.drive.speed = 0.0
        
        global gap_pub

        gapTuple=melhorGap(GPArray)
        frente=[0]
        pontoAdiante=0
        
        if len(listaScan) > 1:
            frente=listaScan[int(gapTuple[3]/2)-4:int(gapTuple[3]/2)+5]
            pontoAdiante=listaScan[90*3]

        if not gapTuple == (0,0,0,0):
            if ((gapTuple[0]-int(gapTuple[3]/2)) > 2*int(gapTuple[3]/3)) or ((gapTuple[0]-int(gapTuple[3]/2)) < (0 - 2*int(gapTuple[3]/3))):  
                direc_msg.drive.steering_angle = ((gapTuple[0]-int(gapTuple[3]/2))*0.4/(gapTuple[3]/2))
            elif ((gapTuple[0]-int(gapTuple[3]/2)) < int(gapTuple[3]/3)) and ((gapTuple[0]-int(gapTuple[3]/2)) > (0 - int(gapTuple[3]/3))):
                direc_msg.drive.steering_angle = ((gapTuple[0]-int(gapTuple[3]/2))*0.5/(gapTuple[3]/2))
            else:
                direc_msg.drive.steering_angle = ((gapTuple[0]-int(gapTuple[3]/2))*0.4/(gapTuple[3]/2))
	if(pontoAdiante < 5):
        	direc_msg.drive.speed = 1*pontoAdiante
	elif(pontoAdiante < 10):
        	direc_msg.drive.speed = 0.5*pontoAdiante  
	else:
        	direc_msg.drive.speed = 0.35*pontoAdiante  

        if pontoAdiante < 0.2:
            direc_msg.drive.speed = 0.5
        return direc_msg

    def behaviourControll(self, listaScan):
        
        dist_min_existe = False
        indice_min=0
        indice_max=0
        if len(listaScan) > 1:
            indice_min = 80*3
            indice_max = 100*3
        
        global estado
        global gapArray
        global mux_switch_count
      
        distancia = min(listaScan[indice_min:indice_max+1])
    
        if (distancia < 4):
            dist_min_existe = True


        if (dist_min_existe == True and estado == 0): 
            estado = 1
    
        if estado == 1:
            mux_switch_count += 1
            if dist_min_existe == True:
                mux_switch_count = 0
            if (mux_switch_count > 20 and dist_min_existe == False):
                estado = 0
                mux_switch_count = 0
                
    def WallFollow(self, listaScan):

        wall_avoid_msg = AckermannDriveStamped()
	
        if len(listaScan) > 1:        	

		distancia_direita = listaScan[0]
		distancia_esquerda = listaScan[len(listaScan)-1]

		theta = 45
		set_point = (distancia_direita + distancia_esquerda)/2

		a = listaScan[3*145]
		b = listaScan[len(listaScan)-1]
		if (a > 0.01 and b > 0.01):
			alpha = math.atan((a*math.cos(theta)-b)/(a*math.sin(theta)))
			AB = b*math.cos(alpha)

			# AC eh a distancia que o carro eh projetado
			AC = 0.5
			CD = AB + AC*math.sin(alpha)

			Kp = 0.4
			Kd = 0.2

			#erro_atual = set_point - (CD+CD2)/2
			erro_atual = set_point - CD
			global erro_anterior

			acao_de_controle = Kp*erro_atual+Kd*(erro_atual - erro_anterior)



			if(-30*abs(erro_atual) + 7 > 3):
			    if(listaScan[90*3] > 9):
				wall_avoid_msg.drive.speed = -30*abs(erro_atual) + 9
			    else:
				if(-30*abs(erro_atual) + 9/listaScan[90*3] > 3):
				    wall_avoid_msg.drive.speed = -30*abs(erro_atual) + 9/listaScan[90*3]
				else:
				    wall_avoid_msg.drive.speed = 3
			else:
			    wall_avoid_msg.drive.speed = 3
			    
			wall_avoid_msg.drive.steering_angle = -acao_de_controle

			erro_anterior = erro_atual
	else:
		wall_avoid_msg.drive.speed = 2
		wall_avoid_msg.drive.steering_angle = 0

        return wall_avoid_msg

def main():

    fg = FollowTheGap()
    rospy.spin()

# O drive_pub foi declarado global para que ele possa ser usado tando no construtor quanto na funcao. callback
#gap_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
gap_pub = rospy.Publisher('/lego_team_id/drive', AckermannDriveStamped, queue_size=10)

main()
