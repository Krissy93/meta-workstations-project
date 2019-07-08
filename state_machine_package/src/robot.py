#!/usr/bin/env python

import math
import numpy as np

def workspace():
    # tolgo 5 gradi a tutte le posizioni date nel datasheet
    J1_min = math.radians(-162)
    J1_max = math.radians(162)
    alfa1_min = J1_min
    alfa1_max = J1_max

    J2_min = math.radians(-125)
    J2_max = math.radians(85)
    alfa2_min = J2_min + math.pi/2
    alfa2_max = J2_max + math.pi/2

    J3_min = math.radians(-66)
    J3_max = math.radians(96)
    alfa3_min = J3_min - math.pi/2
    alfa3_max = J3_max - math.pi/2

    J4_min = math.radians(-175)
    J4_max = math.radians(175)

    J5_min = math.radians(-108)
    J5_max = math.radians(108)
    alfa5_min = J5_min
    alfa5_max = J5_max

    J6_min = math.radians(-355)
    J6_max = math.radians(355)

    Jmin = np.array([J1_min, J2_min, J3_min, J4_min, J5_min, J6_min])
    Jmax = np.array([J1_max, J2_max, J3_max, J4_max, J5_max, J6_max])
    Qmin = np.array([alfa1_min, alfa2_min, alfa3_min, alfa5_min])
    Qmax = np.array([alfa1_max, alfa2_max, alfa3_max, alfa5_max])
    return Jmin, Jmax, Qmin, Qmax

def robot():

    h1 = 321.5
    l1 = 50
    l2 = 270
    l3 = 307.0847
    l4 = 78.5
    theta = 0.23

    robot = np.array([h1, l1, l2, l3, l4, theta])
    return robot

def singularity(robot, xp, yp, zp, psi):
    ok = True
    o1 = math.fabs(math.sqrt(math.pow(xp,2) + math.pow(yp,2)) - robot[4]*math.cos(psi))
    o2 = math.fabs(zp - robot[4]*math.sin(psi))

    dmax = math.sqrt(math.pow(robot[1] - o1,2) + math.pow(robot[0] - o2,2))

    # cilindro interno singolare
    if (math.pow(xp,2) + math.pow(yp,2)) <= 50:
        print('ERROR: punto singolare vicino all''asse singolare (x,y) = (0,0)!')
        ok = False
    elif dmax > (robot[2] + robot[3]):
        psi_min = math.atan2(zp - robot[0], math.sqrt(math.pow(xp,2) + math.pow(yp,2) - robot[1]))
        o1_min = math.fabs(math.sqrt(math.pow(xp,2) + math.pow(yp,2)) - robot[4]*math.cos(psi_min))
        o2_min = math.fabs(zp - robot[4] * math.sin(psi_min))
        dmax_min = math.sqrt(math.pow(robot[1] - o1_min,2) + math.pow(robot[0] - o2_min,2))
        if dmax_min > (robot[2] + robot[3]):
            print('ERROR: punto esterno alla zona raggiungibile!')
            ok = False
        else:
            print('ATTENZIONE: punto raggiungibile solo con psi = ' + str(psi_min))
            psi = psi_min

    return psi, ok

def inverseKinematics(robot, Qmin, Qmax, S, motor):
    '''  -S = valori posizione nello spazio di lavoro
    S = [ xp[mm] , yp[mm] , zp[mm] , psi[grad] ]
    psi = angolo della pinza rispetto al piano in gradi

    -Q = valori di posizione nello spazio dei giunti [alfa1,alfa2,alfa3,alfa4]
    (il giunto J5 corrisponde ad alfa4)

    -motor = consente di decidere quale configurazione del motore alfa2 e alfa3 adottare
           [1=config. "alto"; 0=config. "basso"] '''

    Q = np.array([0, 0, 0, 0, 0, 0])
    xp = S[0]
    yp = S[1]
    zp = S[2]
    psi = math.radians(S[3]) # in radianti
    psi, ok = singularity(robot, xp, yp, zp, psi)
    PSI = psi - 2 * robot[5] #theta e' parte della definizione geometrica del robot

    if ok == True:
        # giunto 1
        alfa1 = math.atan2(yp, xp)
        if (alfa1 <= Qmin[0]) or (alfa1 >= Qmax[0]):
            print('ERROR: giunto J1-ALFA1 fuori range massimo.')
        else:
            # continuo con il giunto 3
            xa = xp - (robot[4] * math.cos(PSI) + robot[1]) * math.cos(alfa1)
            ya = yp - (robot[4] * math.cos(PSI) + robot[1]) * math.sin(alfa1)
            za = zp - (robot[4] * math.sin(PSI)) - robot[0]
            Diag = math.sqrt(xa * xa + ya * ya)
            # controllo se sono nello stesso quadrante
            if (xp * xa <= 0) and (yp * ya <= 0):
                Diag = - Diag

            alfa3 = math.acos((math.pow(Diag,2) + math.pow(za,2) - math.pow(robot[2],2) - math.pow(robot[3],2)) / (2 * robot[2] * robot[3]))
            if motor == 1: #configurazione alta, giro il segno. altrimenti e' in configurazione bassa di default
                alfa3 = -alfa3

            alfa3 = alfa3 + robot[5] # + theta

            if (alfa3 <= Qmin[2]) or (alfa3 >= Qmax[2]):
                print('ERROR: giunto J3-ALFA3 fuori range massimo.')
            else:
                # procedo con il giunto 2
                alfa2 = math.atan2(za, Diag) - math.atan2(robot[3] * math.sin(alfa3), robot[2] + (robot[3] * math.cos(alfa3)))
                if (alfa2 <= Qmin[1]) or (alfa2 >= Qmax[1]):
                    print('ERROR: giunto J2-ALFA2 fuori range massimo.')
                else:
                    # procedo con il giunto 5
                    alfa5 = psi - alfa2 - alfa3
                    if (alfa5 <= Qmin[3]) or (alfa5 >= Qmax[3]):
                        print('ERROR: giunto J5-ALFA5 fuori range massimo.')
                    else:
                        Q = np.array([alfa1, alfa2 - math.pi/2, alfa3 + math.pi/2, 0, alfa5, 0])

    return Q

def elaborateKin(S):
    Jmin, Jmax, Qmin, Qmax = workspace()
    R = robot()
    Q = inverseKinematics(R, Qmin, Qmax, S, 1)
    #print(Q)
    return Q

def elaborateKinMulti(Points):
    Jmin, Jmax, Qmin, Qmax = workspace()
    R = robot()
    Q = []
    for i in range(0,len(Points)):
        S = np.array(Points[i])
        q = inverseKinematics(R, Qmin, Qmax, S, 1)
        Q.append(q)
    #print(Q)
    return Q
