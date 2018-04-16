import pypid as pid
import numpy as n


def PD_ARM(t1_f,t2_f,t1_i,t2_i):
    motor_output = 0
    P1 = 0
    D1 = 0
    




    #returns both motor outputs
    return m1_out, m2_out

def PID_EE(P,I,D,tF,tX,eTot,eOld):

    error = tF-tX
    eNew = error - eOld
    #eTot = eTot + error
    eTot=0

    signal = abs((P*error)+(I*eTot)+(D*eNew))
    

    return eTot, eNew, signal, tX