#import pypid as pid
import numpy as n
import math as m


def PD_ARM(t1_f,t2_f,t1_i,t2_i):
    motor_output = 0
    P1 = 0
    D1 = 0
    
    #returns both motor outputs
    return m1_out, m2_out

def PID_EE(P,I,D,tF,tX,eTot,eOld,dir):

    eNew = abs(tF-tX)
    eDelta = eNew - eOld
    eTot=0#Ignore I term for now

    signal = abs((P*eNew)+(I*eTot)+(D*eDelta))

    if(signal>1000):
        signal=1000

    if(dir==0 and tX<0):
            pwm = n.interp(signal,[0,1000],[0,100])
    if(dir==0 and tX>=0):
            pwm = n.interp(signal,[0,1000],[0,-100])
    if(dir==1 and tX<0):
            pwm = n.interp(signal,[0,1000],[0,-100])
    if(dir==1 and tX>=0):
            pwm = n.interp(signal,[0,1000],[0,100])

    
    return eTot, eNew, pwm, tX


