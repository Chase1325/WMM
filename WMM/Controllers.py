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
    eTot=eTot+eNew#Ignore I term for now

    signal = abs((P*eNew)+(I*eTot)+(D*eDelta))

    if(signal>1000):
        signal=1000

        #t2_f<tX_new=0, else 1
    if(dir==0 and tX<0):
            pwm = n.interp(signal,[0,1000],[0,100])
    if(dir==1 and tX>=0):
            pwm = n.interp(signal,[0,1000],[0,-100])
    if(dir==1 and tX<0):
            pwm = n.interp(signal,[0,1000],[0,-100])
    if(dir==0 and tX>=0):
            pwm = n.interp(signal,[0,1000],[0,100])

    
    return eTot, eNew, pwm, tX

def PID_EE2(P,I,D,tF,tX,eTot,eOld,dir):

    eNew = abs(tF-tX)
    eDelta = eNew - eOld
    eTot=eTot+eNew#Ignore I term for now

    signal = abs((P*eNew)+(I*eTot)+(D*eDelta))

    if(signal>1000):
        signal=1000

        #t2_f<tX_new=0, else 1
    if(dir==0 and tX<0):
            pwm = n.interp(signal,[0,1000],[15,30])
    if(dir==1 and tX>=0):
            pwm = n.interp(signal,[0,1000],[-15,-30])
    if(dir==1 and tX<0):
            pwm = n.interp(signal,[0,1000],[-15,-30])
    if(dir==0 and tX>=0):
            pwm = n.interp(signal,[0,1000],[15,30])

    
    return eTot, eNew, pwm, tX

