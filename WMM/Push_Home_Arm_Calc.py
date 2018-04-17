import math as m
#import sympy as S
import numpy as n
import Kinematics as Kin
import Controllers as C
import time
import rospy
from std_msgs.msg import Float32


dh_param = [[1, 5, 5, 1,], # Link 0
            [3, 4, 3, 2,], # Link 1
            [9, 1, 3, 1]]  # Link 2



def calcIK():
    t1=0
    t2=0
    Hx = 0
    Hy = 0

    #Calculate Joint 1 and 2 angles using:
        #EEx, EEy, Hx, Hy
        #

    ForwardPos = Kin.Arm_F_Pos_Kin(dh_param,Hx,Hy)

    InvPos = Kin.Arm_Inv_Pos_Kin(ForwardPos,Hx,Hy)
    t1 = InvPos[0]
    t2 = InvPos[1]

    J = Kin.Arm_Jacobian(dh_param,t1,t2)
    print(J)
    #Kin.Arm_Inv_Vel_Kin(J,5,5)
    print('Calculated IK')
    return t1,t2


def home2Handle_Control(t1_f,t2_f):
    print('Drive Arm to needed position')

    t1_i = -90 #deg
    t2_i = 90 #deg

    

    e1 = t1_f-t1_i
    e2 = t2_f-t2_i
    error = n.transpose([e1,e2])


def rotateEE():
   
    # Creating a ROS node to publish PWM signals to an Arduino subscriber
    pwmPublisher = rospy.Publisher('pwm', Float32, queue_size=1)
    rospy.init_node('pwmNode', anonymous=True)
    secs = 3
    print "Waiting", secs,"seconds..."
    time.sleep(secs)   
 
    print('Control EE to open handle')
    tX = 0 #Degrees
    tF = 100 #Degrees
    
    P = 5
    I = 0
    D = 15

    eTot=0
    eOld=abs(tF-tX)

    error = abs(tF-tX)
    tX_new = 0

    while (error>=1):

        #tX = measured angle

        Calc_PID = C.PID_EE(P,I,D,tF,tX_new,eTot,eOld)
        #Returns eTot, eNew, signal, and tX
        
        eOld = Calc_PID[1]
        eTot = Calc_PID[0]

        signal = Calc_PID[2]

        tX_new = Calc_PID[3] + signal/1000 #TEMPORARY FOR TESTING
        error = abs(tF-tX_new)

        print("Error: " + str(error) + " Signal: " + str(signal))

        #send this to PWM converted representation via Arduino
        pwmPublisher.publish(signal)
        time.sleep(0.1)

        
    nudge = False
    while (nudge == False):

        signal = Calc_PID[2] #Maintain downward position on handle

        #Perform forward kinematics to 'nudge'door and prevent it relocking

        nudge = True

    tF = 0
    tX = tX_new
    e2 = abs(tF-tX)
    print(e2)
    eOld = e2
    eTot = 0 

    P = 5
    I = 0
    D = 10

    while (e2>=1):

        #tX = measured angle

        Calc_PID2 = C.PID_EE(P,I,D,tF,tX_new,eTot,eOld)
        #Returns eTot, eNew, signal, and tX
        
        eOld = Calc_PID2[1]
        eTot = Calc_PID2[0]

        signal = Calc_PID2[2]

        tX_new = Calc_PID2[3] - signal/1000 #TEMPORARY FOR TESTING
        e2 = abs(tF-tX_new)

        print("Error: " + str(e2) + " Signal: " + str(signal))
        #send this to PWM converted representation via Arduino
        pwmPublisher.publish(signal)
        time.sleep(0.1)


    while True:
        continue



