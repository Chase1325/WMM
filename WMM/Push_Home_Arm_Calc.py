import math as m
#import sympy as S
import numpy as n
import Kinematics as Kin
import Controllers as C
import time
#import rospy
#from std_msgs.msg import Float32


L1 = 0.50973
L2 = 0.44786
height = 0 #Distance from Wheel base to Plane of arm
q1 = -(m.pi)/2
q2 = (m.pi)/2

#t1,t2 = S.symbols('t1 t2')


#DH Paremeters: [theta, d, a, alpha]
dh_param = [[0, height, 0, 0], # Link 0
            [L1, 0, 0, q1], # Link 1
            [L2, 0, 0, q2]]  # Link 2

def calcIK():
    t1=0
    t2=0
    Hx = L1        #Distance in X to door handle
    Hy = L2      #Distance in Y to door handle
    
    ForwardPos = Kin.RZ(q1)*Kin.trans(0,L1,0)*Kin.RZ(q2)*Kin.trans(0,L2,0)
    print(ForwardPos)

    #Need to then Multiply by Translational matricies for EE to Door Handle
    InvPos = Kin.Arm_Inv_Pos_Kin2(ForwardPos,Hx,Hy,L1,L2)
    t1 = n.degrees(InvPos[0])
    t2 = n.degrees(InvPos[1])

    ForwardPos2=Kin.RZ(InvPos[0])*Kin.trans(L1,0,0)*Kin.RZ(InvPos[1])*Kin.trans(L2,0,0)
    print("TEST",ForwardPos2)
    J = Kin.Arm_Jacobian(dh_param,t1,t2)
    print(ForwardPos)
    print(t1,t2)
    print('Calculated IK')
    return t1,t2

#Pass in the required joint angles from inverse kinematics
def home2Handle_Control(t1_f,t2_f):
    print('Drive Arm to needed position')

    #pwmPublisher1 = rospy.Publisher('pwm1', Float32, queue_size=1)
    #pwmPublisher2 = rospy.Publisher('pwm2', Float32, queue_size=1)
    #rospy.init_node('pwmNode', anonymous=True)
    #secs = 3
    #print ("Waiting", secs,"seconds...")
    #time.sleep(secs) 
    t1_f = -90
    t1_i = 0 #deg
    t2_i = 90 #deg
    e1 = abs(t1_f)-t1_i
    e2 = abs(t2_f-t2_i)
    #error = n.transpose([e1,e2])

    #Joint Control independently for now:

    #Tune for each joint
    P = 1
    I = 0
    D = 5

    eTot=0
    eOld=e1

    tX_new = 0
    while (e1>=1):

        #tX = measured angle

        Calc_PID = C.PID_EE(P,I,D,t1_f,tX_new,eTot,eOld)
        #Returns eTot, eNew, signal, and tX
        
        eOld = Calc_PID[1]
        eTot = Calc_PID[0]

        signal_j1 = Calc_PID[2]

        tX_new = Calc_PID[3]  + signal_j1/1000 #TEMPORARY FOR TESTING
        error = abs(t1_f+tX_new)
        e1 = error

        print("Error: " + str(error) + " Signal: " + str(signal_j1))

        #send this to PWM converted representation via Arduino
        #pwmPublisher1.publish(signal_j1)
        time.sleep(0.1)

    P = 5
    I = 0
    D = 15

    eTot=0
    eOld=e2

    tX_new = 0
    while (e2>=1):

        #tX = measured angle

        Calc_PID = C.PID_EE(P,I,D,t2_f,tX_new,eTot,eOld)
        #Returns eTot, eNew, signal, and tX
        
        eOld = Calc_PID[1]
        eTot = Calc_PID[0]

        signal_j2 = Calc_PID[2]

        tX_new = Calc_PID[3] - signal_j2/1000 #TEMPORARY FOR TESTING
        error = abs(t2_f-tX_new)
        e2 = error
        print("Error: " + str(error) + " Signal: " + str(signal_j2))

        #send this to PWM converted representation via Arduino
        #pwmPublisher2.publish(signal_j2)
        #time.sleep(0.1)





def rotateEE():
   
    # Creating a ROS node to publish PWM signals to an Arduino subscriber
    #pwmPublisher3 = rospy.Publisher('pwm3', Float32, queue_size=1)
    #rospy.init_node('pwmNode', anonymous=True)
    #secs = 3
    #print ("Waiting", secs,"seconds...")
    #time.sleep(secs)   
 
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

        signal_j3 = Calc_PID[2]

        tX_new = Calc_PID[3] + signal_j3/1000 #TEMPORARY FOR TESTING
        error = abs(tF-tX_new)

        print("Error: " + str(error) + " Signal: " + str(signal_j3))

        #send this to PWM converted representation via Arduino
        #pwmPublisher3.publish(signal_j3)
        #time.sleep(0.1)

        
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

        signal_j3 = Calc_PID2[2]

        tX_new = Calc_PID2[3] - signal_j3/1000 #TEMPORARY FOR TESTING
        e2 = abs(tF-tX_new)

        print("Error: " + str(e2) + " Signal: " + str(signal_j3))
        #send this to PWM converted representation via Arduino
        #pwmPublisher3.publish(signal_j3)
        #time.sleep(0.1)


    while True:
        continue



