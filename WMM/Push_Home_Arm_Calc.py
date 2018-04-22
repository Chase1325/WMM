import math as m
#import sympy as S
import numpy as n
import Kinematics as Kin
import Controllers as C
import time
from collections import deque
import rospy
from std_msgs.msg import Float32


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

# LIFO queues to handle incoming joint angle data from the ROS subscribers
joint1_queue = deque([], maxlen = 5)
joint2_queue = deque([], maxlen = 5)
joint3_queue = deque([], maxlen = 5)
            
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


def joint1Callback(msg):
    joint1_queue.append(msg.data)

def joint2Callback(msg):
    joint2_queue.append(msg.data)

def joint3Callback(msg):
    joint3_queue.append(msg.data)


#Pass in the required joint angles from inverse kinematics
def home2Handle_Control(t1_f,t2_f):
    print('Drive Arm to needed position')

    rospy.init_node('pwmNode', anonymous=True)
    pwmPublisher1 = rospy.Publisher('pwm1', Float32, queue_size=1)
    pwmPublisher2 = rospy.Publisher('pwm2', Float32, queue_size=1)
    rospy.Subscriber("joint1_theta", Float32, joint1Callback)
    rospy.Subscriber("joint2_theta", Float32, joint2Callback)
    
    secs = 3
    print ("Waiting", secs,"seconds...")
    time.sleep(secs)
    
    t1_f = 0
    t2_f = 0
    t1_i = -90 #deg
    t2_i = 90 #deg
    e1 = abs(t1_f-t1_i)
    e2 = abs(t2_f-t2_i)
    #error = n.transpose([e1,e2])

    #Joint Control independently for now:

    #Tune for each joint
    P = 6
    I = 0.1
    D = 10

    eTot=0
    eOld=e1

    dir=0

    tX_new = 0
    while (e1>=1):

        #tX = measured angle
        tX_new = n.interp(joint1_queue.pop(),[35,275],[-120,120])
        
        print("Popped from t1 bitches: " + str(tX_new))
        
        if(abs(t1_f-tX_new)<=1):
            break;

        if(t1_f<tX_new):
            dir = 0
        else:
            dir = 1

        Calc_PID = C.PID_EE(P,I,D,t1_f,tX_new,eTot,eOld,dir)
        #Returns eTot, eNew, signal, and tX
        
        eOld = Calc_PID[1]
        eTot = Calc_PID[0]

        signal_j1 = Calc_PID[2]

        #send this to PWM converted representation via Arduino
        pwmPublisher1.publish(signal_j1)
        time.sleep(0.1)

        #tX_new = n.interp(joint1_queue.pop(),[35,275],[-120,120]) # + signal_j1/1000 #TEMPORARY FOR TESTING
        #e1 = abs(t1_f-tX_new)
        
        #print("Error: " + str(e1) + " Signal: " + str(signal_j1))


    P = 1
    I = 0.1
    D = 10

    eTot=0
    eOld=e2

    tX_new = 0
    while (e2>=1):

        #tX = measured angle
        tX_new = n.interp(joint2_queue.pop(),[59,299],[120,-120])
        print("Joint 2: " + str(tX_new))

        if(abs(t2_f-tX_new)<=1):

            break;

        if(t2_f<tX_new):
            dir = 0
        else:
            dir = 1

        Calc_PID = C.PID_EE(P,I,D,t2_f,tX_new,eTot,eOld,dir)
        #Returns eTot, eNew, signal, and tX
        
        eOld = Calc_PID[1]
        eTot = Calc_PID[0]

        signal_j2 = Calc_PID[2]

        #send this to PWM converted representation via Arduino
        pwmPublisher2.publish(signal_j2)
        time.sleep(0.1)

        #tX_new = n.interp(joint2_queue.pop(),[59,299],[-120,120]) #- signal_j2/1000 #TEMPORARY FOR TESTING
        #e2 = abs(t2_f-tX_new)
        
        #print("Error: " + str(error) + " Signal: " + str(signal_j2))


def rotateEE():
   
    # Creating a ROS node to publish PWM signals to an Arduino subscriber
    rospy.init_node('pwmNode', anonymous=True)
    pwmPublisher3 = rospy.Publisher('pwm3', Float32, queue_size=1)
    rospy.Subscriber("joint3_theta", Float32, joint3Callback)
    
    secs = 3
    print ("Waiting", secs,"seconds...")
    time.sleep(secs)   
 
    print('Control EE to open handle')
    tX = 0 #Degrees
    tF = -100 #Degrees
    
    P = 5
    I = 0
    D = 15

    eTot=0
    eOld=abs(tF-tX)

    e3 = abs(tF-tX)
    tX_new = 0

    while (e3>=1):

        #tX = measured angle
        tX_new = joint3_queue.pop()

        if(tF<tX_new):
            dir = 0
        else:
            dir = 1

        Calc_PID = C.PID_EE(P,I,D,tF,tX_new,eTot,eOld,dir)
        #Returns eTot, eNew, signal, and tX
        
        eOld = Calc_PID[1]
        eTot = Calc_PID[0]

        signal_j3 = Calc_PID[2]

        #send this to PWM converted representation via Arduino
        pwmPublisher3.publish(signal_j3)
        time.sleep(0.1) # TODO need to change time delay

        tX_new = joint3_queue.pop() #+ signal_j3/1000 #TEMPORARY FOR TESTING
        e3 = abs(tF-tX_new)

        #print("Error: " + str(e3) + " Signal: " + str(signal_j3))

        

        
    nudge = False
    while (nudge == False):

        signal = Calc_PID[2] #Maintain downward position on handle

        #Perform forward kinematics to 'nudge'door and prevent it relocking

        nudge = True

    tF = 0
    tX = tX_new
    e4 = abs(tF-tX)
    print(e4)
    eOld = e4
    eTot = 0 

    P = 5
    I = 0
    D = 10

    while (e4>=1):

        #tX = measured angle
        tX_new = joint3_queue.pop()

        if(tF<tX_new):
            dir = 0
        else:
            dir = 1

        Calc_PID2 = C.PID_EE(P,I,D,tF,tX_new,eTot,eOld,dir)
        #Returns eTot, eNew, signal, and tX
        
        eOld = Calc_PID2[1]
        eTot = Calc_PID2[0]

        signal_j3 = Calc_PID2[2]

        pwmPublisher3.publish(signal_j3)
        time.sleep(0.1) # TODO need to change time delay

        tX_new = joint3_queue.pop() #- signal_j3/1000 #TEMPORARY FOR TESTING
        e4 = abs(tF-tX_new)

        #print("Error: " + str(e4) + " Signal: " + str(signal_j3))
        #send this to PWM converted representation via Arduino
    


    while True:
        continue



