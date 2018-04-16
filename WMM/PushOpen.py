import string, sys
import Push_Home_Arm_Calc as PHAC
#import State as S
#import StateMachine as SM

#PushOpen State Machine

#PARAMETERS
HomeX = 0
HomeY = 0
HomeZ = 0


print('Inside Push Open')

#OPERATION 1: CALCULATE ARM IK
print('Calculate Arm IK')
invKin = PHAC.calcIK()

#OPERATION 2: CONTROL ARM MOTION
print('Control Arm Motion')
PHAC.home2Handle_Control(invKin[0],invKin[1])

#OPERATION 3: OPEN DOOR HANDLE
print('Rotate EE')
PHAC.rotateEE()

#OPERATION 6: PASS THROUGH THE DOOR
print('Pass Through Door')
#PHHC.hybridControl()

#OPERATION 7: RETRACT ARM/CLEAR DOOR
print('Retract Arm') 
#PHAC.