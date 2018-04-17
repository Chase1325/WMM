import numpy as n
#import sympy as s
import math as m

def dhparam2matrix(dh_param_i, qi):
    M1 = n.matrix([[m.cos(qi + dh_param_i[0]), -1*m.sin(qi + dh_param_i[0]), 0, 0], [m.sin(qi + dh_param_i[0]), m.cos(qi + dh_param_i[0]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    M2 = n.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, dh_param_i[1]], [0, 0, 0, 1]])
    M3 = n.matrix([[1, 0, 0, dh_param_i[2]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    M4 = n.matrix([[1, 0, 0, 0], [0, m.cos(dh_param_i[3]), -1*m.sin(dh_param_i[3]), 0], [0, m.sin(dh_param_i[3]), m.cos(dh_param_i[3]), 0], [0, 0, 0, 1]])
    T = M1 * M2 * M3 * M4;
    print(T)
    return T

def Arm_F_Pos_Kin(dh_param, q1, q2):
    T_Rob_0 = dhparam2matrix(dh_param[0],0)
    T_0_1 = dhparam2matrix(dh_param[1], q1)
    T_1_2 = dhparam2matrix(dh_param[2], q2)
    T_Rob_2 = T_Rob_0 * T_0_1 * T_1_2
    return T_Rob_2

def Arm_Inv_Pos_Kin(T,Hx,Hy,Ax,Ay):

    #T_inv = n.linalg.inv(T)
    X2_cos = (m.pow(Hx,2)+m.pow(Hy,2)-m.pow(Ay,2)-m.pow(Ax,2))/(2*Ax*Ay)
    print(X2_cos)
    X2_sin = m.pow(1-m.pow(X2_cos,2),1/2)
    print(X2_sin)
    t2 = m.atan2(-X2_sin,X2_cos)
    print(t2)

    X1_cos = ((Hx*(Ay*X2_cos+Ax)+Hy*Ay*X2_sin)/(m.pow(Ay*X2_cos+Ax,2)+m.pow(Ay*X2_sin,2)))
    X1_sin = ((Hy*(Ax*X2_cos+Ax)-Hx*Ay*X2_sin)/(m.pow(Ay*X2_cos+Ax,2)+m.pow(Ay*X2_sin,2)))
    t1 = m.atan2(X1_cos,X1_sin)
    
    return t1, t2

def Arm_Inv_Pos_Kin2(T,Hx,Hy,Ax,Ay):

    t2 = m.acos((m.pow(Hx,2)+m.pow(Hy,2)-m.pow(Ax,2)-m.pow(Ay,2))/(2*Ax*Ay))
    print(t2)
    alpha = m.atan2(Hy,Hx)
    t1 = alpha - m.acos((m.pow(Hx,2)+m.pow(Hy,2)+m.pow(Ax,2)-m.pow(Ay,2))/(2*Ax*m.pow((m.pow(Hx,2)+m.pow(Hy,2)),1/2)))
    print(t1)
    
    return t1, t2

def Arm_Jacobian(dh_param, q1, q2):
   J = n.matrix([[m.sin(q1) * dh_param[1][2] + m.sin(q1 + q2) * dh_param[2][2], m.sin(q1 + q2) * dh_param[2][2]], 
                  [m.cos(q1) * dh_param[1][2] + m.cos(q1 + q2) * dh_param[2][2], m.cos(q1 + q2) * dh_param[2][2]]])
   return J

def Arm_Inv_Vel_Kin(J, x_dot, y_dot):
   q_dot = n.linalg.inv(J) * n.matrix([[x_dot], [y_dot]])
   print(q_dot)
   return q_dot

def Base_Vel_Kin(radius, length, theta, phi1dot, phi2dot):
   constraints = n.matrix([[1, 0, length], [1, 0, -1*length], [0, 1, 0]])
   xi_base = Rot_W_Rob(theta)*n.linalg.inv(constraints)*(radius*n.matrix([[phi1dot], [phi2dot], [0]]))
   return xi_base

def Rot_W_Rob(theta):
   R = n.matrix([[m.cos(theta), -1*m.sin(theta), 0], [m.sin(theta), m.cos(theta), 0], [0, 0, 1]])
   return R

def RX(q):
    rx = n.matrix([[1, 0, 0, 0],[0, m.cos(q), -m.sin(q), 0], [0, m.sin(q), m.cos(q), 0], [0,0,0,1]])
    return rx

def RY(q):
    ry = n.matrix([[m.cos(q), 0, m.sin(q), 0],[0, 1, 0, 0], [-m.sin(q), 0, m.cos(q), 0], [0,0,0,1]])
    return ry

def RZ(q):
    rz = n.matrix([[m.cos(q), -m.sin(q), 0, 0],[m.sin(q), m.cos(q), 0, 0], [0, 0, 1, 0], [0,0,0,1]])
    return rz

def trans(x,y,z):
    T = n.matrix([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
    return T