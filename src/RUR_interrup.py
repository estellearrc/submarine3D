from roblib import * 
import math


# Initialisation 
fig = figure()
ax = Axes3D(fig)

dt = 0.05

Cx = 1
# AUV caracteristics
m, g, w, l = 2*10, 9.81, 0.12, 1
R, H = 0.5*w, 0.5*l
Sx = pi*R**2
rho = 1
# propeller caracteristics
wp, lp = 0.04, 0.1
rp, hp = 0.5*wp, 0.5*lp
a, b = R + rp, H + hp
# inertia matrix
I = array([[0.5*m*R**2, 0, 0], [0, 0.25*m*R**2+(m*H**2)/3, 0],
        [0, 0, 0.25*m*R**2+(m*H**2)/3]])

# positions of the rotors, all blades have the same pitch
Q = array([[0, -a, 0, a, 0, 0], [0, 0, 0, 0,  0, b], [a, 0, -a, 0,  -b, 0]])
# orientation of the forces
D = array([[1, 0, 1, 0, 0, 0], [0, 0, 0, 0, 1, 0], [0, 1, 0, 1, 0, 1]])
N = D.shape[1]
# concentrator
C = vstack((D, Q))
inv_C = inv(C)

p = array([[10], [0], [0]])  #x,y,z (front,right,down)
R = eulermat(0.2,0.3,0.4)
vr = array([[0], [0], [0]])
wr = array([[0], [0], [0]])
α=zeros((N,1))

f = 0.1*array([[1],[1],[1],[1],[1],[1]])
t = 0
α = 0
commande = "forward"

p_g = array([[10], [0], [0]])
R_g = eulermat(0,0,0)
vr_g = array([[0], [0], [0]])
wr_g = array([[0], [0], [0]])


def normalize(v):
    if v[0, 0] == 0 and v[1, 0] == 0 and v[2, 0] == 0:
        return v
    else:
        return v / np.linalg.norm(v, 2)

def cross_col(a, b): return np.cross(a.T, b.T).T

def draw_scene3D(ax, p, R, α, f):
    f = f.flatten()
    theta_rear = -arctan2(f[5], f[4])
    theta_right = -arctan2(f[3], f[2])
    theta_left = -arctan2(f[1], f[0])
    draw_RUR(ax, R, p, α, theta_rear, theta_right, theta_left)


def pd(t):  
    if commande == "stay" :
        return array([[p_g[0,0]], [p_g[1,0]], [p_g[2,0]]])

    if commande == "forward" :
        return array([[p_g[0,0] - t], [p_g[1,0]], [p_g[2,0]]])
    
    if commande == "backward" :
        return array([[p_g[0,0] + t], [p_g[1,0]], [p_g[2,0]]])

    if commande == "up" :
        return array([[p_g[0,0]], [p_g[1,0]], [p_g[2,0]+t]])

    if commande == "down" :
        return array([[p_g[0,0]], [p_g[1,0]], [p_g[2,0]-t]])

    if commande == "right" :
        return array([[p_g[0,0]], [p_g[1,0]+t], [p_g[2,0]]])

    if commande == "left" :
        return array([[p_g[0,0]], [p_g[1,0]-t], [p_g[2,0]]])
    
    if commande == "observe": 
        return array([[5+20*cos(t) + p_g[0,0]], [20*sin(t) + p_g[1,0]], [p_g[2,0]]])



def dpd(t) :
    if commande == "stay" :
        return array([[0],[0],[0]])
    if commande == "forward": 
        return array([[1],[0],[0]])
    if commande == "backward": 
        return array([[-1],[0],[0]])
    if commande == "up": 
        return array([[0],[0],[1]])
    if commande == "down": 
        return array([[0],[0],[-1]])
    if commande == "right": 
        return array([[0],[1],[0]])
    if commande == "left": 
        return array([[0],[-1],[0]])

    if commande == "observe": 
        return array([[-20*sin(t)], [20*cos(t)], [0]])

def ddpd(t) :
    if commande == "observe" :
        array([[-20*cos(t)], [-20*sin(t)], [0]])
    return array([[0],[0],[0]])
    

def f_Rd(t):  
    if commande == "up": 
        return R_g@expw([[0], [pi/2], [0]])
    if commande == "down": 
        return R_g@expw([[0], [-pi/2], [0]])
    if commande == "right": 
        return R_g@expw([[0], [0], [pi/2]])
    if commande == "left": 
        return R_g@expw([[0], [0], [-pi/2]])

    if commande == "stay": 
        return expw([[0], [0], [0]])

    if commande == "observe": 
        dp = -dpd(t)
        up = array([[0], [0], [1]])
        xaxis = normalize(dp)
        # print(direction)
        yaxis = cross_col(xaxis, up)
        yaxis = normalize(yaxis)
        zaxis = cross_col(xaxis, yaxis)
        zaxis = normalize(zaxis)
        R = array([[xaxis[0, 0], xaxis[1, 0], xaxis[2, 0]], [
                yaxis[0, 0], yaxis[1, 0], yaxis[2, 0]], [zaxis[0, 0], zaxis[1, 0], zaxis[2, 0]]])

        return R_g@expm(pi * adjoint(array([[1], [0], [0]])))@R@expm((-pi/3)* adjoint(array([[0], [0], [1]])))@expm((-pi/2 + pi/10)* adjoint(array([[0], [0], [1]])))

    return  R_g@expw([[0], [0], [0]])

def f_dRd(t): return (1/(2*dt))*(f_Rd(t+dt)-f_Rd(t-dt))
def f_ddRd(t): return (1/(2*dt))*(f_dRd(t+dt)-f_dRd(t-dt))

def positioner(p,R,vr,wr,t):
    Rt = inv(R)
    dvrd = Rt@ddpd(t) + 2*(Rt@dpd(t) - vr) + Rt@(pd(t)-p) - adjoint(wr)@vr
    return dvrd

def orientator(p,R,vr,wr,t):
    Rt = np.transpose(R)
    Rd = f_Rd(t)
    Rdt = np.transpose(Rd)
    dRd = f_dRd(t)
    dRdt = np.transpose(dRd)
    ddRd = f_ddRd(t)

    wd = adjoint_inv(dRd@Rdt)
    dwd = adjoint_inv(dRd@dRdt + ddRd@Rdt)
    er = Rt@adjoint_inv(logm(Rd@Rt))
    der = -wr + Rdt@wd
    dwrd = (adjoint(wr)@Rdt + dRdt)@wd + Rdt@dwd + 2*der + er

    return dwrd

def control(p,R,vr,wr,t) :
    #Calcul de fr
    dvrd = positioner(p,R,vr,wr,t)
    fr = m*dvrd - m*inv(R)@array([[0],[0],[g*0]]) - m*adjoint(wr)@vr

    #Calcul de tau_r
    dwrd = orientator(p,R,vr,wr,t)
    tau_r = I@dwrd + adjoint(wr)@I@wr

    return fr,tau_r

def draw_platform(ax,pt,R):
    lz=5*l
    T = tran3H(*-pt)
    T[1,3] = -T[1,3] 
    RH = ToH(R)
    M = T@RH@add1([[lz,-lz,-lz, lz,lz],[lz,lz,-lz,-lz,lz],[0,0,0,0,0]])

    # Corps du robot
    T2 = tran3H(*-pt)
    T2[1,3] = -T2[1,3] 
    T2 = T2@ToH(R)
    M2 = T2@cylinder3H(1, 10)
    draw3H(ax,M2,'grey',False,-1)

    # tête du robot
    sphere = T2@tran3H(10, 0, 0)@draw_sphere3D(1)
    draw3H(ax,sphere,'grey',False,-1)

    draw3H(ax,M,'grey',False,-1)


def clock_RUR(p,R,vr,wr,f,t):
    #Récupération de fr et tau_r
    fr,tau_r = control(p,R,vr,wr,t)

    #Equation d'état et Euler combinés
    p=p+dt* R@vr
    vr= vr + dt* (-adjoint(wr)@vr + inv(R)@array([[0],[0],[g*0]]) + (1/m)*fr)
    R=R@expm(adjoint(dt*wr))
    wr=wr+dt*(inv(I)@(-adjoint(wr)@I@wr+ tau_r.reshape(3,1)))

    #Evolution de f
    v_fr_tau_r = vstack((fr,tau_r))
    f = inv_C@v_fr_tau_r
    return p,R,vr,wr,f


def simu(p, R, vr, wr, f,t,α) :
    while(True) :
        try :
            t+=0.1
            p, R, vr, wr,f = clock_RUR(p, R, vr, wr, f,t)

            global p_g
            p_g = p

            global vr_g
            vr_g = vr

            global wr_g
            wr_g = wr

            global f_g
            f_g = f

            clean3D(ax, -50, 50, -50, 50, 0, 50)

            #draw_platform(ax, pd(t), f_Rd(t))
            draw_scene3D(ax, p, R, α, f)

            α = α + dt * 30 * f
            pause(0.001)

        except KeyboardInterrupt :
            global commande
            commande = input("Entrer une commande : ")
            t = 0
            global R_g
            R_g = R
            simu(p, R, vr, wr, f,t,α)



simu(p, R, vr, wr, f,t,α)
