from roblib import *
import math

fig = figure()
ax = Axes3D(fig)

dt = 0.1

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


def draw_scene3D(ax, p, R, α, f):
    f = f.flatten()
    theta_rear = -arctan2(f[5], f[4])
    theta_right = -arctan2(f[3], f[2])
    theta_left = -arctan2(f[1], f[0])
    draw_RUR(ax, R, p, α, theta_rear, theta_right, theta_left)


def pd(t):
    # return 20*array([[sin(t)], [sin(2*t)], [1 + 0.1*sin(0.3*t)]])
    # return array([[20*cos(0.5*t)], [20*sin(0.5*t)], [0]])
    # return array([[0], [0], [0]])
    return array([[20+20*cos(t)], [20*sin(t)], [20*sin(0.5*t)]])


def dpd(t):
    # return 20*array([[cos(t)], [2*cos(2*t)], [0.03*cos(0.3*t)]])
    # return 20*array([[-sin(0.5*t)], [cos(0.5*t)], [0]])
    # return array([[0], [0], [0]])
    return array([[-20*sin(t)], [20*cos(t)], [10*cos(0.5*t)]])


def ddpd(t):
    # return 20*array([[-sin(t)], [-4*sin(2*t)], [-0.009*sin(0.3*t)]])
    # return 20*array([[-cos(0.5*t)], [-sin(0.5*t)], [0]])
    # return array([[0], [0], [0]])
    return array([[-20*cos(t)], [-20*sin(t)], [-5*sin(0.5*t)]])


# # project the estimated position p on the line of unit vector xk, passing by A
# def proj_on_line(p, xk, A):
#     P = A + (p - A).T[0].dot(xk.T[0]) * xk  # projection point
#     e = np.linalg.norm(P - p, 2)  # distance to the line
#     return P, e


def normalize(v):
    if v[0, 0] == 0 and v[1, 0] == 0 and v[2, 0] == 0:
        return v
    else:
        return v / np.linalg.norm(v, 2)


def cross_col(a, b): return np.cross(a.T, b.T).T


# def step1(self, Rk, lk, p):  # from {Rk,lk,p} to {Rd}
#     # rotation of angle θe along the axis ew to reduce the distance to the line e
#     self.Rd = Rk
#     xk = Rk[:, [0]]
#     P, e = proj_on_line(p, xk, lk[:, [0]])
#     if e > 0:
#         θe = 0.5 * math.atan(e)  # gain 0.5 to reduce overshooting
#         mp = normalize(P - p)
#         ew = cross_col(xk, mp)
#         self.Rd = expw(θe * ew) @ Rk


def f_Rd(t):
    dp = -dpd(t)
    # p1 = pd(t)
    # p2 = pd(t+dt)
    # psy = angle(dp)
    # theta, R = angle3d(p2-p1, p1 + dt*dp)
    # theta, R = angle3d(p1, dp)
    # return R
    # return eulermat(0, 0, psy)
    # return expw([[arctan2(dp[2], dp[1])], [arctan2(dp[2], dp[0])], [arctan2(dp[1], dp[0])]])
    # return expw([[p[0] + dt*dp[0]], [p[1] + dt*dp[1]], [p[2] + dt*dp[2]]])
    # return expw([[0], [0], [arctan2(dp[1], dp[0])]])
    up = array([[0], [0], [1]])
    xaxis = normalize(dp)
    # print(direction)
    yaxis = cross_col(xaxis, up)
    yaxis = normalize(yaxis)
    zaxis = cross_col(xaxis, yaxis)
    zaxis = normalize(zaxis)
    R = array([[xaxis[0, 0], xaxis[1, 0], xaxis[2, 0]], [
              yaxis[0, 0], yaxis[1, 0], yaxis[2, 0]], [zaxis[0, 0], zaxis[1, 0], zaxis[2, 0]]])
    #print(R)
    # tang = arctan2(dp[2]-pd_[2],dp[1]-pd_[1])
    # cap = arctan2(dp[1]-pd_[1],dp[0]-pd_[0])
    
    # Rdes = expw([[0],[0],[cap]])

    return expm(pi * adjoint(array([[1], [0], [0]])))@R@expm((-pi/3)* adjoint(array([[0], [0], [1]])))#@expm((-pi/2 + pi/10)* adjoint(array([[0], [0], [1]])))
    #return eulermat(0, 0, 0)


def f_dRd(t):
    return (1/(2*dt))*(f_Rd(t+dt)-f_Rd(t-dt))
    #return eulermat(0, 0, 0)


def f_ddRd(t):
    return (1/(2*dt))*(f_dRd(t+dt)-f_dRd(t-dt))
    #return eulermat(0, 0, 0)


def positioner(p, R, vr, wr, t):
    Rt = inv(R)
    dvrd = Rt@ddpd(t) + 2*(Rt@dpd(t) - vr) + Rt@(pd(t)-p) - adjoint(wr)@vr
    return dvrd


def orientator(p, R, vr, wr, t):
    Rt = np.transpose(R)
    Rd = f_Rd(t)
    Rdt = np.transpose(Rd)
    dRd = f_dRd(t)
    dRdt = np.transpose(dRd)
    ddRd = f_ddRd(t)

    wd = adjoint_inv(dRd@Rdt)
    dwd = adjoint_inv(dRd@dRdt + ddRd@Rdt)
    if norm(Rd, 2) == 0:
        er = array([[0], [0], [0]])
    else:
        er = Rt@adjoint_inv(logm(Rd@Rt))
    der = -wr + Rdt@wd
    dwrd = (adjoint(wr)@Rdt + dRdt)@wd + Rdt@dwd + 2*der + er

    return dwrd


def control(p, R, vr, wr, t):
    # Calcul de fr
    dvrd = positioner(p, R, vr, wr, t)
    fr = m*dvrd - m*inv(R)@array([[0], [0], [g*0]]) - m*adjoint(wr)@vr

    # Calcul de tau_r
    dwrd = orientator(p, R, vr, wr, t)
    tau_r = I@dwrd + adjoint(wr)@I@wr

    return fr, tau_r


# positions of the rotors, all blades have the same pitch
Q = array([[0, -a, 0, a, 0, 0], [0, 0, 0, 0,  0, b], [a, 0, -a, 0,  -b, 0]])
# orientation of the forces
D = array([[1, 0, 1, 0, 0, 0], [0, 0, 0, 0, 1, 0], [0, 1, 0, 1, 0, 1]])
N = D.shape[1]
# concentrator
C = vstack((D, Q))
inv_C = inv(C)


def draw_platform(ax, p, R):
    lz = 5*l
    T = tran3H(*-p) @ ToH(R)
    M = T @ add1([[lz, -lz, -lz, lz, lz],
                  [lz, lz, -lz, -lz, lz], [0, 0, 0, 0, 0]])

    # Corps du robot
    T2 = tran3H(*-p) @ ToH(R)
    M2 = T2@cylinder3H(1, 10)
    draw3H(ax, M2, 'grey', False, -1)

    # tête du robot
    sphere = T2@tran3H(10, 0, 0)@draw_sphere3D(1)
    draw3H(ax, sphere, 'grey', False, -1)

    draw3H(ax, M, 'grey', False, -1)


def clock_RUR(p, R, vr, wr, f, t):
    # Récupération de fr et tau_r
    fr, tau_r = control(p, R, vr, wr, t)

    # Equation d'état et Euler combinés
    p = p+dt * R@vr
    vr = vr + dt * (-adjoint(wr)@vr + inv(R) @
                    array([[0], [0], [g*0]]) + (1/m)*fr)
    R = R@expm(adjoint(dt*wr))
    wr = wr+dt*(inv(I)@(-adjoint(wr)@I@wr + tau_r.reshape(3, 1)))

    # Evolution de f
    v_fr_tau_r = vstack((fr, tau_r))
    f = inv_C@v_fr_tau_r
    return p, R, vr, wr, f


p = array([[20], [0], [0]])  # x,y,z (front,right,down)
R = eulermat(0.2, 0.3, 0.4)
vr = array([[0], [0], [0]])
wr = array([[0], [0], [0]])
α = zeros((N, 1))

f = 0.1*array([[1], [1], [1], [1], [1], [1]])
clean3D(ax, -20, 20, -20, 20, -25, 25)
i = 0

z=linspace(0,20,400)

liste_pos=[p]


for t in arange(0, 20, dt):
    ti = t
    p, R, vr, wr, f = clock_RUR(p, R, vr, wr, f, ti)
    liste_pos.append(p)

    clean3D(ax, -30, 30, -30, 30, -25, 25)
    # if i % 10 == 0:

    for pos in liste_pos :
        ax.scatter(pos[0,0], pos[1,0], pos[2,0], c='red', marker='.')

    ax.plot(20+20*cos(z), 20*sin(z), 20*sin(0.5*z))

    draw_scene3D(ax, p, R, α, f)
    # draw_platform(ax, pd(t), f_Rd(t))
    # plot3D(ax, p, 'red', 1)
    ax.scatter(p[0, 0], p[1, 0], p[2, 0], c='red',
               marker='o')  # actual position
    ax.scatter(pd(ti)[0, 0], pd(ti)[1, 0], pd(ti)[2, 0],
               c='blue', marker='o')  # desired trajectory
    ax.scatter(20, 0, 20*sin(0.5*ti),
               c='green', marker='o')  # desired trajectory
    #ax.quiver(p[0, 0], p[1, 0], p[2, 0], 5*dt*dpd(t)[0],
     #          5*dt*dpd(t)[1], 5*dt*dpd(t)[2])
    Rd = f_Rd(ti)
    #draw_scene3D(ax, p, Rd, α, f)
    #φ, θ, ψ = eulermat2angles(Rd)
    # print("phi =", φ)
    # print("theta =", θ)
    # print("psy =", ψ)
    α = α + dt * 30 * f
    #i += 1
    pause(0.001)

pause(10)
