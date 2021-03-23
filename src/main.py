from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
fig = figure()
ax = Axes3D(fig)
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
dt = 0.05


def draw_scene3D(ax, p, R, α, f):
    f = f.flatten()
    theta_rear = arctan2(f[5], f[4])
    theta_right = arctan2(f[3], f[2])
    theta_left = arctan2(f[1], f[0])
    draw_RUR(ax, R, p, α, theta_rear, theta_right, theta_left)


def pd(t): return array([[sin(0.3*t)], [cos(0.4*t)], [-10+0.1*sin(0.3*t)]])


def dpd(t):
    return array(
        [[0.3*cos(0.3*t)], [-0.4*sin(0.4*t)], [0.1*0.3*cos(0.3*t)]])


def ddpd(t):
    return array([[-0.3*0.3*sin(0.3*t)],
                  [-0.4*0.4*cos(0.4*t)], [-0.1*0.3*0.3*sin(0.3*t)]])


def Rd(t): return expw([[sin(t)], [cos(2*t)], [t]])
def dRd(t): return (1/(2*dt))*(Rd(t+dt)-Rd(t-dt))
def ddRd(t): return (1/(2*dt))*(dRd(t+dt)-dRd(t-dt))


def orientator(t, p, R, vr, wr):
    # proportional and derivative 2nd-order controller with a time of response = 1s
    α0 = 1
    α1 = 2
    RT = R.T
    RdT = Rd(t).T
    dRdT = dRd(t).T

    wd = adjoint_inv(dRd(t)@RdT)
    dwd = adjoint_inv(dRd(t)@dRdT + ddRd(t)@RdT)
    er = RT@adjoint_inv(logm(Rd(t)@RT))
    der = -wr + RdT@wd
    dwrd = (adjoint(wr)@RdT + dRdT)@wd + RdT@dwd + α1*der + α0*er
    return dwrd


def positioner(t, p, R, vr, wr):
    # proportional and derivative 2nd-order controller with a time of response = 1s
    α0 = 1
    α1 = 2
    dvrd = R.T@ddpd(t)+α1*(R.T@dpd(t)-vr)+α0*R.T@(pd(t)-p)-adjoint(wr)@vr
    return dvrd


def clock_RUR(p, R, vr, wr, f):
    # Euler method with state equations = evolution function
    p = p+dt*R@vr
    R = R@expm(adjoint(dt*wr))
    vr = vr+dt*0.5*(-adjoint(wr)@vr+(R.T) @
                    array([[0], [0], [g]]) + (l/m)*D@f - 0.5*Cx*Sx*rho*vr**2)
    wr = wr+dt*(inv(I)@(-adjoint(wr)@I@wr+Rτ@f))
    return p, R, vr, wr


def control(t, p, R, vr, wr):
    # controller
    dwrd = orientator(t, p, R, vr, wr)  # desired rotational speed
    dvrd = positioner(t, p, R, vr, wr)  # desired translational speed
    τrd = I@dwrd+adjoint(wr)@I@wr  # desired torques
    frd = m*dvrd-m*(R.T)@array([[0], [0], [g]]) - \
        m*adjoint(wr)@vr  # desired forces
    f = inv(C)@vstack((frd, τrd))
    return f


# positions of the rotors, all blades have the same pitch
Q = array([[0, -a, 0, a, 0, 0], [0, 0, 0, 0,  0, b], [a, 0, -a, 0,  -b, 0]])
# orientation of the forces
D = array([[1, 0, 1, 0, 0, 0], [0, 0, 0, 0, 1, 0], [0, 1, 0, 1, 0, 1]])
N = D.shape[1]
# orientation of the torques
Rτ = hstack((adjoint(Q[:, i]) @ D[:, i].reshape(3, 1) for i in range(0, N)))
# concentrator
C = vstack((D, Rτ))


p = array([[10], [0], [-20]])  # x,y,z (front,right,down)
R = eulermat(0.2, 0.3, 0.4)
vr = array([[0], [0], [0]])
wr = array([[0], [0], [0]])
α = zeros((N, 1))


for t in arange(0, 10, dt):
    # f = 0.1*array([[1], [1], [1], [1], [1], [1]])
    f = control(t, p, R, vr, wr)
    p, R, vr, wr = clock_RUR(p, R, vr, wr, f)
    clean3D(ax, -20, 20, -20, 20, 0, 25)
    draw_scene3D(ax, p, R, α, f)
    α = α + dt * 30 * f
    pause(0.001)
pause(10)
