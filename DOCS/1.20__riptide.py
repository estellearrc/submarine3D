from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def f(x, u):
    # state equations
    x = x.flatten()
    u = u.flatten()
    u0, u1, u2, u3 = list(u)
    phi, theta, psy = list(x[3:6])
    vr = x[6]  # speed
    wr = vr*B@array([[u1], [u2], [u3]])  # rotation vector
    arx = p1*u0**2-p2*vr**2  # acceleration
    R = eulermat(phi, theta, psy)  # orientation of the riptide
    dp = R@array([[vr], [0], [0]])
    dvr = arx
    dR = eulerderivative(phi, theta, psy)@wr
    return vstack((dp, dR, dvr))


def control(x, arx_bar, wr_bar):
    # compute the command u for the propeller and the 3 fins based on arx_bar and wr_bar
    vr = x[6]
    u0 = sqrt((arx_bar+p2*vr**2)/p1)
    u123 = inv(B)@wr_bar/vr
    u = vstack((u0, u123))
    return u


fig = figure()
ax = Axes3D(fig)
p1, p2, p3, p4 = 1, 1, 1, 1  # parameters
B = array([[-p3, -p3, -p3], [0, p4*sin(2*pi/3), -p4*sin(2*pi/3)],
           [-p4, p4*cos(2*pi/3), -p4*cos(2*pi/3)]])  # coefficients to compute wr
x = array([[0, 0, -5, 0, 0, 0, 0.1]]).T  # x,y,z,φ,θ,ψ,v
α = 0  # angles for the blade
# u0 = command for the propeller, u1,u2,u3 = angles for the fins
u = array([[2], [0], [0], [0]])
dt = 0.1
wr_bar = [[0], [0], [0.1]]  # turning left (around z-axis)
for t in arange(0, 100, dt):
    clean3D(ax, -15, 15, -15, 15, -15, 5)
    draw_riptide(ax, x, u, α)
    α = α+dt*u[0]  # updating the propeller's blade angle
    # acceleration proportional to the error between v_bar = 1m/s and vr=x[6]
    arx_bar = 0.1*(1-x[6])
    u = control(x, arx_bar, wr_bar)
    x = x+dt*f(x, u)  # Euler equation
pause(10)
