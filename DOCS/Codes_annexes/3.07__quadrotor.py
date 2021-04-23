from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
fig = figure()
ax = Axes3D(fig)

m, g, b, d, l = 10, 9.81, 2, 1, 1
I = array([[10, 0, 0], [0, 10, 0], [0, 0, 20]])
dt = 0.01
B = array([[b, b, b, b], [-b*l, 0, b*l, 0], [0, -b*l, 0, b*l], [-d, d, -d, d]])


def draw_quadri(x):  # vecteur d'état x,y,z, angles d'Euler
    ax.clear()
    clean3D(ax, -50, 50, -50, 50, 0, 30)
    # we infate the robot, just to see something
    draw_quadrotor3D(ax, x, α, 5*l)  # alpha = angles of the blades


def f_vdp(x):
    # équation de Van der Pol
    x = x.flatten()
    vdpθ = x[1]
    vdp1 = -(0.001*(x[0]**2)-1)*x[1]-x[0]
    dx = array([[vdpθ], [vdp1]])
    return dx


def f(x, w):
    x = x.flatten()
    φ, θ, ψ = x[3:6]
    vr = (x[6:9]).reshape(3, 1)
    wr = (x[9:12]).reshape(3, 1)
    w2 = w*abs(w)
    τ = B@w2.flatten()
    R = eulermat(φ, θ, ψ)  # matrice de rotation de R0 vers R1
    dp = R@vr  # évolution de la position p dans R0
    dvr = -adjoint(wr)@vr + \
        (R.T)@array([[0], [0], [g]])+array([[0], [0], [-τ[0]/m]])
    # avec la matrice de rotation plutôt que les angles d'Euler
    dR = R@expm(adjoint(wr))
    dφθψ = dR @ wr
    # dφθψ = eulerderivative(φ, θ, ψ) @ wr
    dwr = inv(I)@(-adjoint(wr)@I@wr+τ[1:4].reshape(3, 1))
    return vstack((dp, dφθψ, dvr, dwr))


def control(X):
    X = X.flatten()
    x, y, z, φ, θ, ψ = list(X[0:6])
    vr = (X[6:9]).reshape(3, 1)
    wr = (X[9:12]).reshape(3, 1)
    R = eulermat(φ, θ, ψ)
    dp = R@vr
    # backstepping method
    zd = -10
    vd = 10
    fd = f_vdp(array([[x], [y]]))
    # commande proportionnelle dérivée, tanh pour saturer la commande
    τd0 = 300*tanh(z-zd)+60*vr[2]
    # pour que le drone aille dans la direction de fd (gite)
    phi = 0.5*tanh(10*sawtooth(angle(fd)-angle(dp)))
    # pour que le drone accélère dans sa direction d'avancement selon l'axe x (asiette)
    theta = -0.3*tanh(vd-vr[0])
    psy = angle(dp)  # pour que la caméra soit toujours devant (lacet)
    Rd = eulermat(phi, theta, psy)
    # inverse of block 3
    wrd = R.T@adjoint_inv(logm(Rd@R.T))
    # inverse of block 2
    τd123 = I@((200*(wrd-wr))+adjoint(wr)@I@wr)
    # inverse of block 1
    # vitesses w1²,w2²,w3²,w4² de rotation des hélices au carré algébrique
    W2 = inv(B)@vstack(([τd0], τd123))
    w = sqrt(abs(W2))*sign(W2)
    return w


# x,y,z,   φ,θ,ψ   vr  wr (front,right,down)
x = array([[0, 0, -5, 1, 0, 0, 10, 0, 0, 0, 0, 0]]).T
α = array([[0, 0, 0, 0]]).T  # angles for the blades


for t in arange(0, 0.1*50, dt):
    w = control(x)  # consigne
    xdot = f(x, w)
    x = x + dt*xdot  # Euler
    draw_quadri(x)
    α = α+dt*10*w  # makes the blades turning
    pause(0.001)
pause(1)
