# Fifth version of the follow line algorithm
# the lines parameters have been changed to length lk and rotation vector wk

#CE PROGRAMME N'EST PAS LIÉ À NOTRE PROJET. SEULES CERTAINES ÉQUATIONS SONT INTÉRESSANTE À RÉUTILISER

from roblib import *
import pyibex as py
import math
import csv
import random
import transformations as tf
from parameters import _p_0, _p_0_hat, _v_0, _R_0, _noise_d, _noise_pz, _noise_R, _B, B_inv, dt, p1, p2, _Marks, _N, \
    _nb_box, _box_k_init, _Mx, _My, _Mz, u_max, fpz, fd, _Pa, _Pb, L_t, _vd, _current, nl, L_ak, L_Rk, _L_dk, _L_wk, _Rk, _lk, β, _P0


class Riptide:
    # block the of system
    # input {u: control signal}
    # output {y: measurement signal}
    def __init__(self, p_0, v_0, R_0, noise_d, noise_pz, noise_R):
        # initialise with the initial state p0, v0, R0, wr0
        self.p = p_0  # position
        self.v = v_0  # velocity
        self.R = R_0  # orientation matrix

        self.noise_pz = noise_pz
        self.noise_R = noise_R
        self.noise_d = noise_d  # distance noise with marks

    def sys_update(self, u, current=np.zeros((3, 1))):  # update the state of the system (Euler method)
        wr = self.v * _B @ u[1:4]
        self.p = self.p + dt * (self.R @ array([[self.v, 0, 0]]).T + current)
        self.v = self.v + dt * (p1 * u[0, 0] ** 2 - p2 * self.v * abs(self.v))
        self.R = self.R @ expw(dt * wr)

    def measure(self):  # return the output of the system with noise

        # velocity
        y_pz = self.p[2, 0] + rand(self.noise_pz)

        # orientation
        R_tilde = rand_box(self.noise_R).reshape((3, 3))
        y_R = R_tilde @ self.R

        # distance between the position p_hat and the Marks of Mark_list with confidence interval
        y_d = np.zeros((_N, 1))
        for j in range(_N):
            y_d[j, 0] = np.linalg.norm(self.p - np.array([_Marks[j, :]]).T) + rand(self.noise_d)

        return y_pz, y_R, y_d


class Kalman:
    # block of the state estimation
    # input {u: control signal, y: measurement signal}
    # output {px_hat, py_hat, v_hat}
    def __init__(self, P0, Pa, Pb, px_hat0, py_hat0, vr_hat0):
        # initialise the kalman filter
        self.px_hat = px_hat0
        self.py_hat = py_hat0
        self.vr_hat = vr_hat0

        self.gx = np.zeros((_N, 1))  # estimation of the distances from the markers

        self.Pa = Pa  # covariance of noise alpha
        self.Pb = Pb  # covariance of noise beta
        self.P = P0 # covariance matrix of the state

        self.Ak = np.eye(3)
        self.Ck = np.zeros((_N, 3))

    def update_Ak(self, R): # update the evolution matrix Ak
        self.Ak = np.eye(3)
        self.Ak[0, 2] = dt * R[0, 0]
        self.Ak[1, 2] = dt * R[1, 0]
        self.Ak[2, 2] = 1 - 2 * p2 * abs(self.vr_hat)

    def update_Ck(self,pz): # update the evolution matrix Ck
        self.Ck = np.zeros((_N, 3))
        for i in range(_N):
            p = np.array([[self.px_hat, self.py_hat, pz]]).T
            self.gx[i, 0] = np.linalg.norm(p - _Marks[[i], :].T)
            self.Ck[i] = np.array([(p[0, 0] - _Marks[i, 0]) / self.gx[i, 0],
                                   (p[1, 0] - _Marks[i, 1]) / self.gx[i, 0],
                                   0])
    def kalman_predict(self, u, R):  # prediction function of the kalman filter
        self.px_hat = self.px_hat + dt * R[0, 0] * self.vr_hat
        self.py_hat = self.py_hat + dt * R[1, 0] * self.vr_hat
        self.vr_hat = self.vr_hat + dt * (p1 * u[0, 0] ** 2 - p2 * self.vr_hat * abs(self.vr_hat))
        self.P = self.Ak @ self.P @ self.Ak.T + self.Pa

    def kalman_correct(self, y_d):  # correction function of the kalman filter with the measurement vector y
        zk = y_d - self.gx  # innovation
        Sk = self.Ck @ self.P @ self.Ck.T + self.Pb  # covariance of zk
        Kk = self.P @ self.Ck.T @ inv(Sk)  # kalman gain

        state = np.array([[self.px_hat, self.py_hat, self.vr_hat]]).T
        state = state + Kk @ zk
        self.px_hat, self.py_hat, self.vr_hat = state[0, 0], state[1, 0], state[2, 0]
        self.P = (eye(3) - Kk @ self.Ck) @ self.P

    def obs_out(self):  # return estimated state as a vector
        return self.px_hat, self.py_hat, self.vr_hat


class Controller:
    # controller block of the system
    # input {vd:velocity, Rk: line orientation, lk: line coordinates,
    #        p: estimated position of the riptide, R: measured orientation of the riptide,
    #        v: estimated velocity of the riptide}
    # output {u: control signal}
    def __init__(self):
        self.u = np.zeros((4, 1))
        self.Rd = np.eye(3)  # desired orientation matrix

    def controller_update(self, vd, Rk, lk, p, R, v):
        self.step1(Rk, lk, p)
        self.step2(vd, R, v)

    def step1(self, Rk, lk, p):  # from {Rk,lk,p} to {Rd}
        # rotation of angle θe along the axis ew to reduce the distance to the line e
        self.Rd = Rk
        xk = Rk[:, [0]]
        P, e = proj_on_line(p, xk, lk[:, [0]])
        if e > 0:
            θe = 0.5 * math.atan(e)  # gain 0.5 to reduce overshooting
            mp = normalize(P - p)
            ew = cross_col(xk, mp)
            self.Rd = expw(θe * ew) @ Rk

    def step2(self, vd, R, v):  # from (vd,Rd) to u
        self.u[0, 0] = math.sqrt(p2 / p1) * vd  # thruster

        # fins
        if abs(v) > 0.5:
            er = R.T @ logw(self.Rd @ R.T)
            if any(np.iscomplex(er)):  # if Rd @ x.R.T has no real logarithm, add small rotation to Rd
                er = R.T @ logw(eulermat(0, 0, 0.01) @ self.Rd @ R.T)
            self.u[1:4] = 1 / v * B_inv @ (er)

        um = max(abs(self.u[1:4]).T[0])
        if um > u_max:
            self.u[1:4] = (u_max / um) * self.u[1:4]  # saturation


class Record:  # made to memorise variable to then write them in the csv
    def __init__(self, t, p, vr, R, px_hat, py_hat, yz, u, P_p, y_d, joint):
        self.t = t
        self.p = p
        self.vr = vr
        self.R = R
        self.px_hat = px_hat
        self.py_hat = py_hat
        self.yz = yz
        self.u = u
        self.P_p = P_p
        self.y_d = y_d
        self.joint = joint


def proj_on_line(p, xk, A):  # project the estimated position p on the line of unit vector xk, passing by A
    P = A + (p - A).T[0].dot(xk.T[0]) * xk  # projection point
    e = np.linalg.norm(P - p, 2)  # distance to the line
    return P, e


def next_line(p, l):  # return true of the robot thinks it has passed the end of the line, estimating its position as p
    A, B = l[:, [0]], l[:, [1]]
    v1, v2 = p - B, A - B
    if v1.T[0].dot(v2.T[0]) < 0:
        return True
    return False


def normalize(v): return v / np.linalg.norm(v, 2)


def cross_col(a, b): return np.cross(a.T, b.T).T




def rand(In: py.Interval):  # take a random number in the interval
    # return In.lb() + random.random() ** 2 * (In.ub() - In.lb())  for non gaussian
    return In.lb() + random.random() * (In.ub() - In.lb())  # for gaussian


def rand_box(Box: py.IntervalVector):  # take a random vector in the box
    vec = []
    for Inter in Box:
        vec.append(rand(Inter))
    return np.array([vec]).T




def Pa():
    Pa = 0.01*np.eye(3)
    return Pa


def Pb():
    Pb = 100 * eye(_N)
    return Pb


# Blocs
_cont = Controller()
_riptide = Riptide(_p_0, _v_0, _R_0, _noise_d, _noise_pz, _noise_R)
_obs = Kalman(_P0, _Pa, _Pb, _p_0_hat[0,0], _p_0_hat[1,0], _v_0)

# recording lists
L_rec = []

print("simulation start")
for k in range(len(L_t)):

    # measurement and correction
    _y_pz, _y_R, _y_d = _riptide.measure()
    _obs.update_Ck(_y_pz)
    _obs.kalman_correct(_y_d)
    _px_hat, _py_hat, _vr_hat = _obs.obs_out()

    _p_hat = np.array([[_px_hat, _py_hat, _y_pz]]).T
    _P_p = np.zeros((3, 3))
    _P_p[0:2, 0:2] = _obs.P[0:2, 0:2]
    _P_p[2, 2] = _noise_pz.diam()

    # control update
    _cont.controller_update(_vd, _Rk, _lk, _p_hat, _y_R, _vr_hat)
    _joint = np.concatenate((np.array([[β]]), _cont.u[1:4]), 0)
    # record iteration k
    L_rec.append(
        Record(L_t[k], _riptide.p, _riptide.v, _riptide.R, _obs.px_hat, _obs.py_hat, _y_pz, _cont.u, _P_p, _y_d,
               _joint))

    # system update for k+1
    β = β - dt * _cont.u[0, 0]

    _riptide.sys_update(_cont.u, _current)
    _obs.update_Ak(_y_R)
    _obs.kalman_predict(_cont.u, _y_R)

    if next_line(_p_hat, _lk):
        if nl >= len(L_ak[0]) - 2:
            break  # end of the mission
        print("next line")
        nl += 1
        l_old = _lk
        _lk = L_ak[:, nl:nl + 2]
        _Rk = L_Rk[nl]

print("simulation done")
print("saving data")

# save landmark to csv
with open('v6_landmark.csv', 'w', newline='') as csvfile:
    fieldnames = ["lmx", "lmy", "lmz"]
    thewriter = csv.DictWriter(csvfile, fieldnames=fieldnames)
    thewriter.writeheader()

    for lm in _Marks:
        lmx = lm[0]
        lmy = lm[1]
        lmz = lm[2]
        thewriter.writerow({'lmx': lmx,
                            'lmy': lmy,
                            'lmz': lmz, })
# save planning to csv
with open('v6_planning.csv', 'w', newline='') as csvfile:
    fieldnames = ["akx", "aky", "akz", "dk", "wkx", "wky", "wkz"]
    thewriter = csv.DictWriter(csvfile, fieldnames=fieldnames)
    thewriter.writeheader()

    _L_dk.append(0)  # to add the last waypoint
    _L_wk = np.concatenate((_L_wk, np.array([[0, 0, 0]]).T), 1)
    for k in range(len(_L_dk)):
        akx = L_ak[0, k]
        aky = L_ak[1, k]
        akz = L_ak[2, k]
        dk = _L_dk[k]
        wkx = _L_wk[0, k]
        wky = _L_wk[1, k]
        wkz = _L_wk[2, k]
        thewriter.writerow({'akx': akx,
                            'aky': aky,
                            'akz': akz,
                            'dk': dk,
                            'wkx': wkx,
                            'wky': wky,
                            'wkz': wkz})

# save state to csv
with open('v6_state.csv', 'w', newline='') as csvfile:
    fieldnames = ["t", "px", "py", "pz", "vr", "px_hat", "py_hat", "pz_hat",
                  "r11", "r12", "r13", "r21", "r22", "r23", "r31", "r32", "r33",
                  'q1', 'q2', 'q3', 'q4', 'b', 'u2', 'u3', 'u4',
                  'cov_q1', 'cov_q2', 'cov_q3', 'cov_q4',
                  'cov_l1', 'cov_l2', 'cov_l3']

    thewriter = csv.DictWriter(csvfile, fieldnames=fieldnames)
    thewriter.writeheader()

    for k in range(len(L_rec)):
        t = L_rec[k].t
        px = L_rec[k].p[0, 0]
        py = L_rec[k].p[1, 0]
        pz = L_rec[k].p[2, 0]
        px_hat = L_rec[k].px_hat
        py_hat = L_rec[k].py_hat
        pz_hat = L_rec[k].yz
        vr = L_rec[k].vr
        r11 = L_rec[k].R[0, 0]
        r12 = L_rec[k].R[0, 1]
        r13 = L_rec[k].R[0, 2]
        r21 = L_rec[k].R[1, 0]
        r22 = L_rec[k].R[1, 1]
        r23 = L_rec[k].R[1, 2]
        r31 = L_rec[k].R[2, 0]
        r32 = L_rec[k].R[2, 1]
        r33 = L_rec[k].R[2, 2]

        # quaternions
        R_hom = np.concatenate((np.concatenate((L_rec[k].R, np.zeros((3, 1))), 1), np.array([[0., 0., 0., 1.]])))
        quat = tf.quaternion_from_matrix(R_hom)
        q1 = quat[0]
        q2 = quat[1]
        q3 = quat[2]
        q4 = quat[3]

        # control signal
        b = L_rec[k].joint[0, 0]
        u2 = L_rec[k].joint[1, 0]  # top fin
        u3 = L_rec[k].joint[2, 0]  # right fin
        u4 = L_rec[k].joint[3, 0]  # left fin

        # covariance
        P = L_rec[k].P_p
        cov_l, cov_R = np.linalg.eig(P)
        cov_R_hom = np.concatenate((np.concatenate((cov_R, np.zeros((3, 1))), 1), np.array([[0., 0., 0., 1.]])))
        cov_quat = tf.quaternion_from_matrix(cov_R_hom)

        cov_q1 = cov_quat[0]
        cov_q2 = cov_quat[1]
        cov_q3 = cov_quat[2]
        cov_q4 = cov_quat[3]

        cov_l1 = cov_l[0]
        cov_l2 = cov_l[1]
        cov_l3 = cov_l[2]

        row_set = {'t': t, 'px': px, 'py': py, 'pz': pz, 'px_hat': px_hat, 'py_hat': py_hat, 'pz_hat': pz_hat, 'vr': vr,
                   'r11': r11, 'r12': r12, 'r13': r13, 'r21': r21, 'r22': r22, 'r23': r23, 'r31': r31, 'r32': r32,
                   'r33': r33,
                   'q1': q1, 'q2': q2, 'q3': q3, 'q4': q4, 'b': b, 'u2': u2, 'u3': u3, 'u4': u4,
                   'cov_q1': cov_q1, 'cov_q2': cov_q2, 'cov_q3': cov_q3, 'cov_q4': cov_q4,
                   'cov_l1': cov_l1, 'cov_l2': cov_l2, 'cov_l3': cov_l3}

        # localization
        # for i in range(_nb_box):
        #     _pos_box = _pos_boxes[k][i]
        #     X, Y, Z = _pos_box[0], _pos_box[1], _pos_box[2]
        #     row_set.update({'x_box_' + str(i): X.mid(), 'y_box_' + str(i): Y.mid(), 'z_box_' + str(i): Z.mid(),
        #                     'lx_box_' + str(i): X.diam(), 'ly_box_' + str(i): Y.diam(), 'lz_box_' + str(i): Z.diam()})

        thewriter.writerow(row_set)
print("done")
