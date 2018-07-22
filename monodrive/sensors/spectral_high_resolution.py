
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import numpy as np
from numpy import linalg as lg
import pyfftw

import time
import numpy.fft as fftpack


def compute_covariance(X):
    N = X.shape[1]
    R = (1. / N) * X * X.H

    return R


def compute_autocovariance(x, M):
    N = x.shape[0]

    x_vect = np.transpose(np.matrix(x))

    yn = x_vect[M - 1::-1]
    R = yn * yn.H
    for indice in range(1, N - M):
        yn = x_vect[M - 1 + indice:indice - 1:-1]
        R = R + yn * yn.H

    R = R / N
    return R


def modified_correlation(x, M):
    N = x.shape[0]

    x_vect = np.transpose(np.matrix(x))

    yn = x_vect[M - 1::-1]
    zn = np.conj(x_vect[0:M])
    R = yn * yn.H
    R2 = zn*zn.H
    for indice in range(1, N - M):
        yn = x_vect[M - 1 + indice:indice - 1:-1]
        zn = np.conj(x_vect[indice: M + indice])
        R = R + yn * yn.H
        R2 = R2 +zn*zn.H

    R = (R+R2) / (2.*N)
    return R

def root_MUSIC_One(x, L, M, Fe):
    N = x.shape[0]
    if M == None:
        M = 16

    R = compute_autocovariance(x, M)
    U, S, V = lg.svd(R)
    G = U[:, 3:]

    P = G * G.H

    Q = 0j * np.zeros(2 * M - 1)

    for (idx, val) in enumerate(range(M - 1, -M, -1)):
        diag = np.diag(P, val)
        Q[idx] = np.sum(diag)

    roots = np.roots(Q)

    roots = np.extract(np.abs(roots) < 1, roots)
    roots = np.extract(np.imag(roots) != 0, roots)

    distance_from_circle = np.abs(np.abs(roots) - 1)
    index_sort = np.argsort(distance_from_circle)
    component_roots = roots[index_sort[:L]]

    angle = -np.angle(component_roots)

    f = Fe * angle / (2. * np.pi)

    return f


def Esprit(x, L, M, Fe):
    N = x.shape[0]

    if M == None:
        M = N // 2

    R = compute_autocovariance(x, M)
    U, S, V = lg.svd(R)
    S = U[:, :3]

    S1 = S[:-1, :]

    S2 = S[1:, :]

    Phi = (S1.H * S1).I * S1.H * S2

    V, U = lg.eig(Phi)

    angle = -np.angle(V)

    f = Fe * angle / (2. * np.pi)

    return f


def range_by_fft(z1, Wx, NN):
    Hz = z1 * Wx  # 1375x64 2D-array Hann windowed dechirped samples (fast/slow plan)
    Z = pyfftw.interfaces.numpy_fft.fft(Hz, NN, 0)  # 1024 points FFT performed on the 64 1D-arrays
    ZA = abs(Z)  # 1024x64 2D-array with amplitudes
    x = ZA.sum(axis=1)/64  # 1024 points 1D-Array, summing up over sweeps in order to reduce noise effect and clean up the spectrum
    Lgx = x.size
    # Following is CFAR algorithm
    # we used CFAR order statistics : OSCFAR (refer to the report by Celite on Radar design, CFAR section)
    G = 2  # Guard interval
    Ncfar = 10  # averaging window size
    Thr = 3  # threshold depending on false alarm detection probability
    y = x * x  # compute energy x[k]^2
    p = []  # initialization of peaks array
    p = np.array(p)
    qy = 0 * y  # initialization of the value of the peaks

    # compute CFAR for the first samples of the block (right neighbours)
    for k in range(0, 2 * (G + Ncfar) - 1):
        z = y[k + 1:k + 1 + Ncfar - 1]
        T = np.median(z)
        if (y[k] >  T):
            p = np.hstack((p, [k]))
            qy[k] = y[k]

    # compute CFAR for the following block (right and left neighbours)
    for k in range(2 * (G + Ncfar) - 1, (Lgx - G - Ncfar - 1)):
        z = np.concatenate((y[k + G:k + G + Ncfar - 1], y[k - G:-1:k - G - Ncfar + 1]), axis=0)
        T = np.median(z)
        if (y[k] > Thr * T):
            p = np.hstack((p, [k]))
            qy[k] = y[k]

    # compute CFAR for the last samples of the block (left neighbours)
    for k in range(2 * (G + Ncfar) - 1, Lgx - 1):
        z = y[k - G:-1:k - G + Ncfar + 1]
        T = np.median(z)
        if (y[k] > Thr * T):
            p = np.hstack((p, [k]))
            qy[k] = y[k]
    # peaks localization
    DenoisingThreshold = 1 / 5000
    Lgy = qy.size
    k = 1
    p = []
    q = []
    p = np.array(p)
    q = np.array(q)
    mm = max(qy) * DenoisingThreshold
    RCS_Th = 2
    if (qy[0] > mm and qy[0] > qy[1]):
        p += [0]
        q += [qy[0]]
    for k in range(0, Lgy - 2):
        if (qy[k] > 0):
            RCS_k = 10 * np.log10(qy[k] * ((k+1) ** 2) * (4 * np.pi) ** 3 / NN ** 2) - 25
        else:
            RCS_k = 1
        if (qy[k] > qy[k - 1] and qy[k] > qy[k + 1] and qy[k]> mm):
            p = np.hstack((p, [k]))
            q = np.hstack((q, [qy[k]]))
    return [p, q]


def ML_AoA_Estimation(project):
        N = project.shape[0]
        # M = project.shape[1]

        Theta = np.arange(-25,25,0.1)
        Theta_rd = Theta / 180. * np.pi
        P = len(Theta);
        V = np.zeros((N, P))+np.zeros((N, P))*1j
        for p in range(P):
            for n in range(N):
                V[n, p] = np.exp(-1j * np.pi * n  * np.sin(Theta_rd[p]))
        Theta_Est = 0. #np.zeros(1, M)

        ML = np.zeros(P)
        for p in range(P):
            A = V[:, p]
            cte = np.dot(np.conj(np.transpose(A)), np.transpose(project))
            ML[p] = np.abs(cte)

            index = ML.argmax()
            Theta_Est = Theta[index]

        return Theta_Est

def NumberOfTargetsAIC(x, M):
    N = x.shape[0]
    R = modified_correlation(x,M)
    U, S, V = lg.svd(R)
    S_vec = S.reshape(M, 1)
    S_vec = np.sqrt(S_vec)
    S_vec = S_vec / S_vec.max()
    J = np.zeros(M - 2)
    for d in range(0, M - 2):
        cte1 = 1
        cte0 = 0
        for i in range(d + 1, M):
            cte1 = cte1 * S_vec[i]
            cte0 = cte0 + S_vec[i]
        cte1 = cte1 ** (1 / (M - d))
        cte0 = cte0 / (M - d)
        J[d] = abs((M/2* np.log((cte1 / cte0) ** (M - d))) + d * (2 * M - d) / 2)   #-(1.0 * np.log((cte1 / cte0) ** (M - d))) + d * (2 * M - d) / 2  #
    mn = np.argmin(J)
    toto = S_vec[0:mn + 1]
    # toto = S_vec[np.nonzero(S_vec > Thrshld)[0]]
    L1 = mn+1 #toto.size
    k = 0
    mn2 = 0
    p = 1
    while (k < M) & (p == 1):
        if S_vec[k] > 1e-5:
            k = k + 1
            mn2 = mn2 + 1
        else:
            p = 0

    # L = min(L1, mn2)
    L = min(L1,2)
    return L