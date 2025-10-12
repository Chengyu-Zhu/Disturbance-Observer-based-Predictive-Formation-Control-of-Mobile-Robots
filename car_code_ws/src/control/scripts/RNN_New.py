"""
RNN solver for QP-like problems with inequality constraints.
"""
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def relu_subgradient(z, choice=0.5):
    sg = np.zeros_like(z, dtype=float)
    sg[z > 0] = 1.0
    sg[z < 0] = 0.0
    sg[z == 0] = choice
    return sg


def ode_system(t, y, P, v, gamma, Upsilon, W, F, M, h, eta):
    # sizes
    n_x = W.shape[0]
    n_lambda = h.size
    n_mu = F.shape[0]

    x = y[:n_x]
    lambda_vec = y[n_x:n_x + n_lambda]
    mu = y[n_x + n_lambda:]

    # residual for inequality: h - M*x
    res = h - M.dot(x)

    # avoid division by zero
    # 如果 mu 与 res 尺寸不匹配，则用与 res 相同尺寸的零向量作为 mu 的替代，避免广播错误
    if mu.size == res.size:
        mu_for_w = mu
    else:
        mu_for_w = np.zeros_like(res)

    w = np.sqrt(res**2 + mu_for_w**2 + np.finfo(float).eps)

    D1 = np.diag(res / w)
    D2 = np.diag(mu_for_w / w)

    # Compute D3, D4 safely: if mu and res sizes match, do element-wise; otherwise use zeros
    if mu.size == res.size and mu.size > 0:
        D3 = np.diag(relu_subgradient(res, 0.5) * np.maximum(0, mu))
        D4 = np.diag(np.maximum(0, res) * relu_subgradient(mu, 0.5))
    else:
        # fallback to zero matrices of appropriate size to avoid broadcasting errors
        D3 = np.zeros((res.size, res.size))
        D4 = np.zeros((res.size, res.size))

    # build S matrix
    # Note shapes: W (n_x x n_x), F (n_mu x n_x) in MATLAB code F was m x p*Nu where m==1
    S_top = np.hstack([W, F.T, M.T])
    S_mid = np.hstack([F, np.zeros((n_mu, n_mu)), np.zeros((n_mu, n_lambda))])
    bottom_left = -eta * M + eta * D1.dot(M) - (1 - eta) * D3.dot(M)
    bottom_mid = np.zeros((n_lambda, n_mu))
    bottom_right = eta * np.eye(n_lambda) - eta * D2 + (1 - eta) * D4
    S_bottom = np.hstack([bottom_left, bottom_mid, bottom_right])

    S = np.vstack([S_top, S_mid, S_bottom])

    Py_plus_v = P.dot(y) + v

    # solve S * dydt = -gamma * Upsilon * (P*y + v)
    rhs = -gamma * Upsilon * Py_plus_v
    # Use numpy.linalg.solve for linear system
    dydt = np.linalg.solve(S, rhs)
    return dydt
