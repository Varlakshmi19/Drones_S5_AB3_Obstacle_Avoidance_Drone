"""
Minimal, dependency-light SINDy / SINDYc (Brunton, Proctor & Kutz, PNAS 2016).

    dX/dt  ~=  Theta(X, U) @ Xi          Xi sparse

Theta : library of candidate functions (polynomials + optional custom terms)
Xi    : found with Sequentially Thresholded Least Squares (STLSQ)

Only numpy (+ scipy for Savitzky-Golay smoothing) is required, so it runs
anywhere the simulation runs. Results can be cross-checked against pysindy.
"""

import math
from itertools import combinations_with_replacement
import numpy as np


# ----------------------------------------------------------------------
# Derivatives (Pure NumPy with Savitzky-Golay Polynomial Differentiation)
# ----------------------------------------------------------------------
def _savgol_coeffs(window_length, polyorder, deriv=1, delta=1.0):
    half_window = (window_length - 1) // 2
    x = np.arange(-half_window, half_window + 1, dtype=float)
    order = np.arange(polyorder + 1).reshape(-1, 1)
    A = x ** order
    pinvA = np.linalg.pinv(A)
    return pinvA[deriv] * (math.factorial(deriv) / (delta ** deriv))


def finite_difference(X, dt, smooth_window=0, polyorder=3):
    """
    Time derivative of X (n_samples x n_features).
    Uses pure-NumPy Savitzky-Golay polynomial differentiation if smooth_window > 0.
    """
    X = np.asarray(X, dtype=float)
    if smooth_window and len(X) > smooth_window and smooth_window > polyorder:
        w = smooth_window if smooth_window % 2 == 1 else smooth_window + 1
        coeffs = _savgol_coeffs(w, polyorder, deriv=1, delta=dt)
        half_w = (w - 1) // 2
        dX = np.zeros_like(X)
        for col in range(X.shape[1]):
            dX[:, col] = np.convolve(X[:, col], coeffs[::-1], mode="same")
        dX[:half_w, :] = np.gradient(X[:half_w + 1, :], dt, axis=0)[:half_w, :]
        dX[-half_w:, :] = np.gradient(X[-half_w - 1:, :], dt, axis=0)[-half_w:, :]
        return dX
    return np.gradient(X, dt, axis=0)


# ----------------------------------------------------------------------
# Candidate function library
# ----------------------------------------------------------------------
class PolynomialLibrary:
    """
    All monomials of the input features up to `degree`, plus optional
    custom functions f(Z) -> column and their names.
    """

    def __init__(self, degree=2, include_bias=True, custom=None):
        self.degree = degree
        self.include_bias = include_bias
        self.custom = custom or []          # list of (name_fn(names), fn(Z))
        self.names_ = None
        self._combos = None

    def fit(self, names):
        n = len(names)
        combos = []
        if self.include_bias:
            combos.append(())
        for d in range(1, self.degree + 1):
            combos.extend(combinations_with_replacement(range(n), d))
        self._combos = combos
        self.input_names_ = list(names)
        term_names = []
        for c in combos:
            if not c:
                term_names.append("1")
            else:
                parts = []
                for idx in sorted(set(c)):
                    k = c.count(idx)
                    parts.append(names[idx] if k == 1 else f"{names[idx]}^{k}")
                term_names.append(" ".join(parts))
        for name_fn, _ in self.custom:
            term_names.append(name_fn(names))
        self.names_ = term_names
        return self

    def transform(self, Z):
        Z = np.atleast_2d(np.asarray(Z, dtype=float))
        cols = []
        for c in self._combos:
            if not c:
                cols.append(np.ones(len(Z)))
            else:
                cols.append(np.prod(Z[:, list(c)], axis=1))
        for _, fn in self.custom:
            cols.append(fn(Z))
        return np.column_stack(cols)


# ----------------------------------------------------------------------
# Sparse regression
# ----------------------------------------------------------------------
def stlsq(Theta, dX, threshold=0.05, alpha=1e-5, max_iter=25):
    """
    Sequentially Thresholded (ridge) Least Squares.
    Columns of Theta are normalised internally so one threshold works
    across terms of very different magnitude.
    """
    norms = np.linalg.norm(Theta, axis=0)
    norms[norms == 0] = 1.0
    T = Theta / norms
    n_terms, n_targets = T.shape[1], dX.shape[1]
    Xi = np.zeros((n_terms, n_targets))

    def ridge(A, b):
        return np.linalg.solve(A.T @ A + alpha * np.eye(A.shape[1]), A.T @ b)

    for j in range(n_targets):
        active = np.ones(n_terms, dtype=bool)
        coef = ridge(T, dX[:, j])
        for _ in range(max_iter):
            # threshold relative to the largest coefficient (scale-free)
            small = np.abs(coef) < threshold * np.abs(coef).max()
            new_active = active.copy()
            new_active[np.where(active)[0][small]] = False
            if new_active.sum() == 0 or np.array_equal(new_active, active):
                break
            active = new_active
            coef = ridge(T[:, active], dX[:, j])
        full = np.zeros(n_terms)
        full[active] = coef
        Xi[:, j] = full
    return Xi / norms[:, None]


# ----------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------
class SINDy:
    """
    x_dot = Theta([x, u]) @ Xi

    state_names  : names of the state variables (targets are their derivatives)
    input_names  : names of exogenous inputs (SINDYc); may be empty
    """

    def __init__(self, library=None, threshold=0.05, alpha=1e-5):
        self.library = library or PolynomialLibrary(degree=2)
        self.threshold = threshold
        self.alpha = alpha

    # ------------------------------------------------------------------
    def fit(self, X, X_dot, U=None, state_names=None, input_names=None):
        X = np.atleast_2d(X)
        self.n_x = X.shape[1]
        U = np.zeros((len(X), 0)) if U is None else np.atleast_2d(U)
        if U.shape[0] != len(X):
            U = U.T
        self.n_u = U.shape[1]
        self.state_names = state_names or [f"x{i}" for i in range(self.n_x)]
        self.input_names = input_names or [f"u{i}" for i in range(self.n_u)]
        self.library.fit(self.state_names + self.input_names)
        Theta = self.library.transform(np.hstack([X, U]))
        self.Xi = stlsq(Theta, np.asarray(X_dot), self.threshold, self.alpha)
        return self

    def predict(self, X, U=None):
        X = np.atleast_2d(X)
        U = np.zeros((len(X), 0)) if U is None else np.atleast_2d(U)
        return self.library.transform(np.hstack([X, U])) @ self.Xi

    def score(self, X, X_dot, U=None):
        """R^2 per target on derivative prediction."""
        pred = self.predict(X, U)
        ss_res = ((X_dot - pred) ** 2).sum(axis=0)
        ss_tot = ((X_dot - X_dot.mean(axis=0)) ** 2).sum(axis=0)
        return 1.0 - ss_res / np.maximum(ss_tot, 1e-12)

    def simulate(self, x0, dt, n_steps=None, U=None):
        """
        Forward-simulate with RK4; inputs are held constant over each step (ZOH).
        """
        if U is not None:
            U = np.atleast_2d(U)
            n_steps = len(U)
        X = np.zeros((n_steps, self.n_x))
        X[0] = x0
        for k in range(n_steps - 1):
            u = U[k:k + 1] if U is not None else None
            f = lambda x: self.predict(x[None, :], u)[0]
            x = X[k]
            k1 = f(x)
            k2 = f(x + 0.5 * dt * k1)
            k3 = f(x + 0.5 * dt * k2)
            k4 = f(x + dt * k3)
            X[k + 1] = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        return X

    # ------------------------------------------------------------------
    def equations(self, precision=4):
        out = []
        for j, name in enumerate(self.state_names):
            terms = [f"{c:+.{precision}g} {t}" if t != "1" else f"{c:+.{precision}g}"
                     for c, t in zip(self.Xi[:, j], self.library.names_) if c != 0]
            rhs = " ".join(terms) if terms else "0"
            out.append(f"d({name})/dt = {rhs}")
        return out

    def print_equations(self, precision=4):
        for line in self.equations(precision):
            print(line)

    def n_active_terms(self):
        return int((self.Xi != 0).sum())

    def to_dict(self):
        return dict(Xi=self.Xi, terms=self.library.names_,
                    states=self.state_names, inputs=self.input_names)
