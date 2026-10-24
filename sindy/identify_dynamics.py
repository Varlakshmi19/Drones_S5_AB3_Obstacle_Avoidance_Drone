"""
SINDy system identification on the drone simulation logs.

Works on logs from BOTH controllers:
    logs/brain_s*.npz   <- python brain_avoidance.py --all --headless --log
    logs/fsm_s*.npz     <- python obstacle_avoidance.py   (LOG_DATA = True)

Four models are identified:

  A. Translational physics      [ax, ay, az] = f(roll, pitch, thrust, v)
     -> should rediscover  az = (kf/m)*sum(rpm^2)*cos(roll)cos(pitch) - g, etc.

  B. Rotational physics         [dwx, dwy, dwz] = f(rpm_i^2)
     -> motor-mixing angular acceleration coefficients of CF2X "X" frame

  C. Torque physics             [tau_x, tau_y, tau_z] = I * dw/dt = f(rpm_i^2)
     -> 3rd equation: derived by multiplying 2nd equation LHS (dw/dt) with
        inertia tensor [Ixx, Iyy, Izz] to recover aerodynamic body torques

  D. Closed-loop response       [dvx, dvy] = f(vx, vy, vcx, vcy)   (SINDYc)
     -> compact model of "command in -> drone motion out" for the whole
        PID + airframe stack. Validated by free rollout on a held-out run.

Usage
-----
    python -m sindy.identify_dynamics                 # all logs, hold out scenario 4
    python -m sindy.identify_dynamics --test 6        # hold out another scenario
    python -m sindy.identify_dynamics --logs logs/fsm_s*.npz
"""

import argparse
import glob
import json
import os

import numpy as np

from .sindy import SINDy, PolynomialLibrary

G = 9.8
# CF2X drone inertia tensor principal moments (kg*m^2) from cf2x.urdf
IXX = 1.4e-5
IYY = 1.4e-5
IZZ = 2.17e-5
INERTIA = np.array([IXX, IYY, IZZ])

# Dynamic directory resolution (works whether invoked from repo root or pybullet/)
BASE_DIR = "pybullet" if os.path.isdir("pybullet") and not os.path.isdir("logs") else "."
OUT_DIR = os.path.join(BASE_DIR, "results", "sindy")
MODEL_DIR = os.path.join(BASE_DIR, "models")


# ----------------------------------------------------------------------
# Data loading
# ----------------------------------------------------------------------
def load_log(path):
    z = np.load(path, allow_pickle=True)
    data, cols = z["data"], [str(c) for c in z["columns"]]
    d = {c: data[:, i] for i, c in enumerate(cols)}
    d["_dt"] = float(z["dt"])
    d["_name"] = os.path.basename(path)
    d["_scenario"] = int(z["scenario"])
    d["_kf"] = float(z["kf"]) if "kf" in z else 3.16e-10
    d["_km"] = float(z["km"]) if "km" in z else 7.94e-12
    d["_mass"] = float(z["mass"]) if "mass" in z else 0.027
    d["_arm"] = float(z["arm"]) if "arm" in z else 0.0397
    d["_ixx"] = float(z["ixx"]) if "ixx" in z else IXX
    d["_iyy"] = float(z["iyy"]) if "iyy" in z else IYY
    d["_izz"] = float(z["izz"]) if "izz" in z else IZZ
    return d


def forward_diff(x, dt):
    """(x[k+1]-x[k])/dt, paired with state/input at k (zero-order-hold input)."""
    return (x[1:] - x[:-1]) / dt


def stack(logs, fn):
    parts = [fn(L) for L in logs]
    return [np.vstack([p[i] for p in parts]) for i in range(len(parts[0]))]


# ----------------------------------------------------------------------
# Feature builders (each returns X, X_dot, U) with rows aligned at step k
# ----------------------------------------------------------------------
THRUST_SCALE = 1e-8   # sum(rpm^2) is ~1e8; scale so coefficients are O(1)


def translational(L):
    dt = L["_dt"]
    v = np.column_stack([L["vx"], L["vy"], L["vz"]])
    a = forward_diff(v, dt)
    T = THRUST_SCALE * (L["rpm0"]**2 + L["rpm1"]**2 + L["rpm2"]**2 + L["rpm3"]**2)
    # thrust from the rpm that was applied during step k -> k+1
    feats = np.column_stack([L["roll"], L["pitch"], T])[:-1]
    return feats, a, v[:-1]


def rotational(L):
    dt = L["_dt"]
    w = np.column_stack([L["wx"], L["wy"], L["wz"]])
    dw = forward_diff(w, dt)
    r2 = THRUST_SCALE * np.column_stack([L[f"rpm{i}"]**2 for i in range(4)])[:-1]
    return w[:-1], dw, r2


def closed_loop(L):
    dt = L["_dt"]
    v = np.column_stack([L["vx"], L["vy"]])
    dv = forward_diff(v, dt)
    u = np.column_stack([L["vcx"], L["vcy"]])[:-1]
    return v[:-1], dv, u


# ----------------------------------------------------------------------
# Fitting helpers
# ----------------------------------------------------------------------
def fit_translational(train):
    F, A, V = stack(train, translational)
    # physics-informed candidate terms on top of a linear library
    lib = PolynomialLibrary(degree=1, include_bias=True, custom=[
        (lambda n: "T cos(roll)cos(pitch)", lambda Z: Z[:, 2] * np.cos(Z[:, 0]) * np.cos(Z[:, 1])),
        (lambda n: "T sin(pitch)",          lambda Z: Z[:, 2] * np.sin(Z[:, 1])),
        (lambda n: "T sin(roll)",           lambda Z: Z[:, 2] * np.sin(Z[:, 0])),
        (lambda n: "roll pitch",            lambda Z: Z[:, 0] * Z[:, 1]),
        (lambda n: "T roll pitch",          lambda Z: Z[:, 2] * Z[:, 0] * Z[:, 1]),
    ])
    # "states" here are the accelerations' regressors; we fit a = Theta([roll,pitch,T,vx,vy,vz]) Xi
    m = SINDy(lib, threshold=0.02, alpha=1e-6)
    m.fit(np.hstack([F, V]), A, state_names=["roll", "pitch", "T", "vx", "vy", "vz"])
    m.state_names = ["vx", "vy", "vz"]          # targets are dv/dt
    return m, np.hstack([F, V]), A


def fit_rotational(train):
    W, dW, R2 = stack(train, rotational)
    lib = PolynomialLibrary(degree=1, include_bias=True)
    m = SINDy(lib, threshold=0.05, alpha=1e-6)
    m.fit(np.hstack([W, R2]), dW, state_names=["wx", "wy", "wz", "r0", "r1", "r2", "r3"])
    m.state_names = ["wx", "wy", "wz"]
    return m, np.hstack([W, R2]), dW


def compute_torque_model(mB, inertia=INERTIA):
    """
    Multiply the 2nd equation (rotational dynamics dw/dt = f(w, rpm_i^2)) LHS and
    coefficients by the inertia tensor I = diag(Ixx, Iyy, Izz) to yield aerodynamic
    body torques tau = I * dw/dt = f(rpm_i^2).
    """
    lib = PolynomialLibrary(degree=1, include_bias=True)
    mTau = SINDy(lib, threshold=0.0, alpha=0.0)
    mTau.library.fit(["wx", "wy", "wz", "r0", "r1", "r2", "r3"])
    mTau.state_names = ["tau_x", "tau_y", "tau_z"]
    mTau.n_x = 3
    mTau.n_u = 0
    # Multiplying LHS (dw/dt) by inertia scales each column j of Xi by inertia[j]
    mTau.Xi = mB.Xi * inertia
    return mTau


def fit_closed_loop(train):
    V, dV, U = stack(train, closed_loop)
    lib = PolynomialLibrary(degree=2, include_bias=True)
    m = SINDy(lib, threshold=0.04, alpha=1e-5)
    m.fit(V, dV, U, state_names=["vx", "vy"], input_names=["vcx", "vcy"])
    return m, V, dV, U


def r2(y, yhat):
    ss_res = ((y - yhat) ** 2).sum(axis=0)
    ss_tot = ((y - y.mean(axis=0)) ** 2).sum(axis=0)
    return 1.0 - ss_res / np.maximum(ss_tot, 1e-12)


def regressor_predict(model, Z):
    return model.library.transform(Z) @ model.Xi


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", default="logs/*.npz")
    ap.add_argument("--test", type=int, default=4, help="scenario held out for validation")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    paths = sorted(glob.glob(args.logs))
    if not paths and args.logs == "logs/*.npz" and os.path.isdir("pybullet/logs"):
        paths = sorted(glob.glob("pybullet/logs/*.npz"))
    if not paths:
        raise SystemExit(f"No logs match {args.logs}. Run:  python brain_avoidance.py --all --headless --log")
    logs = [load_log(pth) for pth in paths]
    train = [L for L in logs if L["_scenario"] != args.test]
    test = [L for L in logs if L["_scenario"] == args.test]
    if not test:
        test, train = logs[-1:], logs[:-1] or logs
    print(f"Train: {[L['_name'] for L in train]}")
    print(f"Test : {[L['_name'] for L in test]}")

    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(MODEL_DIR, exist_ok=True)
    report = {}

    # ---------------- A: translational physics ------------------------
    print("\n=== A. Translational physics  (a = f(attitude, thrust, v)) ===")
    mA, _, _ = fit_translational(train)
    mA.print_equations(4)
    ZA, AA, VA = stack(test, translational)
    r2A = r2(AA, regressor_predict(mA, np.hstack([ZA, VA])))
    print("held-out R^2 [ax, ay, az]:", np.round(r2A, 4))

    kf, mass = train[0]["_kf"], train[0]["_mass"]
    if kf and mass:
        print(f"URDF ground truth: kf/m = {kf / mass / THRUST_SCALE:.4f} (x1e-8 per rpm^2), g = {G}")
    report["A_translational"] = dict(equations=mA.equations(), heldout_r2=r2A.tolist(),
                                     kf_over_m_true=(kf / mass / THRUST_SCALE) if kf and mass else None)

    # ---------------- B: rotational physics ---------------------------
    print("\n=== B. Rotational physics  (dw = f(w, rpm_i^2)) ===")
    mB, _, _ = fit_rotational(train)
    mB.print_equations(4)
    WB, dWB, RB = stack(test, rotational)
    r2B = r2(dWB, regressor_predict(mB, np.hstack([WB, RB])))
    print("held-out R^2 [dwx, dwy, dwz]:", np.round(r2B, 4))
    report["B_rotational"] = dict(equations=mB.equations(), heldout_r2=r2B.tolist())

    # ---------------- C: torque physics (3rd equation) ----------------
    print("\n=== C. Torque physics  (tau = I * dw/dt) [3rd Equation] ===")
    print(f"Multiplying 2nd equation LHS by inertia I = [{IXX:.2e}, {IYY:.2e}, {IZZ:.2e}] kg*m^2:")
    mTau = compute_torque_model(mB, INERTIA)
    torque_lhs = ["tau_x (Ixx*dwx/dt)", "tau_y (Iyy*dwy/dt)", "tau_z (Izz*dwz/dt)"]
    mTau.print_equations(4, lhs_format=torque_lhs)
    tau_test_true = dWB * INERTIA
    tau_test_pred = regressor_predict(mTau, np.hstack([WB, RB]))
    r2Tau = r2(tau_test_true, tau_test_pred)
    print("held-out R^2 [tau_x, tau_y, tau_z]:", np.round(r2Tau, 4))

    # Analytical URDF comparison for CF2X
    arm = train[0].get("_arm", 0.0397)
    lx = arm * np.sqrt(2) / 2.0
    kf_gt = train[0].get("_kf", 3.16e-10)
    km_gt = train[0].get("_km", 7.94e-12)
    tau_roll_pitch_gt = lx * kf_gt / THRUST_SCALE
    tau_yaw_gt = km_gt / THRUST_SCALE
    print(f"URDF ground truth: roll/pitch L*kf = {tau_roll_pitch_gt:.4e} N*m (per scaled rpm^2), yaw km = {tau_yaw_gt:.4e} N*m")
    report["C_torque"] = dict(
        equations=mTau.equations(4, lhs_format=torque_lhs),
        heldout_r2=r2Tau.tolist(),
        inertia_kg_m2=INERTIA.tolist(),
        urdf_ground_truth=dict(tau_roll_pitch_scale=tau_roll_pitch_gt, tau_yaw_scale=tau_yaw_gt)
    )

    # ---------------- D: closed-loop SINDYc ---------------------------
    print("\n=== D. Closed-loop response  (dv = f(v, v_cmd))  [SINDYc] ===")
    mC, _, _, _ = fit_closed_loop(train)
    mC.print_equations(4)
    VC, dVC, UC = stack(test, closed_loop)
    r2C = mC.score(VC, dVC, UC)
    print("held-out derivative R^2 [dvx, dvy]:", np.round(r2C, 4))

    rollouts = []
    for L in test:
        V, _, U = closed_loop(L)
        Vsim = mC.simulate(V[0], L["_dt"], U=U)
        pos_true = np.column_stack([L["x"], L["y"]])[:-1]
        pos_sim = pos_true[0] + np.cumsum(Vsim, axis=0) * L["_dt"]
        vel_rmse = float(np.sqrt(((Vsim - V) ** 2).mean()))
        pos_err_end = float(np.linalg.norm(pos_sim[-1] - pos_true[-1]))
        path_len = float(np.linalg.norm(np.diff(pos_true, axis=0), axis=1).sum())
        print(f"rollout {L['_name']}: velocity RMSE = {vel_rmse:.4f} m/s | "
              f"end position error = {pos_err_end:.3f} m over {path_len:.2f} m of flight")
        rollouts.append(dict(log=L["_name"], L=L, V=V, Vsim=Vsim, pos_true=pos_true, pos_sim=pos_sim,
                             vel_rmse=vel_rmse, end_err=pos_err_end, path_len=path_len))
    report["D_closed_loop"] = dict(
        equations=mC.equations(), heldout_derivative_r2=r2C.tolist(),
        rollouts=[{k: v for k, v in r.items() if k in ("log", "vel_rmse", "end_err", "path_len")} for r in rollouts])
    report["C_closed_loop"] = report["D_closed_loop"]

    # ---------------- save ---------------------------------------------
    for tag, m, lhs_fmt in (
        ("translational", mA, None),
        ("rotational", mB, None),
        ("torque", mTau, ["tau_x", "tau_y", "tau_z"]),
        ("closed_loop", mC, None),
    ):
        with open(os.path.join(MODEL_DIR, f"sindy_{tag}.json"), "w") as f:
            json.dump(dict(terms=m.library.names_, targets=m.state_names,
                           Xi=m.Xi.tolist(), equations=m.equations(4, lhs_format=lhs_fmt)), f, indent=2)
    with open(os.path.join(OUT_DIR, "report.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(f"\nSaved models to {MODEL_DIR}/ and report to {OUT_DIR}/report.json")

    if not args.no_plots:
        make_plots(mA, mB, mTau, test, rollouts)


def make_plots(mA, mB, mTau, test, rollouts):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    L = test[0]
    t = L["t"][:-1]

    # A: accelerations
    Z, A, V = translational(L)
    Ahat = regressor_predict(mA, np.hstack([Z, V]))
    fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for i, lab in enumerate(["a_x", "a_y", "a_z"]):
        ax[i].plot(t, A[:, i], label="simulation", lw=1.6)
        ax[i].plot(t, Ahat[:, i], "--", label="SINDy", lw=1.2)
        ax[i].set_ylabel(f"{lab} (m/s²)")
    ax[0].legend(loc="upper right")
    ax[-1].set_xlabel("time (s)")
    fig.suptitle(f"A. Translational dynamics — held-out {L['_name']}")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "A_translational.png"), dpi=130)
    plt.close(fig)

    # B: angular accelerations
    W, dW, R = rotational(L)
    dWhat = regressor_predict(mB, np.hstack([W, R]))
    fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for i, lab in enumerate(["dω_x", "dω_y", "dω_z"]):
        ax[i].plot(t, dW[:, i], label="simulation", lw=1.6)
        ax[i].plot(t, dWhat[:, i], "--", label="SINDy", lw=1.2)
        ax[i].set_ylabel(f"{lab} (rad/s²)")
    ax[0].legend(loc="upper right")
    ax[-1].set_xlabel("time (s)")
    fig.suptitle(f"B. Rotational dynamics — held-out {L['_name']}")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "B_rotational.png"), dpi=130)
    plt.close(fig)

    # C: torques (tau = I * dw/dt)
    tau_true = dW * INERTIA
    tau_hat = regressor_predict(mTau, np.hstack([W, R]))
    fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    tau_labs = [r"\tau_x\ (I_{xx}\dot{\omega}_x)", r"\tau_y\ (I_{yy}\dot{\omega}_y)", r"\tau_z\ (I_{zz}\dot{\omega}_z)"]
    for i, lab in enumerate(tau_labs):
        ax[i].plot(t, tau_true[:, i], label=r"simulation ($I \cdot \dot{\omega}$)", lw=1.6)
        ax[i].plot(t, tau_hat[:, i], "--", label="SINDy torque", lw=1.2)
        ax[i].set_ylabel(f"${lab}$ (N·m)")
    ax[0].legend(loc="upper right")
    ax[-1].set_xlabel("time (s)")
    fig.suptitle(f"C. Torque dynamics (tau = I * dw/dt) — held-out {L['_name']}")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "C_torque.png"), dpi=130)
    plt.close(fig)

    # D: rollout
    for r in rollouts:
        fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
        tt = r["L"]["t"][:-1]
        ax[0].plot(tt, r["V"][:, 0], label="vx sim")
        ax[0].plot(tt, r["Vsim"][:, 0], "--", label="vx SINDy")
        ax[0].plot(tt, r["V"][:, 1], label="vy sim")
        ax[0].plot(tt, r["Vsim"][:, 1], "--", label="vy SINDy")
        ax[0].set_xlabel("time (s)")
        ax[0].set_ylabel("velocity (m/s)")
        ax[0].legend()
        ax[0].set_title("Free rollout from commands only")
        ax[1].plot(r["pos_true"][:, 0], r["pos_true"][:, 1], label="simulation")
        ax[1].plot(r["pos_sim"][:, 0], r["pos_sim"][:, 1], "--", label="SINDy rollout")
        ax[1].set_aspect("equal")
        ax[1].set_xlabel("x (m)")
        ax[1].set_ylabel("y (m)")
        ax[1].legend()
        ax[1].set_title(f"Path — end error {r['end_err']:.3f} m")
        fig.suptitle(f"D. Closed-loop SINDYc model — held-out {r['log']}")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, f"D_rollout_{r['log'].replace('.npz', '')}.png"), dpi=130)
        fig.savefig(os.path.join(OUT_DIR, f"C_rollout_{r['log'].replace('.npz', '')}.png"), dpi=130)
        plt.close(fig)
    print(f"Plots written to {OUT_DIR}/")


if __name__ == "__main__":
    main()
