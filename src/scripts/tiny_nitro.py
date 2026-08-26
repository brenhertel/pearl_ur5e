#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import h5py
from mpl_toolkits import mplot3d


NUM_JOINTS = 6

def read_data(fname):
    hf = h5py.File(fname, 'r')
    print(list(hf.keys()))
    js = hf.get('joint_state_info')
    joint_time = np.array(js.get('joint_time'))
    joint_pos = np.array(js.get('joint_positions'))
    joint_vel = np.array(js.get('joint_velocities'))
    joint_eff = np.array(js.get('joint_effort'))
    joint_data = [joint_time, joint_pos, joint_vel, joint_eff]
    
    tf = hf.get('transform_info')
    tf_time = np.array(tf.get('transform_time'))
    tf_pos = np.array(tf.get('transform_positions'))
    tf_rot = np.array(tf.get('transform_orientations'))
    tf_data = [tf_time, tf_pos, tf_rot]
    print(tf_pos)
    
    wr = hf.get('wrench_info')
    wrench_time = np.array(wr.get('wrench_time'))
    wrench_frc = np.array(wr.get('wrench_force'))
    wrench_trq = np.array(wr.get('wrench_torque'))
    wrench_data = [wrench_time, wrench_frc, wrench_trq]
    
    hf.close()
    
    return joint_data, tf_data, wrench_data


def load_demo_q(path: str) -> np.ndarray:
    #q = np.load(path)
    #if q.ndim != 2:
    #    raise ValueError(f"Expected demo q with shape (T, nj), got {q.shape}")
    joint_data, tf_data, wrench_data = read_data(path)
    q = joint_data[1]
    return q.astype(float)


def resample_demo_to_T(q_demo: np.ndarray, T: int) -> np.ndarray:
    T_demo, nj = q_demo.shape
    if T_demo == T:
        return q_demo.copy()

    s_old = np.linspace(0.0, 1.0, T_demo)
    s_new = np.linspace(0.0, 1.0, T)

    q_new = np.zeros((T, nj), dtype=float)
    for j in range(nj):
        q_new[:, j] = np.interp(s_new, s_old, q_demo[:, j])
    return q_new


def build_start_end_via_constraints(q_demo_T: np.ndarray, T: int, k_via: int):
    T_demo, nj = q_demo_T.shape
    assert T_demo == T

    n = nj * T
    A = np.zeros((3 * nj, n))
    b = np.zeros(3 * nj)

    for j in range(nj):
        # Start
        A[3 * j + 0, j * T + 0] = 1.0
        b[3 * j + 0] = q_demo_T[0, j]

        # Via
        A[3 * j + 1, j * T + k_via] = 1.0
        b[3 * j + 1] = q_demo_T[k_via, j]

        # End
        A[3 * j + 2, j * T + (T - 1)] = 1.0
        b[3 * j + 2] = q_demo_T[-1, j]

    return A, b


def map_raw_index_to_resampled(k_raw: int, T_raw: int, T: int) -> int:
    if T_raw <= 1 or T <= 1:
        return 0
    k = int(round(k_raw * (T - 1) / (T_raw - 1)))
    return max(0, min(T - 1, k))


def finite_difference_matrix(T, order=1):
    """
    Return a (T-order) x T finite difference matrix of given order.
    For example, order=1 gives first differences: y[1]-y[0], y[2]-y[1], ...
    order=2 gives second differences.
    """
    if order < 0 or order >= T:
        raise ValueError("order must be between 0 and T-1")
    # Start with identity for order 0
    D = np.eye(T)
    for _ in range(order):
        D = np.diff(D, axis=0)  # first difference operator composition
    return D  # shape (T-order, T)


def block_diag(*arrs):
    """Simple block diagonal like scipy.linalg.block_diag"""
    shapes = np.array([a.shape for a in arrs])
    out = np.zeros(shapes.sum(axis=0), dtype=arrs[0].dtype)
    r, c = 0, 0
    for a in arrs:
        rr, cc = a.shape
        out[r : r + rr, c : c + cc] = a
        r += rr
        c += cc
    return out


def solve_kkt(
    P: np.ndarray, q: np.ndarray, A: np.ndarray, b: np.ndarray, reg=1e-9, verbose=False
): #-> tuple[np.ndarray, np.ndarray]:
    """Solve QP with equality constraints as a KKT system
    minimize (1/2) x^T P x + q^T x
    s.t.    Ax = b
    KKT system:
    [ P   A.T ] [x ]= [ q ]
    [ A    0  ] [nu]  [ b ]
    """
    P = np.asarray(P, dtype=float)
    q = np.asarray(q, dtype=float).reshape(-1)
    A = np.atleast_2d(np.asarray(A, dtype=float))
    b = np.asarray(b, dtype=float).reshape(-1)

    n = P.shape[0]
    m = A.shape[0]

    # if A is None and b is None:
    #    try:
    #        sol = np.linalg.solve(P, -q)
    #    except np.linalg.LinAlgError:
    #        P = P + reg * np.eye(n)
    #        sol = np.linalg.solve(P, -q)
    #    x = sol
    #    nu = None
    #    if verbose:
    #        print(f"optimal point: {x}")
    #        print(f"Lagrange constant: {nu}")
    #    return x, nu

    KKT = np.zeros((n + m, n + m), dtype=float)
    KKT[:n, :n] = P
    KKT[:n, n:] = A.T
    KKT[n:, :n] = A
    rhs = np.zeros(n + m, dtype=float)
    rhs[:n] = -q
    rhs[n:] = b
    try:
        sol = np.linalg.solve(KKT, rhs)
    except np.linalg.LinAlgError:
        eps = max(reg, 1e-12)
        KKT[:n, :n] += eps * np.eye(n)
        KKT[n:, n:] += eps * np.eye(m)
        sol = np.linalg.solve(KKT, rhs)
    x = sol[:n]
    nu = sol[n:]
    if verbose:
        print(f"optimal point: {x}")
        print(f"Lagrange constant: {nu}")
        print(f"||Px + q + A^T nu|| = {np.linalg.norm(P @ x + q + A.T @ nu):.3e}")
        print(f"||Ax - b|| = {np.linalg.norm(A @ x - b):.3e}")
    return x, nu


def assemble_qp(L, Q, r, lam, R, x_demo):
    """
    Assembling the objective and constraints as P, q, A, b for Quadratic Program
    objective = (1/2) * x.T P x + q^T x
    constraints = Ax == b
    """

    # Validate shapes and convert to numpy arrays
    n = L[0].shape[1] if len(L) > 0 else R.shape[0]
    Pbase = np.zeros((n, n))
    linear = np.zeros(n)
    const = 0.0

    # form the Normative terms
    for Lj, Qj, rj in zip(L, Q, r):
        # Ensure shapes
        Lj = np.atleast_2d(Lj)
        Qj = np.atleast_2d(Qj)
        rj = np.atleast_1d(rj)
        # (L y - r)^T Q (L y - r) = y^T (L^T Q L) y - 2 (L^T Q r)^T y + r^T Q r
        Pbase += Lj.T.dot(Qj).dot(Lj)
        linear += -(
            Lj.T.dot(Qj).dot(rj)
        )  # NOTE: very important to note that q term is negative
        const += float(rj.T.dot(Qj).dot(rj))

    # form Imitation term: lambda * (y - y_demo)^T R (y - y_demo)
    Pbase += lam * R
    linear += -lam * R.dot(x_demo)
    const += float(lam * x_demo.T.dot(R).dot(x_demo))

    # From expansion the gradient was 2*Pbase*y + 2*linear, so set H = 2*Hbase and q = 2*linear
    P = 2.0 * (0.5 * (Pbase.T + Pbase))
    q = 2.0 * linear  # in linear we stored -(...), so q will be negative of typical b
    return P, q, const


def solve_normative_or_mixed(
    q_demo: np.ndarray,
    T: int,
    order: int,
    lam: float, #| None,
    solver: str = "kkt",
    verbose: bool = True,
    k_raw: int = 2000,
    do_stack_check: bool = False,
    lam_scale: float = 1.0,
):
    """
    Usage:
      - order=2 => min-acc
      - order=3 => min-jerk
      - lam=0   => purely normative
      - lam>0   => mixed normative-imitative
      - lam=None => use lam = lam_scale * lam_ref
    """
    q_demo_T = resample_demo_to_T(q_demo, T)
    T_demo, nj = q_demo_T.shape
    assert T_demo == T

    Dk = finite_difference_matrix(T, order=order)
    Lk = block_diag(*([Dk] * nj))
    m = Lk.shape[0]

    L = [Lk]
    Q = [np.eye(m)]
    r = [np.zeros(m)]

    n = nj * T

    ranges = np.max(q_demo_T, axis=0) - np.min(q_demo_T, axis=0)

    static_thresh = 1e-2
    w = np.ones_like(ranges)
    w[ranges < static_thresh] = 0.0

    range_floor = 0.2
    eff = np.maximum(ranges, range_floor)
    w[ranges >= static_thresh] = 1.0 / (eff[ranges >= static_thresh] ** 2)

    R_blocks = [w[j] * np.eye(T) for j in range(nj)]
    R = block_diag(*R_blocks)

    x_demo = q_demo_T.reshape(-1, order="F")

    Pnorm = Lk.T @ Lk
    d_norm = float(np.mean(np.diag(Pnorm)))
    d_R = float(np.mean(np.diag(R)))
    lam_ref = d_norm / max(d_R, 1e-12)

    if lam is None:
        lam = lam_scale * lam_ref

    if verbose:
        print(
            f"[order={order}] lam_ref ≈ {lam_ref:.4e} | "
            f"lam_scale={lam_scale:g} | using lam={lam:.4e}"
        )

    T_raw = q_demo.shape[0]
    k_via = map_raw_index_to_resampled(k_raw=k_raw, T_raw=T_raw, T=T)

    A, b = build_start_end_via_constraints(q_demo_T, T, k_via)
    if verbose:
        print(
            f"[order={order}, lam={lam:g}] via: raw {k_raw} -> "
            f"resampled {k_via} (T_raw={T_raw}, T={T})"
        )

    P, q_lin, _ = assemble_qp(L, Q, r, lam, R, x_demo)

    if solver == "kkt":
        x_opt, _ = solve_kkt(P, q_lin, A, b, verbose=verbose)
        status = "solved (kkt)"
    else:
        # x_opt, status = solve_qp(P, q_lin, A, b, verbose=verbose)
        # if x_opt is None:
        #    raise RuntimeError(status)
        raise ValueError("This solver was no included. Use kkt instead.")

    if verbose:
        print(
            f"[order={order}, lam={lam:g}] "
            f"||x_opt - x_demo||_2 = {np.linalg.norm(x_opt - x_demo):.6e}"
        )

    q_opt = x_opt.reshape((T, nj), order="F")
    return q_demo_T, q_opt, status, k_via


if __name__ == "__main__":
    DEMO_PATH = 'h5_files/recorded_demo 2026-02-10 14:56:02.h5'
    T_HORIZON = 500
    SOLVER = "kkt"  # "kkt" or "cvxpy"
    PLOT_RESULTS = True

    LAM_MID = 1.0  # only used for mixed-acc mode
    LAM_HIGH = 200.0  # only used for mixed-acc mode

    K_RAW = 2000

    MODE = "all"  # one of:
    # "all", "minacc", "minjerk", "mixed-acc", "mixed-jerk"

    JERK_MIXED_SCALE = 1e-11
    JERK_HEAVY_SCALE = 1.0

    VERBOSE = True
    q_demo = load_demo_q(DEMO_PATH)
    print('initial')
    print(q_demo.shape)
    solver_backend = "kkt" if SOLVER == "kkt" else "cvxpy"

    results = {}

    # 1) min-acc
    if MODE in ["all", "minacc"]:
        results["Minimum Acceleration (λ=0)"] = solve_normative_or_mixed(
            q_demo=q_demo,
            T=T_HORIZON,
            order=2,
            lam=0.0,
            solver=solver_backend,
            verbose=VERBOSE,
            k_raw=K_RAW,
        )

    # 2) min-jerk
    if MODE in ["all", "minjerk"]:
        results["Minimum Jerk (λ=0)"] = solve_normative_or_mixed(
            q_demo=q_demo,
            T=T_HORIZON,
            order=3,
            lam=0.0,
            solver=solver_backend,
            verbose=VERBOSE,
            k_raw=K_RAW,
        )

    # 3) mixed-jerk curves
    if MODE in ["all", "mixed-jerk"]:
        results[
            rf"Normative-Imitative (Jerk, $\lambda={JERK_MIXED_SCALE:g}\,\lambda_{{\mathrm{{ref}}}}$)"
        ] = solve_normative_or_mixed(
            q_demo=q_demo,
            T=T_HORIZON,
            order=3,
            lam=None,
            lam_scale=JERK_MIXED_SCALE,
            solver=solver_backend,
            verbose=VERBOSE,
            k_raw=K_RAW,
        )

        results[
            rf"Imitation-Dominant (Jerk, $\lambda={JERK_HEAVY_SCALE:g}\,\lambda_{{\mathrm{{ref}}}}$)"
        ] = solve_normative_or_mixed(
            q_demo=q_demo,
            T=T_HORIZON,
            order=3,
            lam=None,
            lam_scale=JERK_HEAVY_SCALE,
            solver=solver_backend,
            verbose=VERBOSE,
            k_raw=K_RAW,
        )

    # optional mixed-acc
    if MODE in ["all", "mixed-acc"]:
        results[f"mixed acc (λ={LAM_MID:g})"] = solve_normative_or_mixed(
            q_demo=q_demo,
            T=T_HORIZON,
            order=2,
            lam=LAM_MID,
            solver=solver_backend,
            verbose=VERBOSE,
            k_raw=K_RAW,
        )

        results[f"imit-heavy acc (λ={LAM_HIGH:g})"] = solve_normative_or_mixed(
            q_demo=q_demo,
            T=T_HORIZON,
            order=2,
            lam=LAM_HIGH,
            solver=solver_backend,
            verbose=VERBOSE,
            k_raw=K_RAW,
        )

    first_key = next(iter(results.keys()))
    q_demo_T, q_opt, _, via_idx = results[first_key]
    
    q_demo_T, q_opt, _, via_idx = results["Minimum Acceleration (λ=0)"]
    np.savetxt("min_acc2000.txt", q_opt)
    q_demo_T, q_opt, _, via_idx = results["Minimum Jerk (λ=0)"]
    np.savetxt("min_jerk2000.txt", q_opt)
    q_demo_T, q_opt, _, via_idx = results[rf"Normative-Imitative (Jerk, $\lambda={JERK_MIXED_SCALE:g}\,\lambda_{{\mathrm{{ref}}}}$)"]
    np.savetxt("norm_init2000.txt", q_opt)
    q_demo_T, q_opt, _, via_idx = results[rf"Imitation-Dominant (Jerk, $\lambda={JERK_HEAVY_SCALE:g}\,\lambda_{{\mathrm{{ref}}}}$)"]
    np.savetxt("imit_dom2000.txt", q_opt)
    q_demo_T, q_opt, _, via_idx = results[f"mixed acc (λ={LAM_MID:g})"]
    np.savetxt("mix_acc2000.txt", q_opt)
    q_demo_T, q_opt, _, via_idx = results[f"imit-heavy acc (λ={LAM_HIGH:g})"]
    np.savetxt("imit_heavy2000.txt", q_opt)

    for name, (qd, qo, status, _) in results.items():
        print("\n===", name, "===")
        print("Status:", status)
        print("Start error (inf-norm):", np.max(np.abs(qo[0] - qd[0])))
        print("End error   (inf-norm):", np.max(np.abs(qo[-1] - qd[-1])))
        print("Via error   (inf-norm):", np.max(np.abs(qo[via_idx] - qd[via_idx])))

    if PLOT_RESULTS:
        T, nj = q_demo_T.shape
        print('plotting')
        print(T, nj)
        t = np.arange(T)

        style_demo = dict(
            linestyle="--",
            linewidth=1.2,
            marker="o",
            markevery=max(1, T // 60),
            markersize=3.2,
            color="k",
            zorder=6,
            alpha=0.65,
        )

        style_norm = dict(
            linestyle="-",
            linewidth=2.0,
            alpha=0.85,
        )

        style_heavy = dict(
            linestyle="-",
            linewidth=2.2,
            alpha=0.55,
        )

        def is_heavy(name: str) -> bool:
            return "heavy" in name.lower()

        fig = plt.figure()
        fig.set_size_inches(8.5, 1.6 * nj)

        axes = fig.subplots(nj, 1, sharex=True)
        if nj == 1:
            axes = [axes]

        legend_handles = {}
        legend_order = ["demo"] + list(results.keys()) + ["start", "via", "goal"]

        for j in range(nj):
            for name, (_, q_opt, _, _) in results.items():
                style = style_heavy if is_heavy(name) else style_norm
                (ln,) = axes[j].plot(t, q_opt[:, j], label=name, **style)
                legend_handles.setdefault(name, ln)

            (demo_ln,) = axes[j].plot(t, q_demo_T[:, j], label="demo", **style_demo)
            legend_handles.setdefault("demo", demo_ln)

            start_ln = axes[j].scatter(
                0, q_demo_T[0, j], color="k", marker="o", s=45, zorder=10, label="start"
            )
            legend_handles.setdefault("start", start_ln)

            goal_ln = axes[j].scatter(
                t[-1],
                q_demo_T[-1, j],
                marker="*",
                color="k",
                s=45,
                zorder=10,
                label="goal",
            )
            legend_handles.setdefault("goal", goal_ln)

            via_ln = axes[j].scatter(
                t[via_idx],
                q_demo_T[via_idx, j],
                color="k",
                marker="^",
                s=45,
                zorder=10,
                label="via",
            )
            legend_handles.setdefault("via", via_ln)

            axes[j].set_ylabel(f"$q_{j+1}$ (rad)")
            axes[j].grid(True, alpha=0.25)

        axes[-1].set_xlabel("time step")

        handles = [legend_handles[k] for k in legend_order if k in legend_handles]
        labels = [k for k in legend_order if k in legend_handles]
        fig.legend(
            handles,
            labels,
            loc="lower center",
            ncol=3,
            frameon=False,
            bbox_to_anchor=(0.5, 0.015),
            fontsize=9,
            columnspacing=1.8,
            handletextpad=0.6,
        )

        fig.align_ylabels(axes)
        plt.tight_layout(rect=[0, 0.1, 1, 1.0])

        # fig.savefig("exp5_ur5_bottle_placing.png", dpi=300, bbox_inches="tight")
        plt.show()
