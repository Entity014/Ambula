import os
import time
import csv
import numpy as np
import onnxruntime as ort

# ถ้ามี pandas จะทำ table สวยและเซฟง่ายขึ้น
try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False

ONNX_PATH  = "/home/xero/Ambula/robot_ws/src/ambula_rl/src/policy.onnx"
OBS_NPY    = "/home/xero/Ambula/robot_ws/src/ambula_rl/src/obs_dump.npy"

CSV_ACT_LAT = "/home/xero/Ambula/robot_ws/src/ambula_rl/src/onnx_latency_actions.csv"
CSV_OBS     = "/home/xero/Ambula/robot_ws/src/ambula_rl/src/obs_table.csv"
CSV_ALL     = "/home/xero/Ambula/robot_ws/src/ambula_rl/src/obs_act_latency.csv"

SAVE_ACT_LAT = True
SAVE_OBS     = True
SAVE_ALL     = True

PRINT_OBS_ROWS = 20

CLIP_ACTION = False   # ถ้าจะให้เหมือน play.py ให้ True (clamp -1..1)
CLIP_MIN, CLIP_MAX = -1.0, 1.0
WARMUP = 50

# ต้องยาว = obs_dim (เช่น 36) ถ้าไม่ตรง โค้ดจะ fallback เป็น obs_0..obs_{d-1}
OBS_NAMES = [
    # stack_policy t-2 (10)
    "wheel_vel_L[t-2]", "wheel_vel_R[t-2]",
    "ang_vel_x[t-2]", "ang_vel_y[t-2]", "ang_vel_z[t-2]",
    "grav_x[t-2]", "grav_y[t-2]", "grav_z[t-2]",
    "act_L[t-2]", "act_R[t-2]",

    # stack_policy t-1 (10)
    "wheel_vel_L[t-1]", "wheel_vel_R[t-1]",
    "ang_vel_x[t-1]", "ang_vel_y[t-1]", "ang_vel_z[t-1]",
    "grav_x[t-1]", "grav_y[t-1]", "grav_z[t-1]",
    "act_L[t-1]", "act_R[t-1]",

    # stack_policy t (10)
    "wheel_vel_L[t]", "wheel_vel_R[t]",
    "ang_vel_x[t]", "ang_vel_y[t]", "ang_vel_z[t]",
    "grav_x[t]", "grav_y[t]", "grav_z[t]",
    "act_L[t]", "act_R[t]",

    # none_stack_policy t (6)
    "cmd_vx[t]", "cmd_vy[t]", "cmd_wz[t]", "cmd_z_hold[t]",
    "base_lin_vel[t]",
    "base_pos_z[t]",
]



def ensure_parent_dir(path: str) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)


def pick_io_names(sess: ort.InferenceSession):
    """เลือก input/output แรกแบบปลอดภัย"""
    inputs = sess.get_inputs()
    outputs = sess.get_outputs()
    if len(inputs) < 1:
        raise RuntimeError("ONNX model has no inputs")
    if len(outputs) < 1:
        raise RuntimeError("ONNX model has no outputs")
    return inputs[0], outputs[0]


def main():
    # -------------------------
    # Load obs
    # -------------------------
    if not os.path.exists(OBS_NPY):
        raise FileNotFoundError(f"OBS_NPY not found: {OBS_NPY}")
    obs = np.load(OBS_NPY)

    # รองรับกรณี obs เป็น (36,) -> ทำให้เป็น (1,36)
    obs = np.asarray(obs)
    if obs.ndim == 1:
        obs = obs.reshape(1, -1)

    # บังคับ float32 (สำคัญกับ onnxruntime และตรงกับโมเดลส่วนใหญ่)
    obs = obs.astype(np.float32, copy=False)

    N, obs_dim = obs.shape
    print("obs:", obs.shape, obs.dtype)

    # -------------------------
    # ONNX session options (ให้ทนและเร็วขึ้น)
    # -------------------------
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    so.intra_op_num_threads = max(1, os.cpu_count() // 2)  # ปรับได้ตามเครื่อง
    so.inter_op_num_threads = 1

    if not os.path.exists(ONNX_PATH):
        raise FileNotFoundError(f"ONNX_PATH not found: {ONNX_PATH}")

    sess = ort.InferenceSession(ONNX_PATH, sess_options=so, providers=["CPUExecutionProvider"])
    inp, out0 = pick_io_names(sess)

    print("input:", inp.name, inp.shape, inp.type)
    print("output:", out0.name, out0.shape, out0.type)

    # เช็ค obs_dim ให้ตรง input feature ถ้า input shape ระบุได้
    # inp.shape มักเป็น [None, 36] หรือ ['batch', 36]
    if isinstance(inp.shape, (list, tuple)) and len(inp.shape) == 2:
        feat = inp.shape[1]
        if isinstance(feat, int) and feat != obs_dim:
            raise ValueError(f"obs_dim ({obs_dim}) != model feature dim ({feat}). ต้อง dump obs ให้ตรงโมเดล")

    # -------------------------
    # Warm-up (สุ่มหลาย ๆ sample)
    # -------------------------
    warm_n = min(WARMUP, N)
    for i in range(warm_n):
        _ = sess.run(None, {inp.name: obs[i:i+1]})
    print("Warm-up done")

    # -------------------------
    # Run: latency + actions
    # -------------------------
    lat_ms = np.empty((N,), dtype=np.float64)
    acts   = np.empty((N, 2), dtype=np.float32)

    t_start = time.perf_counter()
    for i in range(N):
        x = obs[i:i+1]  # (1, obs_dim)

        t0 = time.perf_counter_ns()
        y = sess.run(None, {inp.name: x})
        t1 = time.perf_counter_ns()

        lat_ms[i] = (t1 - t0) / 1e6

        act = np.asarray(y[0], dtype=np.float32).reshape(-1)  # (2,)
        if act.size != 2:
            raise RuntimeError(f"Expected action dim=2 but got {act.size} at i={i}")

        if CLIP_ACTION:
            act = np.clip(act, CLIP_MIN, CLIP_MAX)
        acts[i, :] = act

    t_end = time.perf_counter()

    # -------------------------
    # Stats
    # -------------------------
    print("\nLatency stats (ms):")
    print("  mean =", float(lat_ms.mean()))
    print("  p95  =", float(np.percentile(lat_ms, 95)))
    print("  p99  =", float(np.percentile(lat_ms, 99)))
    print("  max  =", float(lat_ms.max()))
    print("Total wall time (s):", round(t_end - t_start, 3), "for", N, "samples")

    print("\nAction stats:")
    print("  min  =", acts.min(axis=0))
    print("  max  =", acts.max(axis=0))
    print("  mean =", acts.mean(axis=0))

    print("\nFirst 10 actions:")
    for i in range(min(10, N)):
        print(i, float(acts[i, 0]), float(acts[i, 1]), "lat(ms)=", round(float(lat_ms[i]), 4))

    # -------------------------
    # OBS TABLE (print + save)
    # -------------------------
    if (not isinstance(OBS_NAMES, list)) or (len(OBS_NAMES) != obs_dim):
        print(f"\n[WARN] OBS_NAMES length ({len(OBS_NAMES) if isinstance(OBS_NAMES, list) else 'N/A'}) "
              f"!= obs_dim ({obs_dim}) -> fallback to obs_0..obs_{obs_dim-1}")
        obs_cols = [f"obs_{j}" for j in range(obs_dim)]
    else:
        obs_cols = OBS_NAMES

    # ensure directories
    if SAVE_OBS:
        ensure_parent_dir(CSV_OBS)
    if SAVE_ACT_LAT:
        ensure_parent_dir(CSV_ACT_LAT)
    if SAVE_ALL:
        ensure_parent_dir(CSV_ALL)

    if HAS_PANDAS:
        df_obs = pd.DataFrame(obs, columns=obs_cols)

        print(f"\nOBS table (first {min(PRINT_OBS_ROWS, N)} rows):")
        print(df_obs.head(min(PRINT_OBS_ROWS, N)).to_string(index=False))

        if SAVE_OBS:
            df_obs.to_csv(CSV_OBS, index=False)
            print("\nSaved obs table ->", CSV_OBS)

        if SAVE_ACT_LAT:
            df_act = pd.DataFrame({
                "i": np.arange(N, dtype=np.int32),
                "latency_ms": lat_ms,
                "action_left": acts[:, 0],
                "action_right": acts[:, 1],
            })
            df_act.to_csv(CSV_ACT_LAT, index=False)
            print("Saved act+lat ->", CSV_ACT_LAT)

        if SAVE_ALL:
            df_all = df_obs.copy()
            df_all["latency_ms"] = lat_ms
            df_all["action_left"] = acts[:, 0]
            df_all["action_right"] = acts[:, 1]
            df_all.to_csv(CSV_ALL, index=False)
            print("Saved combined table ->", CSV_ALL)

    else:
        print(f"\n[WARN] pandas not installed -> printing/saving via csv module")

        # print first rows
        print("\nOBS table (first rows):")
        print(",".join(obs_cols))
        for i in range(min(PRINT_OBS_ROWS, N)):
            print(",".join(f"{v:.6g}" for v in obs[i]))

        if SAVE_OBS:
            with open(CSV_OBS, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(obs_cols)
                w.writerows(obs.tolist())
            print("\nSaved obs table ->", CSV_OBS)

        if SAVE_ACT_LAT:
            with open(CSV_ACT_LAT, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["i", "latency_ms", "action_left", "action_right"])
                for i in range(N):
                    w.writerow([i, float(lat_ms[i]), float(acts[i, 0]), float(acts[i, 1])])
            print("Saved act+lat ->", CSV_ACT_LAT)

        if SAVE_ALL:
            with open(CSV_ALL, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([*obs_cols, "latency_ms", "action_left", "action_right"])
                for i in range(N):
                    w.writerow([*obs[i].tolist(), float(lat_ms[i]), float(acts[i, 0]), float(acts[i, 1])])
            print("Saved combined table ->", CSV_ALL)


if __name__ == "__main__":
    main()
