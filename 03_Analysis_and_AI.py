#!/usr/bin/env python3
"""
Analysis and AI evaluation for the external meter-integrity verifier.

Commands
  synthetic         Generate a SYNTHETIC dataset (pipeline test only - NOT thesis data).
  from-meter-log    Build window data from the Mega's USB/terminal CSV log
                    (uses the meter's E_ref column plus simulated instrument noise).
  analyze           Evaluate threshold+persistence baseline and ML models on a
                    window CSV (from the ESP32 or from the commands above).

Window CSV columns (same as the ESP32 output):
  run,win,t_s,V,I,P,PF,F,dRef_Wh,dMeter_Wh,e_pct,U_pct,dU,count,status,label

Examples
  python 03_Analysis_and_AI.py synthetic --out synth.csv
  python 03_Analysis_and_AI.py analyze --csv synth.csv --tol 3 --persist 3
  python 03_Analysis_and_AI.py from-meter-log --log meter_log.csv --out windows.csv --window 60
  python 03_Analysis_and_AI.py analyze --csv run1.csv run2.csv run3.csv
"""
import argparse
import sys
import numpy as np
import pandas as pd

FEATURES = ["U_pct", "e_pct", "dU", "P", "PF", "V", "I", "F", "dRef_Wh"]


# ----------------------------------------------------------------------------
# Data builders
# ----------------------------------------------------------------------------
def add_derived(df):
    """Recompute e_pct, U_pct, dU from energies so every source is consistent."""
    df = df.copy()
    ok = df["dRef_Wh"] > 0
    df["e_pct"] = np.where(ok, (df["dMeter_Wh"] - df["dRef_Wh"]) / df["dRef_Wh"] * 100, np.nan)
    df["U_pct"] = -df["e_pct"]
    df["dU"] = df.groupby("run")["U_pct"].diff().fillna(0.0)
    return df


def make_synthetic(n_runs=40, windows=30, noise_pct=0.4, seed=1):
    rng = np.random.default_rng(seed)
    rows = []
    for run in range(n_runs):
        label = int(run % 2)                      # alternate normal / under-registering
        u_true = rng.uniform(4, 25) if label else 0.0
        p = rng.choice([100, 200, 300]) * rng.uniform(0.97, 1.03)
        pf = rng.uniform(0.9, 1.0)
        for w in range(windows):
            d_ref = p * 60 / 3600.0 * rng.uniform(0.98, 1.02)
            meas_err = rng.normal(0, noise_pct)    # meter + reference noise, percent
            d_met = d_ref * (1 - u_true / 100.0) * (1 + meas_err / 100.0)
            rows.append(dict(run=run, win=w, t_s=(w + 1) * 60, V=230 + rng.normal(0, 1),
                             I=p / (230 * pf), P=p, PF=pf, F=50 + rng.normal(0, 0.02),
                             dRef_Wh=d_ref, dMeter_Wh=d_met, count=0, status="", label=label))
    return add_derived(pd.DataFrame(rows))


def from_meter_log(path, window_s, noise_pct, seed=2):
    """Meter log columns: t_s,Vrms,Irms,P_W,PF,E_meter_kWh,E_ref_kWh,Cost_N,Balance_N,Relay,Label"""
    rng = np.random.default_rng(seed)
    log = pd.read_csv(path, comment="#")
    log = log.dropna()
    rows = []
    t0 = log["t_s"].iloc[0]
    log["win"] = ((log["t_s"] - t0) // window_s).astype(int)
    for w, g in log.groupby("win"):
        if len(g) < 2:
            continue
        d_ref = (g["E_ref_kWh"].iloc[-1] - g["E_ref_kWh"].iloc[0]) * 1000.0
        d_met = (g["E_meter_kWh"].iloc[-1] - g["E_meter_kWh"].iloc[0]) * 1000.0
        d_ref_meas = d_ref * (1 + rng.normal(0, noise_pct / 100.0))   # simulated instrument
        rows.append(dict(run=0, win=w, t_s=g["t_s"].iloc[-1], V=g["Vrms"].mean(), I=g["Irms"].mean(),
                         P=g["P_W"].mean(), PF=g["PF"].mean(), F=50.0, dRef_Wh=d_ref_meas,
                         dMeter_Wh=d_met, count=0, status="", label=int(g["Label"].iloc[-1])))
    return add_derived(pd.DataFrame(rows))


def load_csvs(paths):
    parts = []
    for i, p in enumerate(paths):
        d = pd.read_csv(p, comment="#")
        d["run"] = d["run"].astype(int) + 1000 * i      # keep files separate
        parts.append(d)
    df = pd.concat(parts, ignore_index=True)
    df = df.replace([np.inf, -np.inf], np.nan)
    return add_derived(df)


# ----------------------------------------------------------------------------
# Detection methods
# ----------------------------------------------------------------------------
def suggest_tolerance(df, k=3.0):
    """Tolerance from NORMAL windows only: mean(|U|) + k * std(U)."""
    u = df.loc[(df["label"] == 0) & df["U_pct"].notna(), "U_pct"]
    if len(u) < 5:
        return None
    return float(np.abs(u.mean()) + k * u.std())


def threshold_persistence(df, tol, persist):
    """Per-window prediction: 1 once U% > tol for `persist` consecutive windows."""
    pred = np.zeros(len(df), dtype=int)
    for run, g in df.groupby("run", sort=False):
        count = 0
        for idx, u in zip(g.index, g["U_pct"]):
            count = count + 1 if (pd.notna(u) and u > tol) else 0
            pred[df.index.get_loc(idx)] = int(count >= persist)
    return pred


def metrics(y, yhat):
    y = np.asarray(y)
    yhat = np.asarray(yhat)
    tp = int(((y == 1) & (yhat == 1)).sum())
    tn = int(((y == 0) & (yhat == 0)).sum())
    fp = int(((y == 0) & (yhat == 1)).sum())
    fn = int(((y == 1) & (yhat == 0)).sum())
    prec = tp / (tp + fp) if tp + fp else float("nan")
    rec = tp / (tp + fn) if tp + fn else float("nan")
    f1 = 2 * prec * rec / (prec + rec) if (prec == prec and rec == rec and prec + rec) else float("nan")
    fpr = fp / (fp + tn) * 100 if fp + tn else float("nan")
    return dict(TP=tp, FN=fn, FP=fp, TN=tn, Detection_rate_pct=rec * 100 if rec == rec else float("nan"),
                False_positive_rate_pct=fpr, Precision=prec, Recall=rec, F1=f1)


def show(name, m):
    print(f"\n{name}")
    print(f"  confusion  TP={m['TP']}  FN={m['FN']}  FP={m['FP']}  TN={m['TN']}")
    print(f"  detection rate = {m['Detection_rate_pct']:.1f} %   false-positive rate = {m['False_positive_rate_pct']:.1f} %")
    print(f"  precision = {m['Precision']:.3f}   recall = {m['Recall']:.3f}   F1 = {m['F1']:.3f}")


def analyze(df, tol, persist, seed=0):
    from sklearn.model_selection import GroupShuffleSplit
    from sklearn.tree import DecisionTreeClassifier
    from sklearn.ensemble import RandomForestClassifier

    df = df.dropna(subset=["U_pct"]).reset_index(drop=True)
    n_runs = df["run"].nunique()
    print(f"windows = {len(df)}   runs = {n_runs}   under-registering windows = {int(df['label'].sum())}")
    if n_runs < 4 or df["label"].nunique() < 2:
        sys.exit("Need at least 4 runs containing BOTH label 0 and label 1.")

    # Split by RUN so closely related windows never sit in both train and test.
    gss = GroupShuffleSplit(n_splits=1, test_size=0.3, random_state=seed)
    tr, te = next(gss.split(df, df["label"], groups=df["run"]))
    train, test = df.iloc[tr].reset_index(drop=True), df.iloc[te].reset_index(drop=True)
    print(f"train runs = {train['run'].nunique()}   test runs = {test['run'].nunique()}")

    if tol is None:
        tol = suggest_tolerance(train)
        print(f"tolerance chosen from TRAIN normal windows (mean + 3 sigma) = {tol:.2f} %")
    else:
        print(f"tolerance supplied = {tol:.2f} %")

    show("Threshold + persistence baseline (test set)",
         metrics(test["label"], threshold_persistence(test, tol, persist)))

    X_tr, y_tr = train[FEATURES].fillna(0), train["label"]
    X_te, y_te = test[FEATURES].fillna(0), test["label"]
    for name, model in [("Decision tree (depth 3)", DecisionTreeClassifier(max_depth=3, random_state=seed)),
                        ("Random forest (100 trees)", RandomForestClassifier(n_estimators=100, random_state=seed))]:
        model.fit(X_tr, y_tr)
        show(f"{name} (test set)", metrics(y_te, model.predict(X_te)))
        if hasattr(model, "feature_importances_"):
            imp = sorted(zip(FEATURES, model.feature_importances_), key=lambda t: -t[1])[:4]
            print("  top features: " + ", ".join(f"{n} {v:.2f}" for n, v in imp))

    print("\nNOTE: report only results from experiments you actually ran. "
          "Synthetic data checks that the pipeline works; it is not evidence.")


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("synthetic")
    s.add_argument("--out", default="synthetic_windows.csv")
    s.add_argument("--runs", type=int, default=40)

    f = sub.add_parser("from-meter-log")
    f.add_argument("--log", required=True)
    f.add_argument("--out", default="windows_from_meter.csv")
    f.add_argument("--window", type=float, default=60.0)
    f.add_argument("--noise", type=float, default=0.3, help="simulated reference noise, percent")

    a = sub.add_parser("analyze")
    a.add_argument("--csv", nargs="+", required=True)
    a.add_argument("--tol", type=float, default=None, help="tolerance %% (default: learned from normal windows)")
    a.add_argument("--persist", type=int, default=3)

    args = ap.parse_args()
    if args.cmd == "synthetic":
        make_synthetic(n_runs=args.runs).to_csv(args.out, index=False)
        print(f"wrote {args.out} (SYNTHETIC)")
    elif args.cmd == "from-meter-log":
        from_meter_log(args.log, args.window, args.noise).to_csv(args.out, index=False)
        print(f"wrote {args.out}")
    else:
        analyze(load_csvs(args.csv), args.tol, args.persist)


if __name__ == "__main__":
    main()
