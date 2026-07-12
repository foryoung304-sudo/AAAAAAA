# EKF offline tuners

## Height

```powershell
C:\Users\44844\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe `
  project/tools/ekf_tuner.py `
  "C:\path\to\pasted-text.txt"
```

Outputs:

- `project/tools/ekf_tuner_out/grid_results.csv`
- `project/tools/ekf_tuner_out/baseline_series.csv`
- `project/tools/ekf_tuner_out/best_series.csv`

The truth velocity is a non-causal, zero-phase local polynomial derivative of
projected ToF height. It is a reference proxy, not independent ground truth.
Always validate candidate parameters on another flight log before changing C
constants.

## XY location

Paste a full `LOC_LOG_BEGIN` to `LOC_LOG_END` capture into a text file, then run
from repository root:

```powershell
C:\Users\44844\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe `
  project/tools/loc_ekf_tuner.py `
  "C:\path\to\pasted-text.txt"
```

Outputs:

- `project/tools/loc_ekf_tuner_out/grid_results.csv`
- `project/tools/loc_ekf_tuner_out/baseline_series.csv`
- `project/tools/loc_ekf_tuner_out/best_series.csv`

The XY tuner uses logged flow position and velocity as the reference proxy. It
is mainly for finding bad EKF lag, gate rejection, static drift, and overly loose
or overly stiff velocity fusion. It does not prove the absolute LC302 scale.
