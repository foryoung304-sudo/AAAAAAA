# Agent mandatory rules

## Flight-control work

- Before analyzing flight logs, diagnosing flight behavior, changing control code, or recommending flight parameters, read `FLIGHT_DIAGNOSIS.md` in full. Treat its confirmed, withdrawn, unknown, and closed conclusions as constraints; do not revive a withdrawn conclusion without new evidence.
- Treat the current source and the runtime configuration recorded by the relevant flight log as authoritative when they conflict with stale prose. PID values must come from that flight's trailing `FLIGHTCFG`, not source defaults.
- After changing flight-control behavior, mission state machines, thresholds, limits, logging fields, or diagnostic conclusions, update `FLIGHT_DIAGNOSIS.md` in the same task. Record what changed, why, the evidence level, what older entry it supersedes, and what still needs flight validation. A code-only change is incomplete.
- If the user corrects a physical constraint or declares that the current implementation supersedes an older diagnosis entry, update `FLIGHT_DIAGNOSIS.md` before using that entry again.
- The aircraft and ground car are physically connected by a cable. Any mission or search design must account for cable tension: when the car moves far enough, the aircraft must follow. Do not recommend holding the aircraft fixed while the car travels away.

