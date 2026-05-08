from dataclasses import dataclass
from core.gain_schedule import GainSet, VoltageBand


@dataclass
class ScheduleState:
    active_band: str | None = None
    candidate_band: str | None = None
    candidate_since_ms: int = 0


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def interpolate_gain(voltage: float, bands: list[VoltageBand]) -> tuple[str, GainSet] | None:
    if not bands:
        return None
    for i, band in enumerate(bands):
        if band.min_v <= voltage < band.max_v:
            if i + 1 < len(bands):
                nxt = bands[i + 1]
                span = max(1e-6, band.max_v - band.min_v)
                t = min(1.0, max(0.0, (voltage - band.min_v) / span))
                return (
                    band.name,
                    GainSet(
                        kp=lerp(band.gains.kp, nxt.gains.kp, t),
                        ki=lerp(band.gains.ki, nxt.gains.ki, t),
                        kd=lerp(band.gains.kd, nxt.gains.kd, t),
                    ),
                )
            return band.name, band.gains
    return None


def debounce_band_choice(choice: str | None, now_ms: int, state: ScheduleState, debounce_ms: int) -> str | None:
    if choice is None:
        state.candidate_band = None
        return state.active_band
    if state.active_band is None:
        state.active_band = choice
        return state.active_band
    if choice == state.active_band:
        state.candidate_band = None
        return state.active_band
    if state.candidate_band != choice:
        state.candidate_band = choice
        state.candidate_since_ms = now_ms
        return state.active_band
    if now_ms - state.candidate_since_ms >= debounce_ms:
        state.active_band = choice
        state.candidate_band = None
    return state.active_band
