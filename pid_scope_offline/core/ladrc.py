from dataclasses import dataclass


@dataclass
class LADRCConfig:
    b0: float
    wc: float
    wo: float
    u_min: float = -1e9
    u_max: float = 1e9


@dataclass
class LADRCState:
    z1: float = 0.0
    z2: float = 0.0


def clamp(x: float, mn: float, mx: float) -> float:
    return max(mn, min(mx, x))


def ladrc_step(target: float, feedback: float, dt: float, cfg: LADRCConfig, st: LADRCState) -> tuple[float, LADRCState]:
    beta1 = 2.0 * cfg.wo
    beta2 = cfg.wo * cfg.wo
    e = st.z1 - feedback
    st.z1 += dt * (st.z2 - beta1 * e)
    st.z2 += dt * (-beta2 * e)
    kp = cfg.wc * cfg.wc
    kd = 2.0 * cfg.wc
    u0 = kp * (target - st.z1) - kd * st.z2
    u = clamp((u0 - st.z2) / max(1e-6, cfg.b0), cfg.u_min, cfg.u_max)
    return u, st
