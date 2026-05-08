from dataclasses import dataclass


@dataclass(frozen=True)
class GainSet:
    kp: float
    ki: float
    kd: float


@dataclass(frozen=True)
class VoltageBand:
    name: str
    min_v: float
    max_v: float
    gains: GainSet


def build_bands(config: dict) -> list[VoltageBand]:
    bands: list[VoltageBand] = []
    for item in config.get("voltage_bands", []):
        gains = item.get("gains", {})
        bands.append(
            VoltageBand(
                name=item["name"],
                min_v=float(item["min_v"]),
                max_v=float(item["max_v"]),
                gains=GainSet(float(gains["kp"]), float(gains["ki"]), float(gains["kd"])),
            )
        )
    return sorted(bands, key=lambda b: b.min_v)


def pick_gain_by_voltage(voltage: float, bands: list[VoltageBand]) -> tuple[str, GainSet] | None:
    for band in bands:
        if band.min_v <= voltage < band.max_v:
            return band.name, band.gains
    return None
