import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config" / "devices"


@dataclass(frozen=True)
class ChannelSpec:
    id: int
    name: str
    display_name: str
    kind: str = "pid"
    unit: str = ""
    curves: tuple[str, ...] = ("target", "feedback", "error", "output")
    supports_param_set: bool = True


@dataclass(frozen=True)
class GroundParamSpec:
    name: str
    key: str
    default: str
    command: str
    type: str = "str"
    unit: str = ""
    description: str = ""
    category: str = "通用"
    danger: bool = False
    arm_command: str = ""


@dataclass(frozen=True)
class CommandSpec:
    name: str
    command: str
    danger: bool = False
    arm_command: str = ""


@dataclass(frozen=True)
class DeviceSchema:
    key: str
    device_id: int
    name: str
    display_name: str
    default_baud: int
    telemetry_period_ms: int
    channels: tuple[ChannelSpec, ...]
    ground_params: tuple[GroundParamSpec, ...]
    commands: tuple[CommandSpec, ...]
    status_flags: dict[int, str]
    raw: dict[str, Any]

    @property
    def channel_defs(self) -> dict[tuple[int, int], str]:
        return {(self.device_id, ch.id): ch.display_name for ch in self.channels}

    @property
    def param_commands(self) -> list[tuple[str, str, str, str]]:
        return [(p.name, p.default, p.command, p.description) for p in self.ground_params]


def parse_schema(data: dict[str, Any], key: str = "") -> DeviceSchema:
    device = data.get("device", {})
    transport = data.get("transport", {})
    channels = tuple(
        ChannelSpec(
            id=int(item["id"]),
            name=str(item.get("name", f"ch{item['id']}")),
            display_name=str(item.get("display_name", item.get("name", f"通道{item['id']}"))),
            kind=str(item.get("kind", "pid")),
            unit=str(item.get("unit", "")),
            curves=tuple(item.get("curves", ["target", "feedback", "error", "output"])),
            supports_param_set=bool(item.get("supports_param_set", True)),
        )
        for item in data.get("channels", [])
    )
    params = tuple(
        GroundParamSpec(
            name=str(item["name"]),
            key=str(item.get("key", item["name"])),
            default=str(item.get("default", "")),
            command=str(item.get("command", "")),
            type=str(item.get("type", "str")),
            unit=str(item.get("unit", "")),
            description=str(item.get("description", "")),
            category=str(item.get("category", "通用")),
            danger=bool(item.get("danger", False)),
            arm_command=str(item.get("arm_command", "")),
        )
        for item in data.get("ground_params", [])
    )
    commands = tuple(
        CommandSpec(
            name=str(item["name"]),
            command=str(item.get("command", "")),
            danger=bool(item.get("danger", False)),
            arm_command=str(item.get("arm_command", "")),
        )
        for item in data.get("commands", [])
    )
    return DeviceSchema(
        key=key or str(device.get("name", "device")),
        device_id=int(device.get("id", 1)),
        name=str(device.get("name", "device")),
        display_name=str(device.get("display_name", device.get("name", "设备"))),
        default_baud=int(transport.get("default_baud", 115200)),
        telemetry_period_ms=int(transport.get("telemetry_period_ms", 100)),
        channels=channels,
        ground_params=params,
        commands=commands,
        status_flags={int(k): str(v) for k, v in data.get("status_flags", {}).items()},
        raw=data,
    )


def load_schema(path: str | Path) -> DeviceSchema:
    p = Path(path)
    return parse_schema(json.loads(p.read_text(encoding="utf-8")), p.stem)


def load_default_schema() -> DeviceSchema:
    path = CONFIG_DIR / "smart_car.json"
    return load_schema(path)


def list_schema_files() -> list[Path]:
    if not CONFIG_DIR.exists():
        return []
    return sorted(CONFIG_DIR.glob("*.json"))


def save_schema(schema: dict[str, Any], path: str | Path) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(schema, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
