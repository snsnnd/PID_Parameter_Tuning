from pathlib import Path

from core.device_schema import DeviceSchema, load_schema


ROOT = Path(__file__).resolve().parents[1]
GENERATED_DIR = ROOT / "generated"


def c_ident(name: str) -> str:
    out = []
    for ch in name.upper():
        out.append(ch if ("A" <= ch <= "Z" or "0" <= ch <= "9") else "_")
    text = "".join(out).strip("_")
    return text or "DEVICE"


def generate_driver(schema: DeviceSchema, out_root: str | Path | None = None) -> Path:
    out_dir = Path(out_root) if out_root else GENERATED_DIR / schema.name
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "pid_debug_config.h").write_text(render_config_h(schema), encoding="utf-8")
    (out_dir / "pid_debug_adapter.h").write_text(render_adapter_h(schema), encoding="utf-8")
    (out_dir / "pid_debug_adapter.c").write_text(render_adapter_c(schema), encoding="utf-8")
    (out_dir / "README.md").write_text(render_readme(schema), encoding="utf-8")
    return out_dir


def generate_driver_from_file(path: str | Path) -> Path:
    return generate_driver(load_schema(path))


def render_config_h(schema: DeviceSchema) -> str:
    prefix = c_ident(schema.name)
    lines = [
        "#ifndef PID_DEBUG_CONFIG_H",
        "#define PID_DEBUG_CONFIG_H",
        "",
        f"#define {prefix}_DEVICE_ID ({schema.device_id}U)",
        f"#define {prefix}_TELEMETRY_PERIOD_MS ({schema.telemetry_period_ms}U)",
        "",
    ]
    for ch in schema.channels:
        lines.append(f"#define {prefix}_CH_{c_ident(ch.name)} ({ch.id}U)")
    lines.append("")
    for bit, name in sorted(schema.status_flags.items()):
        ident = c_ident(name)
        if ident == "DEVICE":
            ident = f"BIT_{bit}"
        lines.append(f"#define {prefix}_STATUS_{ident} (1U << {bit})")
    lines.extend(["", "#endif", ""])
    return "\n".join(lines)


def render_adapter_h(schema: DeviceSchema) -> str:
    prefix = c_ident(schema.name)
    lines = [
        "#ifndef PID_DEBUG_ADAPTER_H",
        "#define PID_DEBUG_ADAPTER_H",
        "",
        "#include \"pid_debug.h\"",
        "#include \"pid_debug_config.h\"",
        "",
        "#ifdef __cplusplus",
        "extern \"C\" {",
        "#endif",
        "",
        "typedef struct {",
        "    float target;",
        "    float feedback;",
        "    float output;",
        "    float kp;",
        "    float ki;",
        "    float kd;",
        "    float p_term;",
        "    float i_term;",
        "    float d_term;",
        "    float extra1;",
        "    float extra2;",
        "    uint16_t status_flags;",
        "} pid_debug_channel_sample_t;",
        "",
        "int user_pid_debug_write(const uint8_t *data, uint16_t len);",
        "uint32_t user_pid_debug_time_ms(void);",
        "void user_pid_debug_apply_param(uint8_t channel_id, const pid_debug_param_t *param);",
        "",
        f"void {prefix.lower()}_pid_debug_init(void);",
        f"void {prefix.lower()}_pid_debug_rx_byte(uint8_t byte);",
        f"void {prefix.lower()}_pid_debug_poll(void);",
        f"int {prefix.lower()}_pid_debug_report(uint8_t channel_id, const pid_debug_channel_sample_t *sample);",
        f"int {prefix.lower()}_pid_debug_heartbeat(uint16_t status_flags);",
        "",
        "#ifdef __cplusplus",
        "}",
        "#endif",
        "",
        "#endif",
        "",
    ]
    return "\n".join(lines)


def render_adapter_c(schema: DeviceSchema) -> str:
    prefix = c_ident(schema.name)
    fn = prefix.lower()
    lines = [
        "#include \"pid_debug_adapter.h\"",
        "",
        "static void on_param_set(uint8_t device_id, uint8_t channel_id, const pid_debug_param_t *param) {",
        f"    if (device_id != {prefix}_DEVICE_ID || param == 0) return;",
        "    user_pid_debug_apply_param(channel_id, param);",
        "}",
        "",
        f"void {fn}_pid_debug_init(void) {{",
        "    pid_debug_port_t port = { user_pid_debug_write, user_pid_debug_time_ms, on_param_set };",
        "    pid_debug_init(&port);",
        "}",
        "",
        f"void {fn}_pid_debug_rx_byte(uint8_t byte) {{ pid_debug_rx_byte(byte); }}",
        f"void {fn}_pid_debug_poll(void) {{ pid_debug_poll(); }}",
        "",
        f"int {fn}_pid_debug_report(uint8_t channel_id, const pid_debug_channel_sample_t *sample) {{",
        "    if (sample == 0) return -1;",
        "    pid_debug_telemetry_t t = {0};",
        "    t.timestamp_ms = user_pid_debug_time_ms();",
        "    t.target = sample->target;",
        "    t.feedback = sample->feedback;",
        "    t.error = sample->target - sample->feedback;",
        "    t.output = sample->output;",
        "    t.kp = sample->kp; t.ki = sample->ki; t.kd = sample->kd;",
        "    t.p_term = sample->p_term; t.i_term = sample->i_term; t.d_term = sample->d_term;",
        "    t.extra1 = sample->extra1; t.extra2 = sample->extra2;",
        "    t.status_flags = sample->status_flags;",
        f"    return pid_debug_send_telemetry({prefix}_DEVICE_ID, channel_id, &t);",
        "}",
        "",
        f"int {fn}_pid_debug_heartbeat(uint16_t status_flags) {{",
        f"    return pid_debug_send_heartbeat({prefix}_DEVICE_ID, status_flags, user_pid_debug_time_ms());",
        "}",
        "",
    ]
    return "\n".join(lines)


def render_readme(schema: DeviceSchema) -> str:
    channels = "\n".join(f"- `{ch.id}` {ch.display_name} ({ch.name})" for ch in schema.channels)
    return f"""# {schema.display_name} PID Debug Adapter

Generated adapter for device `{schema.name}`.

## Fixed User API

Implement these functions in your firmware:

```c
int user_pid_debug_write(const uint8_t *data, uint16_t len);
uint32_t user_pid_debug_time_ms(void);
void user_pid_debug_apply_param(uint8_t channel_id, const pid_debug_param_t *param);
```

## Channels

{channels}

Copy `pid_debug.c/.h`, `pid_debug_config.h`, and `pid_debug_adapter.c/.h` into your firmware project.
"""
