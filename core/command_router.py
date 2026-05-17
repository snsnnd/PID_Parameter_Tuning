from dataclasses import dataclass, field


@dataclass
class CommandPlan:
    commands: list[str]
    updates: dict[str, str] = field(default_factory=dict)
    status: str = ""


class CommandRouter:
    def toggle_pid_scope_stream(self, runtime_state: dict[str, str], current_channel: tuple[int, int]) -> CommandPlan:
        pid_scope_on = runtime_state.get("PID_SCOPE") == "1" and runtime_state.get("PID_SCOPE_OVER_HC05") == "1"
        if pid_scope_on:
            return CommandPlan(
                commands=["CFG SET PID_SCOPE 0", "CFG SET PID_SCOPE_OVER_HC05 0"],
                updates={"PID_SCOPE": "0", "PID_SCOPE_OVER_HC05": "0"},
                status="已请求关闭 PIDScope 波形流。",
            )
        d, c = current_channel
        return CommandPlan(
            commands=["CFG SET PID_SCOPE_OVER_HC05 1", "CFG SET PID_SCOPE 1", f"SCOPE {d} {c}"],
            updates={"PID_SCOPE_OVER_HC05": "1", "PID_SCOPE": "1"},
            status="已请求开启 PIDScope 波形流；当前9600波特率下不要频繁查询CFG，避免拥塞。",
        )

    def start_spin_test(self, speed: float) -> CommandPlan:
        return CommandPlan(
            commands=["CFG TIMEOUT 10000", "CFG ARM MOTOR_OUTPUT", "CFG SET MOTOR_OUTPUT 1", f"CFG SPEED {speed:.3f}", "MOTOR 1", "START"],
            updates={"MOTOR_OUTPUT": "1", "MOTOR": "1", "SPIN_TEST": "1", "SPEED_TUNE": "0"},
            status="悬空持续转测试已启动：上位机会每2秒发送静默START心跳，避免远控超时停车。",
        )

    def start_speed_pid_tune(self, speed: float, target: str) -> CommandPlan:
        target = target if target in ("LEFT", "RIGHT", "BOTH") else "BOTH"
        scope_cmd = "SCOPE SPEED" if target == "BOTH" else ("SCOPE 1 2" if target == "RIGHT" else "SCOPE 1 1")
        return CommandPlan(
            commands=[
                "CFG SET PID_SCOPE_OVER_HC05 1", "CFG SET PID_SCOPE 1", "CFG TIMEOUT 10000",
                "CFG ARM MOTOR_OUTPUT", "CFG SET MOTOR_OUTPUT 1", "CFG ARM SPEED_PID", "CFG SET SPEED_PID 1",
                f"TUNE {target} {speed:.3f}", scope_cmd, "MOTOR 1", "START",
            ],
            updates={
                "PID_SCOPE_OVER_HC05": "1", "PID_SCOPE": "1", "MOTOR_OUTPUT": "1",
                "SPEED_PID": "1", "MOTOR": "1", "SPIN_TEST": "1", "SPEED_TUNE": "1",
            },
            status=f"悬空闭环调参已启动：目标={target}，未选中的轮目标速度为 0。",
        )

    def prepare_step_sample(self, target: str) -> CommandPlan:
        target = target if target in ("LEFT", "RIGHT") else "LEFT"
        scope_cmd = "SCOPE 1 2" if target == "RIGHT" else "SCOPE 1 1"
        return CommandPlan(
            commands=[
                "CFG SET PID_SCOPE_OVER_HC05 1", "CFG SET PID_SCOPE 1", "CFG TIMEOUT 10000",
                "CFG ARM MOTOR_OUTPUT", "CFG SET MOTOR_OUTPUT 1", "CFG ARM SPEED_PID", "CFG SET SPEED_PID 1",
                "SCOPE PERIOD 100", scope_cmd, f"TUNE {target} 0.000", "MOTOR 1", "START",
            ],
            updates={
                "PID_SCOPE_OVER_HC05": "1", "PID_SCOPE": "1", "MOTOR_OUTPUT": "1",
                "SPEED_PID": "1", "MOTOR": "1", "SPIN_TEST": "1", "SPEED_TUNE": "1",
            },
            status=f"半自动阶跃采样准备中：目标={target}，先置零再发小阶跃。",
        )

    def fire_step_sample(self, target: str, speed: float) -> CommandPlan:
        target = target if target in ("LEFT", "RIGHT") else "LEFT"
        return CommandPlan(
            commands=[f"TUNE {target} {speed:.3f}", "START"],
            updates={"SPEED": f"{speed:.3f}"},
            status=f"已发出半自动阶跃：目标={target}，速度={speed:.3f}m/s；采样稳定后点击阶跃分析。",
        )

    def start_motor_test(self, mode: str, pwm: int) -> CommandPlan:
        return CommandPlan(
            commands=["CFG TIMEOUT 10000", "CFG ARM MOTOR_OUTPUT", "CFG SET MOTOR_OUTPUT 1", f"MOTORTEST {mode} {pwm}"],
            updates={"MOTOR_OUTPUT": "1", "MOTOR": "1", "SPIN_TEST": "1", "SPEED_TUNE": "0"},
            status=f"电机自检已启动：{mode} PWM={pwm}，车辆必须架空。",
        )

    def stop_motor_test(self) -> CommandPlan:
        return CommandPlan(
            commands=["MOTORTEST STOP", "MOTOR 0", "STOP", "CFG SET MOTOR_OUTPUT 0"],
            updates={"MOTOR": "0", "MOTOR_OUTPUT": "0", "SPIN_TEST": "0", "SPEED_TUNE": "0"},
            status="电机自检已停止，电机输出运行期开关已关闭。",
        )

    def stop_spin_test(self) -> CommandPlan:
        return CommandPlan(
            commands=["SPEED 0", "MOTOR 0", "STOP", "CFG SET SPEED_PID 0", "CFG SET MOTOR_OUTPUT 0"],
            updates={"MOTOR": "0", "SPEED_PID": "0", "MOTOR_OUTPUT": "0", "SPIN_TEST": "0", "SPEED_TUNE": "0"},
            status="悬空持续转测试已停止，电机输出运行期开关已关闭。",
        )
