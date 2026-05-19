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
            commands=["CFG TIMEOUT 0", "CFG ARM MOTOR_OUTPUT", "CFG SET MOTOR_OUTPUT 1", f"CFG SPEED {speed:.3f}", "START"],
            updates={"MOTOR_OUTPUT": "1", "MOTOR": "1", "SPIN_TEST": "1", "SPEED_TUNE": "0"},
            status="悬空持续转测试已启动：不再发送周期START心跳，串口只保留波形/必要回复。",
        )

    def start_straight_line_test(self, speed: float) -> CommandPlan:
        return CommandPlan(
            commands=[
                "CFG SET PID_SCOPE 0", "CFG SET PID_SCOPE_OVER_HC05 0", "CFG TIMEOUT 0",
                "CFG SET REMOTE_CONTROL 1", "CFG ARM MOTOR_OUTPUT", "CFG SET MOTOR_OUTPUT 1",
                "CFG ARM SPEED_PID", "CFG SET SPEED_PID 1", "MODE TRACKING", f"CFG SPEED {speed:.3f}", "START",
            ],
            updates={
                "PID_SCOPE": "0", "PID_SCOPE_OVER_HC05": "0", "REMOTE_CONTROL": "1",
                "MOTOR_OUTPUT": "1", "SPEED_PID": "1", "MOTOR": "1", "SPIN_TEST": "0", "SPEED_TUNE": "0",
                "SPEED": f"{speed:.3f}",
            },
            status=f"长直线测试已启动：TRACKING 模式，目标速度 {speed:.3f}m/s，上位机临时接管速度；测试结束请点停车。",
        )

    def start_race_tracking(self) -> CommandPlan:
        return CommandPlan(
            commands=[
                "CFG SET PID_SCOPE 0", "CFG SET PID_SCOPE_OVER_HC05 0", "CFG SET REMOTE_CONTROL 0",
                "CFG ARM MOTOR_OUTPUT", "CFG SET MOTOR_OUTPUT 1", "CFG ARM SPEED_PID", "CFG SET SPEED_PID 1", "AUTO",
            ],
            updates={
                "PID_SCOPE": "0", "PID_SCOPE_OVER_HC05": "0", "REMOTE_CONTROL": "0",
                "MOTOR_OUTPUT": "1", "SPEED_PID": "1", "MOTOR": "0", "SPIN_TEST": "0", "SPEED_TUNE": "0",
            },
            status="循迹比赛模式已启动：关闭上位机接管，打开真实自主循迹所需电机输出和速度闭环。",
        )

    def start_speed_pid_tune(self, speed: float, target: str) -> CommandPlan:
        target = target if target in ("LEFT", "RIGHT", "BOTH") else "BOTH"
        scope_cmd = "SCOPE SPEED" if target == "BOTH" else ("SCOPE 1 2" if target == "RIGHT" else "SCOPE 1 1")
        return CommandPlan(
            commands=[
                "CFG SET PID_SCOPE_OVER_HC05 1", "CFG SET PID_SCOPE 1", "CFG TIMEOUT 0",
                "CFG ARM MOTOR_OUTPUT", "CFG SET MOTOR_OUTPUT 1", "CFG ARM SPEED_PID", "CFG SET SPEED_PID 1",
                f"TUNE {target} 0.000", scope_cmd,
            ],
            updates={
                "PID_SCOPE_OVER_HC05": "1", "PID_SCOPE": "1", "MOTOR_OUTPUT": "1",
                "SPEED_PID": "1", "MOTOR": "1", "SPIN_TEST": "1", "SPEED_TUNE": "1",
            },
            status=f"悬空闭环调参准备中：目标={target}，先保持 0 目标稳定一小段时间。",
        )

    def fire_speed_pid_tune(self, speed: float, target: str) -> CommandPlan:
        target = target if target in ("LEFT", "RIGHT", "BOTH") else "BOTH"
        return CommandPlan(
            commands=[f"TUNE {target} {speed:.3f}"],
            updates={"SPEED": f"{speed:.3f}"},
            status=f"悬空闭环调参已启动：目标={target}，速度={speed:.3f}m/s；未选中的轮目标速度为 0。",
        )

    def prepare_step_sample(self, target: str) -> CommandPlan:
        target = target if target in ("LEFT", "RIGHT") else "LEFT"
        scope_cmd = "SCOPE 1 2" if target == "RIGHT" else "SCOPE 1 1"
        return CommandPlan(
            commands=[
                "CFG SET PID_SCOPE_OVER_HC05 1", "CFG SET PID_SCOPE 1", "CFG TIMEOUT 0",
                "CFG ARM MOTOR_OUTPUT", "CFG SET MOTOR_OUTPUT 1", "CFG ARM SPEED_PID", "CFG SET SPEED_PID 1",
                "SCOPE PERIOD 100", scope_cmd, f"TUNE {target} 0.000",
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
            commands=[f"TUNE {target} {speed:.3f}"],
            updates={"SPEED": f"{speed:.3f}"},
            status=f"已发出半自动阶跃：目标={target}，速度={speed:.3f}m/s；采样稳定后点击阶跃分析。",
        )

    def apply_model_recommendation(self, target: str, deadband_pwm: float, pwm_per_mps: float, kp: float, ki: float) -> CommandPlan:
        target = target if target in ("LEFT", "RIGHT", "BOTH") else "LEFT"
        return CommandPlan(
            commands=[f"FF {target} {deadband_pwm:.1f} {pwm_per_mps:.1f}", f"PID {target} {kp:.4g} {ki:.4g} 0"],
            status=f"已下发模型整定建议：{target} 前馈 deadband={deadband_pwm:.1f}, gain={pwm_per_mps:.1f}, PI={kp:.4g}/{ki:.4g}/0。",
        )

    def start_motor_test(self, mode: str, pwm: int) -> CommandPlan:
        return CommandPlan(
            commands=["CFG TIMEOUT 0", "CFG ARM MOTOR_OUTPUT", "CFG SET MOTOR_OUTPUT 1", f"MOTORTEST {mode} {pwm}"],
            updates={"MOTOR_OUTPUT": "1", "MOTOR": "1", "SPIN_TEST": "1", "SPEED_TUNE": "0"},
            status=f"电机自检已启动：{mode} PWM={pwm}，车辆必须架空。",
        )

    def stop_motor_test(self) -> CommandPlan:
        return CommandPlan(
            commands=["MOTORTEST STOP", "CFG SET PID_SCOPE 0", "CFG SET PID_SCOPE_OVER_HC05 0", "CFG SET MOTOR_OUTPUT 0", "CFG TIMEOUT 1000"],
            updates={"MOTOR": "0", "MOTOR_OUTPUT": "0", "PID_SCOPE": "0", "PID_SCOPE_OVER_HC05": "0", "SPIN_TEST": "0", "SPEED_TUNE": "0"},
            status="电机自检已停止，波形流和电机输出运行期开关已关闭。",
        )

    def stop_spin_test(self) -> CommandPlan:
        return CommandPlan(
            commands=["TUNE STOP", "SPEED 0", "STOP", "CFG SET PID_SCOPE 0", "CFG SET PID_SCOPE_OVER_HC05 0", "CFG SET SPEED_PID 0", "CFG SET MOTOR_OUTPUT 0", "CFG TIMEOUT 1000"],
            updates={"MOTOR": "0", "SPEED_PID": "0", "MOTOR_OUTPUT": "0", "PID_SCOPE": "0", "PID_SCOPE_OVER_HC05": "0", "SPIN_TEST": "0", "SPEED_TUNE": "0"},
            status="悬空持续转测试已停止，波形流和电机输出运行期开关已关闭。",
        )

    def enable_flash_storage(self) -> CommandPlan:
        return CommandPlan(
            commands=["CFG SET FLASH_STORAGE 1"],
            updates={"FLASH_STORAGE": "1"},
            status="已请求启用 Flash 存储功能；默认不会自动保存或加载。",
        )

    def query_current_params(self) -> CommandPlan:
        return CommandPlan(
            commands=["PARAMS?"],
            status="已请求读取当前 RAM 参数。",
        )

    def query_flash_storage(self) -> CommandPlan:
        return CommandPlan(
            commands=["STORAGE?"],
            status="已请求读取 Flash 保存区内容。",
        )

    def save_flash_storage(self, section: str = "ALL") -> CommandPlan:
        section = section if section in ("ALL", "PID", "FF", "MAP", "PARAMS") else "ALL"
        return CommandPlan(
            commands=["CFG SET FLASH_STORAGE 1", f"SAVE {section}"],
            updates={"FLASH_STORAGE": "1"},
            status=f"已请求保存 {section} 到 Flash。",
        )

    def load_flash_storage(self) -> CommandPlan:
        return CommandPlan(
            commands=["CFG SET FLASH_STORAGE 1", "LOAD"],
            updates={"FLASH_STORAGE": "1"},
            status="已请求从 Flash 加载 PID 参数和辅助地图。",
        )

    def erase_flash_storage(self, section: str = "ALL") -> CommandPlan:
        section = section if section in ("ALL", "PID", "FF", "MAP", "PARAMS") else "ALL"
        return CommandPlan(
            commands=["CFG SET FLASH_STORAGE 1", f"ERASE {section}"],
            updates={"FLASH_STORAGE": "1"},
            status=f"已请求擦除 Flash 保存区中的 {section}。",
        )
