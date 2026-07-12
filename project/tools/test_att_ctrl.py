#!/usr/bin/env python3
"""Host-side attitude-loop checks mirroring project/code/att_ctrl.c."""

from dataclasses import dataclass, field


MAX_ANGLE = 30.0
MAX_YAW_SPEED = 180.0
MAX_ROLLING_SPEED = 360.0
MAX_CT_VAL = 50.0
MAX_YAW_CT_VAL = 25.0


def limit(value, low, high):
    return max(low, min(high, value))


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    i_max: float
    p_max: float
    d_max: float
    low_pass: float
    out_i: float = 0.0
    out_d: float = 0.0
    pre_error: float = 0.0

    def solve(self, error, dt, saturated=False):
        if dt <= 0.0:
            dt = 0.001
        out_p = self.kp * error
        if not (saturated and error * self.out_i > 0.0):
            self.out_i = limit(self.out_i + self.ki * error * dt, -self.i_max, self.i_max)
        derivative = (error - self.pre_error) / dt
        d_raw = self.kd * derivative
        self.out_d = limit(self.out_d * (1.0 - self.low_pass) + d_raw * self.low_pass,
                           -self.d_max, self.d_max)
        self.pre_error = error
        return limit(out_p + self.out_i + self.out_d, -self.p_max, self.p_max)

    def reset(self):
        self.out_i = 0.0
        self.out_d = 0.0
        self.pre_error = 0.0


@dataclass
class AttCtrl:
    angle_pid: list = field(default_factory=list)
    rate_pid: list = field(default_factory=list)
    exp_rol: float = 0.0
    exp_pit: float = 0.0
    exp_yaw: float = 0.0
    fb_rol: float = 0.0
    fb_pit: float = 0.0
    fb_yaw: float = 0.0
    yaw_err: float = 0.0
    exp_ang_vel: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    fb_ang_vel: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    set_yaw_speed: float = 0.0
    ct_val: list = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def init(self):
        self.angle_pid = [
            PID(4.0, 0.05, 0.2, 10.0, 50.0, 10.0, 0.1),
            PID(4.0, 0.05, 0.2, 10.0, 50.0, 10.0, 0.1),
            PID(2.5, 0.02, 0.05, 10.0, 50.0, 10.0, 0.1),
        ]
        self.rate_pid = [
            PID(15.0, 0.5, 0.8, 50.0, 500.0, 100.0, 0.2),
            PID(15.0, 0.5, 0.8, 50.0, 500.0, 100.0, 0.2),
            PID(8.0, 0.1, 0.2, 50.0, 500.0, 100.0, 0.2),
        ]

    def reset_all(self):
        for pid in self.angle_pid + self.rate_pid:
            pid.reset()

    def outer(self, dt, armed, roll, pitch, yaw, target_roll, target_pitch, target_yaw_rate):
        self.fb_rol = roll
        self.fb_pit = pitch
        self.fb_yaw = yaw
        if not armed:
            self.exp_rol = 0.0
            self.exp_pit = 0.0
            self.set_yaw_speed = 0.0
            self.exp_yaw = self.fb_yaw
            self.reset_all()
            return

        self.exp_rol = limit(target_roll, -MAX_ANGLE, MAX_ANGLE)
        self.exp_pit = limit(target_pitch, -MAX_ANGLE, MAX_ANGLE)

        set_yaw_av_tmp = limit(target_yaw_rate, -MAX_YAW_SPEED, MAX_YAW_SPEED)
        if self.yaw_err > 90.0 and set_yaw_av_tmp > 0.0:
            set_yaw_av_tmp = 0.0
        elif self.yaw_err < -90.0 and set_yaw_av_tmp < 0.0:
            set_yaw_av_tmp = 0.0

        self.set_yaw_speed += limit(set_yaw_av_tmp - self.set_yaw_speed, -30.0, 30.0)
        self.exp_yaw += self.set_yaw_speed * dt
        if self.exp_yaw < -180.0:
            self.exp_yaw += 360.0
        elif self.exp_yaw > 180.0:
            self.exp_yaw -= 360.0

        self.yaw_err = self.exp_yaw - self.fb_yaw
        if self.yaw_err < -180.0:
            self.yaw_err += 360.0
        elif self.yaw_err > 180.0:
            self.yaw_err -= 360.0

        self.exp_ang_vel[0] = limit(self.angle_pid[0].solve(self.exp_rol - self.fb_rol, dt),
                                    -MAX_ROLLING_SPEED, MAX_ROLLING_SPEED)
        self.exp_ang_vel[1] = limit(self.angle_pid[1].solve(self.exp_pit - self.fb_pit, dt),
                                    -MAX_ROLLING_SPEED, MAX_ROLLING_SPEED)
        self.exp_ang_vel[2] = limit(self.angle_pid[2].solve(self.yaw_err, dt),
                                    -MAX_YAW_SPEED, MAX_YAW_SPEED)

    def inner(self, dt, armed, gyro):
        if not armed:
            self.ct_val = [0.0, 0.0, 0.0]
            for pid in self.rate_pid:
                pid.reset()
            return
        self.fb_ang_vel = list(gyro)
        self.ct_val[0] = limit(self.rate_pid[0].solve(self.exp_ang_vel[0] - gyro[0], dt),
                               -MAX_CT_VAL, MAX_CT_VAL)
        self.ct_val[1] = limit(self.rate_pid[1].solve(self.exp_ang_vel[1] - gyro[1], dt),
                               -MAX_CT_VAL, MAX_CT_VAL)
        self.ct_val[2] = limit(self.rate_pid[2].solve(self.exp_ang_vel[2] - gyro[2], dt),
                               -MAX_YAW_CT_VAL, MAX_YAW_CT_VAL)


def assert_close(name, got, expected, eps=1e-4):
    if abs(got - expected) > eps:
        raise AssertionError(f"{name}: got {got:.6f}, expected {expected:.6f}")


def test_locked_tracks_current_yaw():
    ctrl = AttCtrl()
    ctrl.init()
    ctrl.outer(0.01, False, 0.0, 0.0, 123.0, 20.0, -20.0, 60.0)
    ctrl.inner(0.002, False, [10.0, -10.0, 5.0])
    assert_close("locked exp_yaw", ctrl.exp_yaw, 123.0)
    assert_close("locked roll output", ctrl.ct_val[0], 0.0)
    assert_close("locked pitch output", ctrl.ct_val[1], 0.0)
    assert_close("locked yaw output", ctrl.ct_val[2], 0.0)


def test_roll_pitch_target_limit_and_output_sign():
    ctrl = AttCtrl()
    ctrl.init()
    ctrl.outer(0.01, False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    ctrl.outer(0.01, True, 0.0, 0.0, 0.0, 45.0, -45.0, 0.0)
    ctrl.inner(0.002, True, [0.0, 0.0, 0.0])
    assert_close("roll target limit", ctrl.exp_rol, 30.0)
    assert_close("pitch target limit", ctrl.exp_pit, -30.0)
    if not (ctrl.exp_ang_vel[0] > 0.0 and ctrl.exp_ang_vel[1] < 0.0):
        raise AssertionError("roll/pitch expected angular velocity sign is wrong")
    assert_close("roll output saturated", ctrl.ct_val[0], MAX_CT_VAL)
    assert_close("pitch output saturated", ctrl.ct_val[1], -MAX_CT_VAL)


def test_yaw_wrap_shortest_path():
    ctrl = AttCtrl()
    ctrl.init()
    ctrl.outer(0.01, False, 0.0, 0.0, 179.0, 0.0, 0.0, 0.0)
    ctrl.outer(0.01, True, 0.0, 0.0, -179.0, 0.0, 0.0, 0.0)
    assert_close("yaw wrap error", ctrl.yaw_err, -2.0)
    if not ctrl.exp_ang_vel[2] < 0.0:
        raise AssertionError("yaw expected angular velocity should take the short negative path")


def test_yaw_rate_smoothing():
    ctrl = AttCtrl()
    ctrl.init()
    ctrl.outer(0.01, False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    ctrl.outer(0.01, True, 0.0, 0.0, 0.0, 0.0, 0.0, 180.0)
    assert_close("first yaw speed step", ctrl.set_yaw_speed, 30.0)
    ctrl.outer(0.01, True, 0.0, 0.0, 0.0, 0.0, 0.0, 180.0)
    assert_close("second yaw speed step", ctrl.set_yaw_speed, 60.0)


def main():
    tests = [
        test_locked_tracks_current_yaw,
        test_roll_pitch_target_limit_and_output_sign,
        test_yaw_wrap_shortest_path,
        test_yaw_rate_smoothing,
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print("att_ctrl host checks passed")


if __name__ == "__main__":
    main()
