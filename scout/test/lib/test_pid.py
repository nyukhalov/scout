from scout.lib.pid import PID
import unittest


class TestPID(unittest.TestCase):
    def test_default_setpoint_value(self):
        pid = PID(1, 2, 3)
        self.assertEquals(0, pid.setpoint)

    def test_when_process_var_match_setpoint_then_return_0(self):
        setpoint = 42
        pid = PID(1, 2, 3, setpoint)
        self.assertEquals(0, pid.update(setpoint))

    def test_when_gains_are_zero_then_return_0(self):
        setpoint = 42
        pid = PID(0, 0, 0, setpoint)
        self.assertEquals(0, pid.update(setpoint + 100))

    def test_P_gain_usage(self):
        kp = 0.1
        pv = 100
        pid = PID(kp, ki=0, kd=0)
        self.assertEquals(kp * pv, pid.update(pv))

    def test_I_gain_usage(self):
        ki = 0.1
        pv = 100
        pid = PID(kp=0, ki=ki, kd=0)

        # first update always returns 0 because err is not accumulated yet
        self.assertEquals(0, pid.update(pv))
        self.assertEquals(ki * pv, pid.update(pv))

    def test_D_gain_usage(self):
        kd = 0.1
        pv = 100
        pv_diff = 10
        pid = PID(kp=0, ki=0, kd=kd)

        # first update always returns 0 because prev_err is not initialized
        self.assertEquals(0, pid.update(pv))
        self.assertEquals(kd * pv_diff, pid.update(pv + pv_diff))
