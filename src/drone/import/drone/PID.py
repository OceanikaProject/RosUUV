class PID:

    def __init__(self, Kp, Ki, Kd):
        self.prev_error = 0
        self.proportional = 0
        self.integral = 0
        self.derivative = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.target = 0

    @staticmethod
    def constrain(value, lower_limit, higher_limit):
        if value < lower_limit: value = lower_limit
        if value > higher_limit: value = higher_limit
        return value

    def control(self, current, dt):
        dt = dt.to_sec()
        error = float(current - self.target)
        self.proportional = error * self.Kp
        self.integral += PID.constrain(error * dt * self.Ki, -100, 100)
        self.derivative = (error - self.prev_error) / dt * self.Kd
        self.prev_error = error
        return PID.constrain(self.proportional + self.integral + self.derivative, -100, 100)

    def break_pid(self):
        self.proportional = 0
        self.integral = 0
        self.derivative = 0

    def set_target(self, target):
        if target != self.target:
            self.break_pid
        self.target = target

    def set_gains(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D