class PID:

    def __init__(self, Kp, Ki, Kd, lower_limit=-100, higher_limit=100):
        self.__prev_error = 0
        self.__proportional = 0
        self.__integral = 0
        self.__derivative = 0
        self.__Kp = Kp
        self.__Ki = Ki
        self.__Kd = Kd
        self.__lower_limit = lower_limit
        self.__higher_limit = higher_limit

        self.__target = 0

    def __constrain(self, value):
        if value < self.__lower_limit: value = self.__lower_limit
        if value > self.__higher_limit: value = self.__higher_limit
        return value

    def control(self, current, dt):
        dt = dt.to_sec()
        error = float(current - self.__target)
        self.__proportional = error * self.__Kp
        self.__integral += self.__constrain(error * dt * self.__Ki)
        self.__derivative = (error - self.__prev_error) / dt * self.__Kd
        self.__prev_error = error
        return int(self.__constrain(self.__proportional + self.__integral + self.__derivative))

    def break_pid(self):
        self.__proportional = 0
        self.__integral = 0
        self.__derivative = 0

    def set_target(self, target):
        if target != self.__target:
            self.break_pid
        self.__target = target

    def set_gains(self, P, I, D):
        self.__Kp = P
        self.__Ki = I
        self.__Kd = D