import control

class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

    def getKp(self, Kp):
        return self.Kp

    def getKi(self, Ki):
        return self.Ki

    def getKd(self, Kd):
        return self.Kd

    def setKp(self, x):
        self.Kp = x

    def create_tf(self):
        # Returns the transfer function for PID
        num = [self.Kd, self.Kp, self.Ki]
        den = [1, 0]
        return control.tf(num, den, self.dt)

    def create_ss(self):
        value = self.create_tf()
        return control.tf2ss(value)
