import math

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class PID:
    def __init__(self, kp, ki, kd, limit=1.0, i_limit=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit, self.i_limit = limit, i_limit
        self.integral = 0.0
        self.prev = None

    def reset(self):
        self.integral = 0.0
        self.prev = None

    def step(self, error, dt):
        if dt <= 0.0:
            return 0.0
        self.integral = clamp(self.integral + error * dt, -self.i_limit, self.i_limit)
        derivative = 0.0 if self.prev is None else (error - self.prev) / dt
        self.prev = error
        return clamp(self.kp*error + self.ki*self.integral + self.kd*derivative,
                     -self.limit, self.limit)
