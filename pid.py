class PID:
    def __init__(self, Kp, Ki, Kd, out_min=0, out_max=180):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.out_min, self.out_max = out_min, out_max
        self.prev_err = 0.0
        self.int_err  = 0.0

    def update(self, err, dt):
        self.int_err  += err * dt
        der_err        = (err - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err  = err

        out = ( self.Kp*err + self.Ki*self.int_err + self.Kd*der_err )
        # 서보 각도 = 90° + PID 출력
        out = 90 + out
        return max(self.out_min, min(self.out_max, out))
