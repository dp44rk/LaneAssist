import time
import numpy as np

class PIDSteering:
    """
    P  : Proportional-Integral-Derivative (〈x_offset〉 → Δθ_raw)
    I  : In-place anti-windup, output clamp
    P' : Post-EMA + Rate-Limiter   (θ_prev → θ_cmd)
    """
    def __init__(self,
                 kp=0.5, ki=0.0004, kd=1.2,
                 ema_alpha=0.6,          # 0→무필터, 1→완전 새값
                 rate_limit=6,           # deg / frame
                 angle_min=45, angle_max=135):
        # PID state
        self.kp, self.ki, self.kd = kp, ki, kd
        self.int_err   = 0.0
        self.prev_err  = 0.0
        self.prev_ts   = time.time()
        # Post filter state
        self.ema_alpha = ema_alpha
        self.rate_lim  = rate_limit
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle     = 90            # initial steering angle

    # ────────────────────────────────────────────────────────────────
    def reset(self, angle=90):
        self.int_err = self.prev_err = 0.0
        self.prev_ts = time.time()
        self.angle   = angle
    # ────────────────────────────────────────────────────────────────
    def update(self, x_offset_px: float) -> int:
        """call every frame; returns smoothed steering angle [deg]"""
        now = time.time()
        dt  = max(now - self.prev_ts, 1e-3)
        self.prev_ts = now

        # ① PID ------------------------------------------------------
        err = x_offset_px                       # + → 우측 치우침
        self.int_err  += err * dt
        der          = (err - self.prev_err) / dt
        self.prev_err = err

        delta = (self.kp * err +
                 self.ki * self.int_err +
                 self.kd * der)

        raw_angle = 90 + delta                  # 90° = straight

        # ② Post-EMA -------------------------------------------------
        ema = (self.ema_alpha * raw_angle +
               (1 - self.ema_alpha) * self.angle)

        # ③ Rate-Limiter --------------------------------------------
        diff = np.clip(ema - self.angle,
                       -self.rate_lim, self.rate_lim)
        self.angle += diff

        # ④ Clamp to physical limits --------------------------------
        self.angle = int(np.clip(self.angle,
                                 self.angle_min, self.angle_max))
        return self.angle
