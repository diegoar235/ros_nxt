class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-128, 127)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        # Variables internas
        self._prev_error = 0.0
        self._integral = 0.0

    def reset(self):
        """Reinicia las variables internas del PID."""
        self._prev_error = 0.0
        self._integral = 0.0

    def update(self, setpoint, measurement, dt):

        error = setpoint - measurement

        # Términos PID
        p = self.kp * error
        self._integral += error * dt
        i = self.ki * self._integral
        d = self.kd * (error - self._prev_error) / dt if dt > 0 else 0.0

        # PID total
        output = p + i + d

        # Aplicar límites de salida
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)

        # Guardar error previo
        self._prev_error = error

        return output
