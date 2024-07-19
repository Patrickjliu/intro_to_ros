class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, integral_limit=100.0):
        """
        Initialize PID controller with given parameters.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            setpoint (float): Desired setpoint.
            integral_limit (float): Limit for integral term.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral_limit = integral_limit
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, measurement, dt):
        """
        Update PID controller with new measurement and time step.

        Args:
            measurement (float): Current measurement.
            dt (float): Time step in seconds.

        Returns:
            float: Control signal.
        """
        if dt <= 0:
            return 0.0  # Avoid division by zero and unstable behavior

        # Calculate error
        error = self.setpoint - measurement
        
        # Proportional term
        proportional = self.kp * error

        # Integral term with windup guard
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        integral = self.ki * self.integral

        # Derivative term
        derivative = self.kd * (error - self.last_error) / dt
        self.last_error = error

        # Control signal
        return proportional + integral + derivative