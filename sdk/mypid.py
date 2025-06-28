class mypid:
    def __init__(self,kp=1.0,ki=0.0,kd=0.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.integral = 0.0
        self.previous_error = 0.0

    def pid(self,target=0,current_value=0,time_step=1.0):
        error = target - current_value
        self.integral += error*time_step
        derivative = (error - self.previous_error)/time_step
        control = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return control
