class mypid:
  def __init__(self, kp,ki,kd):
    self.kp = kp
    self.kd = kd
    self.ki = ki
    self.integral = 0.0
    self.previous_error = 0.0

    def control(target=0, current_value=0):
      