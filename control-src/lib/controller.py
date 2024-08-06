import time
class PID:
  def __init__(self, kp, ki, kd, setpoint=0, sample_time=0.01, output_limits=(None, None)):
    """
    Initialize the PID controller with given parameters.
    
    :param kp: Proportional gain
    :param ki: Integral gain
    :param kd: Derivative gain
    :param setpoint: Desired value
    :param sample_time: Time interval between updates
    :param output_limits: Tuple with (min, max) output limits
    """
    # Initialize gains
    self.kp = kp
    self.ki = ki
    self.kd = kd
    
    # Initialize setpoint
    self.setpoint = setpoint
    
    # Initialize sample time
    self.sample_time = sample_time
    
    # Initialize output limits
    self.output_limits = output_limits
    
    # Initialize PID components
    self._proportional = 0
    self._integral = 0
    self._derivative = 0
    
    # Initialize error terms
    self._last_error = 0
    self._last_time = time.ticks_us()
      
  def update(self, current_value):
    """
    Update the PID controller with the current value and calculate the new output.
    
    :param current_value: Current measured value
    :return: PID controller output
    """
    # Calculate error
    error = self.setpoint - current_value
    
    # Calculate time difference
    now = time.ticks_us()
    time_difference = time.ticks_diff(now, self._last_time) / 1e5
    
    if time_difference >= self.sample_time:
      # Calculate proportional term
      self._proportional = self.kp * error
      
      # Calculate integral term
      self._integral += self.ki * error * time_difference
      self._integral = self._clamp(self._integral, self.output_limits)
      
      # Calculate derivative term
      self._derivative = 0
      if time_difference > 0:
        self._derivative = self.kd * (error - self._last_error) / time_difference
      
      # Calculate total output
      output_before_clamp = self._proportional + self._integral + self._derivative
      output = self._clamp(output_before_clamp, self.output_limits)
      
      # Adjust integral to prevent windup
      if output != output_before_clamp:
        self._integral -= output - output_before_clamp
      
      # Save last error and time
      self._last_error = error
      #self._last_time = now
      self._last_time = time.ticks_us()
      #print("P: ", self._proportional, " I: ", self._integral, " D: ", self._derivative, " Output: ", output)
      
      return output
    else:
      return None

  def set_setpoint(self, setpoint):
    """
    Set a new setpoint for the PID controller.
    
    :param setpoint: Desired value
    """
    self.setpoint = setpoint

  def set_output_limits(self, output_limits):
    """
    Set new output limits for the PID controller.
    
    :param output_limits: Tuple with (min, max) output limits
    """
    self.output_limits = output_limits

  def set_gains(self, kp, ki, kd):
    """
    Set new gains for the PID controller.
    
    :param kp: Proportional gain
    :param ki: Integral gain
    :param kd: Derivative gain
    """
    self.kp = kp
    self.ki = ki
    self.kd = kd

  def _clamp(self, value, limits):
    """
    Clamp the value within the given limits.
    
    :param value: Value to be clamped
    :param limits: Tuple with (min, max) limits
    :return: Clamped value
    """
    min_limit, max_limit = limits
    if min_limit is not None and value < min_limit:
      return min_limit
    if max_limit is not None and value > max_limit:
      return max_limit
    return value


class PI:
  def __init__(self, kp, ki, setpoint=0, sample_time=0.01, output_limits=(None, None)):
    """
    Initialize the PID controller with given parameters.
    
    :param kp: Proportional gain
    :param ki: Integral gain
    :param setpoint: Desired value
    :param sample_time: Time interval between updates
    :param output_limits: Tuple with (min, max) output limits
    """
    # Initialize gains
    self.kp = kp
    self.ki = ki
    
    # Initialize setpoint
    self.setpoint = setpoint
    
    # Initialize sample time
    self.sample_time = sample_time
    
    # Initialize output limits
    self.output_limits = output_limits
    
    # Initialize PID components
    self._proportional = 0
    self._integral = 0
    
    # Initialize error terms
    self._last_error = 0
    self._last_time = time.ticks_us()
      
  def update(self, current_value):
    """
    Update the PID controller with the current value and calculate the new output.
    
    :param current_value: Current measured value
    :return: PID controller output
    """
    # Calculate error
    error = self.setpoint - current_value
    
    # Calculate time difference
    now = time.ticks_us()
    time_difference = time.ticks_diff(now, self._last_time) / 1e6
    #print(time_difference)
    
    if time_difference >= self.sample_time:
      # Calculate proportional term
      self._proportional = self.kp * error
      
      # Calculate integral term
      self._integral += self.ki * error * time_difference

      # Calculate total output
      output_before_clamp = self._proportional + self._integral
      output = self._clamp(output_before_clamp, self.output_limits)
      
      # Adjust integral to prevent windup
      if output != output_before_clamp:
        self._integral -= output - output_before_clamp
      
      # Save last error and time
      self._last_error = error
      self._last_time = now

      #print("P: ", self._proportional, " I: ", self._integral, " Output: ", output)
      
      return output
    else:
      return None

  def set_setpoint(self, setpoint):
    """
    Set a new setpoint for the PID controller.
    
    :param setpoint: Desired value
    """
    self.setpoint = setpoint

  def set_output_limits(self, output_limits):
    """
    Set new output limits for the PID controller.
    
    :param output_limits: Tuple with (min, max) output limits
    """
    self.output_limits = output_limits

  def set_gains(self, kp, ki):
    """
    Set new gains for the PID controller.
    
    :param kp: Proportional gain
    :param ki: Integral gain
    :param kd: Derivative gain
    """
    self.kp = kp
    self.ki = ki

  def _clamp(self, value, limits):
    """
    Clamp the value within the given limits.
    
    :param value: Value to be clamped
    :param limits: Tuple with (min, max) limits
    :return: Clamped value
    """
    min_limit, max_limit = limits
    if min_limit is not None and value < min_limit:
      return min_limit
    if max_limit is not None and value > max_limit:
      return max_limit
    return value
