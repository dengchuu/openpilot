from selfdrive.controls.lib.pid import PIController
from common.numpy_fast import interp
from cereal import car
from common.realtime import sec_since_boot


def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)


class LatControl(object):
  def __init__(self, CP):
    self.pid = PIController((CP.steerKpBP, CP.steerKpV),
                            (CP.steerKiBP, CP.steerKiV),
                            k_f=CP.steerKf, pos_limit=1.0)
    self.pid1 = PIController((CP.steerKpBP, CP.steerKpV),
                            (CP.steerKiBP, CP.steerKiV),
                            k_f=CP.steerKf, pos_limit=1.0)
    self.pid2 = PIController((CP.steerKpBP, CP.steerKpV),
                            (CP.steerKiBP, CP.steerKiV),
                            k_f=CP.steerKf, pos_limit=1.0)
    self.pid3 = PIController((CP.steerKpBP, CP.steerKpV),
                            (CP.steerKiBP, CP.steerKiV),
                            k_f=CP.steerKf, pos_limit=1.0)
    self.last_cloudlog_t = 0.0
    self.angle_steers_des = 0.
    self.angle_steers_des1 = 0.
    self.angle_steers_des2 = 0.
    self.angle_steers_des3 = 0.
    self.output_steer = 0.
    self.output_steer1 = 0.
    self.output_steer2 = 0.
    self.output_steer3 = 0.
    self.prev_path_plan_time = 0.
    self.prev_path_plan_angle = 0.
    self.path_plan_angle = 0.
    self.calc_rate = 0.

  def reset(self):
    self.pid.reset()

  def update(self, active, v_ego, angle_steers, steer_override, CP, VM, path_plan, path_plan_monoTime):

    if path_plan_monoTime != self.prev_path_plan_time:
      self.prev_path_plan_angle = self.path_plan_angle
      self.path_plan_angle = path_plan.angleSteers
      self.prev_path_plan_time= path_plan_monoTime
      self.calc_rate = (self.path_plan_angle - self.prev_path_plan_angle) / 0.05
      print(self.calc_rate, path_plan.rateSteers)

    if v_ego < 0.3 or not active:
      self.output_steer = 0.0
      self.output_steer1 = 0.0
      self.output_steer2 = 0.0
      self.output_steer3 = 0.0
      self.pid.reset()
    else:
      cur_time = sec_since_boot()
      elapsed_time = cur_time - path_plan_monoTime * 1e-9
      #print(elapsed_time, cur_time, path_plan_monoTime * 1e-9)

      self.angle_steers_des = path_plan.angleSteers
      self.angle_steers_des1 = path_plan.angleSteers + elapsed_time * path_plan.rateSteers
      self.angle_steers_des2 = self.prev_path_plan_angle + elapsed_time * path_plan.rateSteers
      self.angle_steers_des3 = interp(cur_time, path_plan.mpcTimes, path_plan.mpcAngles)

      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      steer_feedforward = self.angle_steers_des   # feedforward desired angle
      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        # TODO: feedforward something based on path_plan.rateSteers
        steer_feedforward -= path_plan.angleOffset   # subtract the offset, since it does not contribute to resistive torque
        steer_feedforward *= v_ego**2  # proportional to realigning tire momentum (~ lateral accel)
      deadzone = 0.0

      self.output_steer = self.pid.update(self.angle_steers_des, angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                     feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)
      self.output_steer1 = self.pid.update(self.angle_steers_des1, angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                     feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)
      self.output_steer2 = self.pid.update(self.angle_steers_des2, angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                     feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)
      self.output_steer3 = self.pid.update(self.angle_steers_des3, angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                     feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)

    self.sat_flag = self.pid.saturated
    return self.output_steer3, float(self.angle_steers_des)
