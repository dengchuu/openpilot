from selfdrive.controls.lib.pid import PIController
from common.numpy_fast import interp
from cereal import car
from common.realtime import sec_since_boot

_DT_MPC = 0.05
_DT = 0.01

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

  def update(self, active, v_ego, angle_steers, steer_override, CP, VM, path_plan, rcv_times):

    if rcv_times['pathPlan'] != self.prev_path_plan_time:
      self.prev_path_plan_angle = self.path_plan_angle
      self.path_plan_angle = path_plan.angleSteers
      self.prev_path_plan_time= rcv_times['pathPlan']
      self.middle_angle = (self.path_plan_angle + self.prev_path_plan_angle) / 2.0

    if v_ego < 0.3 or not active:
      self.output_steer = 0.0
      self.output_steer1 = 0.0
      self.output_steer2 = 0.0
      self.output_steer3 = 0.0
      self.pid.reset()
    else:
      cur_time = sec_since_boot()
      plan_time = rcv_times['pathPlan']
      dt = min(cur_time - plan_time, _DT_MPC + _DT) + _DT  # no greater than dt mpc + dt, to prevent too high extraps

      self.angle_steers_des = path_plan.angleSteers
      self.angle_steers_des1 = self.middle_angle + path_plan.rateSteers * dt
      self.angle_steers_des2 = self.prev_path_plan_angle + path_plan.rateSteers * dt
      self.angle_steers_des3 = interp(cur_time + 0.025, path_plan.mpcTimes, path_plan.mpcAngles)

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
