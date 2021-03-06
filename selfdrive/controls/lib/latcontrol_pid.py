from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from selfdrive.kegman_conf import kegman_conf
from common.numpy_fast import interp, clip
import numpy as np
from cereal import log
from common.realtime import sec_since_boot
from common.params import Params
import json

class LatControlPID(object):
  def __init__(self, CP):
    kegman_conf(CP)
    self.frame = 0
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0)
    self.angle_steers_des = 0.
    self.total_poly_projection = max(0.0, CP.lateralTuning.pid.polyReactTime + CP.lateralTuning.pid.polyDampTime)
    self.poly_smoothing = max(1.0, CP.lateralTuning.pid.polyDampTime * 100.)
    self.poly_scale = CP.lateralTuning.pid.polyScale
    self.poly_factor = CP.lateralTuning.pid.polyFactor
    self.poly_scale = CP.lateralTuning.pid.polyScale
    self.path_error = 0.0
    self.cur_poly_scale = 0.0
    self.p_poly = [0., 0., 0., 0.]
    self.s_poly = [0., 0., 0., 0.]
    self.p_prob = 0.
    self.damp_angle_steers = 0.
    self.damp_time = 0.1
    self.react_mpc = 0.0
    self.damp_mpc = 0.25
    self.angle_ff_ratio = 0.0
    self.gernbySteer = True
    self.standard_ff_ratio = 0.0
    self.angle_ff_gain = 1.0
    self.rate_ff_gain = CP.lateralTuning.pid.rateFFGain
    self.angle_ff_bp = [[0.5, 5.0],[0.0, 1.0]]
    self.steer_p_scale = CP.lateralTuning.pid.steerPscale
    self.calculate_rate = True
    self.prev_angle_steers = 0.0
    self.rough_steers_rate = 0.0
    self.steer_counter = 1
    self.lane_change_adjustment = 0.0
    self.lane_changing = 0.0
    self.starting_angle = 0.0
    self.half_lane_width = 0.0
    self.steer_counter_prev = 1
    self.params = Params()
    self.prev_override = False
    self.driver_assist_offset = 0.0
    self.driver_assist_hold = False
    self.angle_bias = 0.

    try:
      lateral_params = self.params.get("LateralParams")
      lateral_params = json.loads(lateral_params)
      self.angle_ff_gain = max(1.0, float(lateral_params['angle_ff_gain']))
    except:
      self.angle_ff_gain = 1.0

  def live_tune(self, CP):
    self.frame += 1
    if self.frame % 3600 == 0:
      self.params.put("LateralParams", json.dumps({'angle_ff_gain': self.angle_ff_gain}))
    if self.frame % 300 == 0:
      # live tuning through /data/openpilot/tune.py overrides interface.py settings
      kegman = kegman_conf()
      self.pid._k_i = ([0.], [float(kegman.conf['Ki'])])
      self.pid._k_p = ([0.], [float(kegman.conf['Kp'])])
      self.pid.k_f = (float(kegman.conf['Kf']))
      self.damp_time = (float(kegman.conf['dampTime']))
      self.react_mpc = (float(kegman.conf['reactMPC']))
      self.damp_mpc = (float(kegman.conf['dampMPC']))
      self.total_poly_projection = max(0.0, float(kegman.conf['polyReact']) + float(kegman.conf['polyDamp']))
      self.poly_smoothing = max(1.0, float(kegman.conf['polyDamp']) * 100.)
      self.poly_factor = float(kegman.conf['polyFactor'])

  def get_projected_path_error(self, v_ego, angle_feedforward, angle_steers, live_params, path_plan, VM):
    curv_factor = interp(abs(angle_feedforward), [1.0, 5.0], [0.0, 1.0])
    self.p_poly[3] += (path_plan.pPoly[3] - self.p_poly[3]) / self.poly_smoothing
    self.p_poly[2] += curv_factor * (path_plan.pPoly[2] - self.p_poly[2]) / (self.poly_smoothing * 1.5)
    self.p_poly[1] += curv_factor * (path_plan.pPoly[1] - self.p_poly[1]) / (self.poly_smoothing * 3.0)
    self.p_poly[0] += curv_factor * (path_plan.pPoly[0] - self.p_poly[0]) / (self.poly_smoothing * 4.5)
    self.p_prob += (path_plan.pProb - self. p_prob) / (self.poly_smoothing)
    self.s_poly[1] = float(np.tan(VM.calc_curvature(np.radians(angle_steers - live_params.angleOffset), float(v_ego))))
    x = int(float(v_ego) * self.total_poly_projection * interp(abs(angle_feedforward), [0., 5.], [0.25, 1.0]))
    self.p_pts = np.polyval(self.p_poly, np.arange(0, x))
    self.s_pts = np.polyval(self.s_poly, np.arange(0, x))
    return self.p_prob * (np.sum(self.p_pts) - np.sum(self.s_pts))

  def reset(self):
    self.pid.reset()

  def adjust_angle_gain(self):
    if (self.pid.f > 0) == (self.pid.i > 0) and abs(self.pid.i) >= abs(self.previous_integral):
      if not abs(self.pid.f + self.pid.i + self.pid.p) > 1: self.angle_ff_gain *= 1.0001
    elif self.angle_ff_gain > 1.0:
      self.angle_ff_gain *= 0.9999
    self.previous_integral = self.pid.i

  def update_lane_state(self, angle_steers, driver_opposing_lane, blinkers_on, path_plan):
    if self.lane_changing > 0.0:
      if self.lane_changing > 2.75 or (not blinkers_on and self.lane_changing < 1.0 and abs(path_plan.cPoly[3]) < 0.5 and min(abs(self.starting_angle - angle_steers), abs(self.angle_steers_des - angle_steers)) < 1.5):
        self.lane_changing = 0.0
      elif 2.25 <= self.lane_changing < 2.5 and abs(path_plan.lPoly[3] + path_plan.rPoly[3]) < abs(path_plan.cPoly[3]):
        self.lane_changing = 2.5
      elif 2.0 <= self.lane_changing < 2.25 and (path_plan.lPoly[3] + path_plan.rPoly[3]) * path_plan.cPoly[3] < 0:
        self.lane_changing = 2.25
      elif self.lane_changing < 2.0 and self.half_lane_width < 1.05 * abs(path_plan.lPoly[3] + path_plan.rPoly[3]):
        self.lane_changing = 2.0
      else:
        self.lane_changing = max(self.lane_changing + 0.01, abs(path_plan.lPoly[3] + path_plan.rPoly[3]))
      if blinkers_on:
        self.lane_change_adjustment = 0.0
      else:
        self.lane_change_adjustment = interp(self.lane_changing, [0.0, 1.0, 2.0, 2.25, 2.5, 2.75], [1.0, 0.0, 0.0, 0.1, .2, 1.0])
      #print("%0.2f lane_changing  %0.2f adjustment  %0.2f p_poly   %0.2f avg_poly" % (self.lane_changing, self.lane_change_adjustment, path_plan.cPoly[3], path_plan.lPoly[3] + path_plan.rPoly[3]))
    elif driver_opposing_lane and (blinkers_on or abs(path_plan.cPoly[3]) > 0.5 or min(abs(self.starting_angle - angle_steers), abs(self.angle_steers_des - angle_steers)) > 1.5):
      self.lane_changing = 0.01
    else:
      self.half_lane_width = (path_plan.lPoly[3] - path_plan.rPoly[3]) / 2.
      self.starting_angle = angle_steers
      self.lane_change_adjustment = 1.0

  def update(self, active, v_ego, angle_steers, angle_steers_rate, steer_override, blinkers_on, CP, VM, path_plan, live_params):

    if angle_steers_rate == 0.0 and self.calculate_rate:
      if angle_steers != self.prev_angle_steers:
        self.steer_counter_prev = self.steer_counter
        self.rough_steers_rate = (self.rough_steers_rate + 100.0 * (angle_steers - self.prev_angle_steers) / self.steer_counter_prev) / 2.0
        self.steer_counter = 0.0
      elif self.steer_counter >= self.steer_counter_prev:
        self.rough_steers_rate = (self.steer_counter * self.rough_steers_rate) / (self.steer_counter + 1.0)
      self.steer_counter += 1.0
      angle_steers_rate = self.rough_steers_rate
    else:
      # If non-zero angle_rate is provided, stop calculating angle rate
      self.calculate_rate = False

    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steerAngle = float(angle_steers)
    pid_log.steerRate = float(angle_steers_rate)

    max_bias_change = 0.0002 / (abs(self.angle_bias) + 0.0001)
    self.angle_bias = float(clip(live_params.angleOffset - live_params.angleOffsetAverage, self.angle_bias - max_bias_change, self.angle_bias + max_bias_change))
    self.live_tune(CP)

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      self.lane_changing = 0.0
      self.previous_integral = 0.0
      self.damp_angle_steers= 0.0
      self.damp_rate_steers_des = 0.0
      self.damp_angle_steers_des = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      self.angle_steers_des = path_plan.angleSteers
      if not self.driver_assist_hold:
        self.damp_angle_steers_des += (interp(sec_since_boot() + self.damp_mpc + self.react_mpc, path_plan.mpcTimes, path_plan.mpcAngles) - self.damp_angle_steers_des) / max(1.0, self.damp_mpc * 100.)
        self.damp_rate_steers_des += (interp(sec_since_boot() + self.damp_mpc + self.react_mpc, path_plan.mpcTimes, path_plan.mpcRates) - self.damp_rate_steers_des) / max(1.0, self.damp_mpc * 100.)
        self.damp_angle_steers += (angle_steers - self.angle_bias + self.damp_time * angle_steers_rate - self.damp_angle_steers) / max(1.0, self.damp_time * 100.)
      else:
        self.damp_angle_steers = angle_steers
        self.damp_angle_steers_des = self.damp_angle_steers + self.driver_assist_offset

      if steer_override and abs(self.damp_angle_steers) > abs(self.damp_angle_steers_des) and self.pid.saturated:
        self.damp_angle_steers_des = self.damp_angle_steers

      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      angle_feedforward = float(self.damp_angle_steers_des - path_plan.angleOffset)
      self.angle_ff_ratio = interp(abs(angle_feedforward), self.angle_ff_bp[0], self.angle_ff_bp[1])
      rate_feedforward = (1.0 - self.angle_ff_ratio) * self.rate_ff_gain * self.damp_rate_steers_des
      steer_feedforward = float(v_ego)**2 * (rate_feedforward + angle_feedforward * self.angle_ff_ratio * self.angle_ff_gain)

      if len(self.poly_scale) > 0:
        if abs(self.damp_angle_steers_des) > abs(self.damp_angle_steers):
          self.cur_poly_scale += 0.05 * (interp(abs(self.damp_rate_steers_des), self.poly_scale[0], self.poly_scale[1]) - self.cur_poly_scale)
        else:
          self.cur_poly_scale += 0.05 * (interp(abs(self.damp_rate_steers_des), self.poly_scale[0], self.poly_scale[2]) - self.cur_poly_scale)
      else:
        self.cur_poly_scale = 1.0

      if len(self.steer_p_scale) > 0:
        if abs(self.damp_angle_steers_des) > abs(self.damp_angle_steers):
          p_scale = interp(abs(angle_feedforward), self.steer_p_scale[0], self.steer_p_scale[1])
        else:
          p_scale = interp(abs(angle_feedforward), self.steer_p_scale[0], self.steer_p_scale[2])
      else:
        p_scale = 1.0

      if CP.carName == "honda" and steer_override and not self.prev_override and not self.driver_assist_hold and self.pid.saturated and abs(angle_steers) < abs(self.damp_angle_steers_des) and not blinkers_on:
        self.driver_assist_hold = True
        self.driver_assist_offset = self.damp_angle_steers_des - self.damp_angle_steers
      else:
        self.driver_assist_hold = steer_override and self.driver_assist_hold

      self.path_error = float(v_ego) * float(self.get_projected_path_error(v_ego, angle_feedforward, angle_steers, live_params, path_plan, VM)) * self.poly_factor * self.cur_poly_scale * self.angle_ff_gain

      if self.driver_assist_hold and not steer_override and abs(angle_steers) > abs(self.damp_angle_steers_des):
        #self.angle_bias = 0.0
        driver_opposing_i = False
      elif (steer_override and self.pid.saturated) or self.driver_assist_hold or self.lane_changing > 0.0 or blinkers_on:
        #self.angle_bias = 0.0
        self.path_error = 0.0

      if self.gernbySteer and not steer_override and v_ego > 10.0:
        if abs(angle_steers) > (self.angle_ff_bp[0][1] / 2.0):
          self.adjust_angle_gain()
        else:
          self.previous_integral = self.pid.i

      driver_opposing_i = steer_override and self.pid.i * self.pid.p > 0 and not self.pid.saturated and not self.driver_assist_hold

      deadzone = 0.0
      output_steer = self.pid.update(self.damp_angle_steers_des, self.damp_angle_steers, check_saturation=(v_ego > 10), override=driver_opposing_i,
                                     add_error=float(self.path_error), feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone, p_scale=p_scale)

      driver_opposing_op = steer_override and (angle_steers - self.prev_angle_steers) * output_steer < 0
      self.update_lane_state(angle_steers, driver_opposing_op, blinkers_on, path_plan)
      output_steer *= self.lane_change_adjustment

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.f = float(self.pid.f)
      pid_log.p2 = float(self.pid.p2)
      pid_log.output = float(output_steer)
      pid_log.saturated = bool(self.pid.saturated)
      pid_log.angleFFRatio = self.angle_ff_ratio
      pid_log.angleBias = self.angle_bias

    self.prev_angle_steers = angle_steers
    self.prev_override = steer_override
    self.sat_flag = self.pid.saturated
    return output_steer, float(self.angle_steers_des), pid_log
