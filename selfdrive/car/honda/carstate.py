from common.numpy_fast import interp
from common.kalman.simple_kalman import KF1D
from selfdrive.can.parser import CANParser, CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.honda.values import CAR, DBC, STEER_THRESHOLD, SPEED_FACTOR

def parse_gear_shifter(gear, vals):

  val_to_capnp = {'P': 'park', 'R': 'reverse', 'N': 'neutral',
                  'D': 'drive', 'S': 'sport', 'L': 'low'}
  try:
    return val_to_capnp[vals[gear]]
  except KeyError:
    return "unknown"


def calc_cruise_offset(offset, speed):
  # euristic formula so that speed is controlled to ~ 0.3m/s below pid_speed
  # constraints to solve for _K0, _K1, _K2 are:
  # - speed = 0m/s, out = -0.3
  # - speed = 34m/s, offset = 20, out = -0.25
  # - speed = 34m/s, offset = -2.5, out = -1.8
  _K0 = -0.3
  _K1 = -0.01879
  _K2 = 0.01013
  return min(_K0 + _K1 * speed + _K2 * speed * offset, 0.)


def get_can_signals(CP):
# this function generates lists for signal, messages and initial values
  signals = [
      ("XMISSION_SPEED", "ENGINE_DATA", 0),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
      ("STEER_ANGLE", "STEERING_SENSORS", 0),
      ("STEER_ANGLE_RATE", "STEERING_SENSORS", 0),
      ("STEER_ANGLE_OFFSET", "STEERING_SENSORS", 0),
      ("STEER_TORQUE_SENSOR", "STEER_STATUS", 0),
      ("LEFT_BLINKER", "SCM_FEEDBACK", 0),
      ("RIGHT_BLINKER", "SCM_FEEDBACK", 0),
      ("GEAR", "GEARBOX", 0),
      ("BRAKE_ERROR_1", "STANDSTILL", 1),
      ("BRAKE_ERROR_2", "STANDSTILL", 1),
      ("SEATBELT_DRIVER_LAMP", "SEATBELT_STATUS", 1),
      ("SEATBELT_DRIVER_LATCHED", "SEATBELT_STATUS", 0),
      ("BRAKE_PRESSED", "POWERTRAIN_DATA", 0),
      ("BRAKE_SWITCH", "POWERTRAIN_DATA", 0),
      ("CRUISE_BUTTONS", "SCM_BUTTONS", 0),
      ("ESP_DISABLED", "VSA_STATUS", 1),
      ("HUD_LEAD", "ACC_HUD", 0),
      ("USER_BRAKE", "VSA_STATUS", 0),
      ("STEER_STATUS", "STEER_STATUS", 5),
      ("GEAR_SHIFTER", "GEARBOX", 0),
      ("PEDAL_GAS", "POWERTRAIN_DATA", 0),
      ("CRUISE_SETTING", "SCM_BUTTONS", 0),
      ("ACC_STATUS", "POWERTRAIN_DATA", 0),
  ]

  checks = [
      ("ENGINE_DATA", 100),
      ("WHEEL_SPEEDS", 50),
      ("STEERING_SENSORS", 100),
      ("SCM_FEEDBACK", 10),
      ("GEARBOX", 100),
      ("STANDSTILL", 50),
      ("SEATBELT_STATUS", 10),
      ("CRUISE", 10),
      ("POWERTRAIN_DATA", 100),
      ("VSA_STATUS", 50),
      ("SCM_BUTTONS", 25),
  ]

  if CP.radarOffCan:
    # Civic is only bosch to use the same brake message as other hondas.
    if CP.carFingerprint not in (CAR.ACCORDH, CAR.CIVIC_HATCH):
      signals += [("BRAKE_PRESSED", "BRAKE_MODULE", 0)]
      checks += [("BRAKE_MODULE", 50)]
    signals += [("CAR_GAS", "GAS_PEDAL_2", 0),
                ("MAIN_ON", "SCM_FEEDBACK", 0),
                ("EPB_STATE", "EPB_STATUS", 0),
                ("BRAKE_HOLD_ACTIVE", "VSA_STATUS", 0),
                ("CRUISE_SPEED", "ACC_HUD", 0),
                ("PARM_1", "CUR_LANE_LEFT_1", 2),
                ("PARM_2", "CUR_LANE_LEFT_1", 1),
                ("PARM_3", "CUR_LANE_LEFT_1", 1),
                ("PARM_4", "CUR_LANE_LEFT_1", 2),
                ("PARM_5", "CUR_LANE_LEFT_1", 1),
                ("PARM_6", "CUR_LANE_LEFT_2", 1),
                ("PARM_7", "CUR_LANE_LEFT_2", 1),
                ("PARM_8", "CUR_LANE_LEFT_2", 1),
                ("PARM_9", "CUR_LANE_LEFT_2", 1),
                ("PARM_10", "CUR_LANE_LEFT_2", 1),
                ("PARM_1", "CUR_LANE_RIGHT_1", 2),
                ("PARM_2", "CUR_LANE_RIGHT_1", 1),
                ("PARM_3", "CUR_LANE_RIGHT_1", 1),
                ("PARM_4", "CUR_LANE_RIGHT_1", 2),
                ("PARM_5", "CUR_LANE_RIGHT_1", 1),
                ("PARM_6", "CUR_LANE_RIGHT_2", 1),
                ("PARM_7", "CUR_LANE_RIGHT_2", 1),
                ("PARM_8", "CUR_LANE_RIGHT_2", 1),
                ("PARM_9", "CUR_LANE_RIGHT_2", 1),
                ("PARM_10", "CUR_LANE_RIGHT_2", 1),
                ("PARM_1", "ADJ_LANE_LEFT_1", 2),
                ("PARM_2", "ADJ_LANE_LEFT_1", 1),
                ("PARM_3", "ADJ_LANE_LEFT_1", 1),
                ("PARM_4", "ADJ_LANE_LEFT_1", 2),
                ("PARM_5", "ADJ_LANE_LEFT_1", 1),
                ("PARM_6", "ADJ_LANE_LEFT_2", 1),
                ("PARM_7", "ADJ_LANE_LEFT_2", 1),
                ("PARM_8", "ADJ_LANE_LEFT_2", 1),
                ("PARM_9", "ADJ_LANE_LEFT_2", 1),
                ("PARM_10", "ADJ_LANE_LEFT_2", 1),
                ("PARM_1", "ADJ_LANE_RIGHT_1", 2),
                ("PARM_2", "ADJ_LANE_RIGHT_1", 1),
                ("PARM_3", "ADJ_LANE_RIGHT_1", 1),
                ("PARM_4", "ADJ_LANE_RIGHT_1", 2),
                ("PARM_5", "ADJ_LANE_RIGHT_1", 1),
                ("PARM_6", "ADJ_LANE_RIGHT_2", 1),
                ("PARM_7", "ADJ_LANE_RIGHT_2", 1),
                ("PARM_8", "ADJ_LANE_RIGHT_2", 1),
                ("PARM_9", "ADJ_LANE_RIGHT_2", 1),
                ("PARM_10", "ADJ_LANE_RIGHT_2", 1),
                ("GERNBY1","RADAR_HUD", 0),
                ("GERNBY2","RADAR_HUD", 0),
                ("GERNBY3","RADAR_HUD", 0),
                ("GERNBY4","RADAR_HUD", 0),
                ("GERNBY5","RADAR_HUD", 0),
                ("GERNBY6","RADAR_HUD", 0),
                ("CMBS_OFF","RADAR_HUD", 0),
                ("LEAD_DISTANCE","RADAR_HUD", 0),
                ("RESUME_INSTRUCTION","RADAR_HUD", 0),
                ("SOLID_LANES","LKAS_HUD", 0),
                ("DASHED_LANES","LKAS_HUD", 0),
                ("STEERING_REQUIRED","LKAS_HUD", 0),
                ("GERNBY1","LKAS_HUD", 0),
                ("GERNBY2","LKAS_HUD", 0),
                ("GERNBY3","LKAS_HUD", 0),
                ("GERNBY4","LKAS_HUD", 0),
                ("LKAS_PROBLEM","LKAS_HUD", 0),
                ("LKAS_OFF","LKAS_HUD", 0),
                ("LDW_RIGHT","LKAS_HUD", 0),
                #("LDW_LEFT","LKAS_HUD", 0),
                ("BEEP","LKAS_HUD", 0),
                ("LDW_ON","LKAS_HUD", 0),
                ("LDW_OFF","LKAS_HUD", 0),
                ("CLEAN_WINDSHIELD","LKAS_HUD", 0),
                ("DTC","LKAS_HUD", 0),
                ("CAM_TEMP_HIGH","LKAS_HUD", 0),
                ("GERNBY1","CAMERA_MESSAGES", 0),
                ("GERNBY2","CAMERA_MESSAGES", 0),
                ("STEER_TORQUE_REQUEST", "STEERING_CONTROL", 0),
                ("GERNBY1", "STEERING_CONTROL", 0),
                ("GERNBY2", "STEERING_CONTROL", 0),
                ("STEER_TORQUE", "STEERING_CONTROL",0),
                ("SPEED_LIMIT", "CAMERA_MESSAGES", 0)]
    checks += [("GAS_PEDAL_2", 100),
                ("CUR_LANE_LEFT_1", 100),
                ("CUR_LANE_LEFT_2", 100),
                ("CUR_LANE_RIGHT_1", 100),
                ("CUR_LANE_RIGHT_2", 100),
                ("ADJ_LANE_LEFT_1", 52),
                ("ADJ_LANE_LEFT_2", 52),
                ("ADJ_LANE_RIGHT_1", 52),
                ("ADJ_LANE_RIGHT_2", 52),
                ("STEERING_CONTROL", 100),
                ("RADAR_HUD", 10),
                ("LKAS_HUD", 10),
                ("CAMERA_MESSAGES", 3)]
  else:
    # Nidec signals.
    signals += [("CRUISE_SPEED_PCM", "CRUISE", 0),
                ("CRUISE_SPEED_OFFSET", "CRUISE_PARAMS", 0)]
    checks += [("CRUISE_PARAMS", 50)]

  if CP.carFingerprint in (CAR.ACCORD, CAR.ACCORD_15, CAR.ACCORDH):
    signals += [("DRIVERS_DOOR_OPEN", "SCM_FEEDBACK", 1)]
  else:
    signals += [("DOOR_OPEN_FL", "DOORS_STATUS", 1),
                ("DOOR_OPEN_FR", "DOORS_STATUS", 1),
                ("DOOR_OPEN_RL", "DOORS_STATUS", 1),
                ("DOOR_OPEN_RR", "DOORS_STATUS", 1),
                ("WHEELS_MOVING", "STANDSTILL", 1)]
    checks += [("DOORS_STATUS", 3)]

  if CP.carFingerprint == CAR.CIVIC:
    signals += [("CAR_GAS", "GAS_PEDAL_2", 0),
                ("MAIN_ON", "SCM_FEEDBACK", 0),
                ("EPB_STATE", "EPB_STATUS", 0),
                ("BRAKE_HOLD_ACTIVE", "VSA_STATUS", 0)]
  elif CP.carFingerprint == CAR.ACURA_ILX:
    signals += [("CAR_GAS", "GAS_PEDAL_2", 0),
                ("MAIN_ON", "SCM_BUTTONS", 0)]
  elif CP.carFingerprint in (CAR.CRV, CAR.ACURA_RDX, CAR.PILOT_2019, CAR.RIDGELINE):
    signals += [("MAIN_ON", "SCM_BUTTONS", 0)]
  elif CP.carFingerprint == CAR.ODYSSEY:
    signals += [("MAIN_ON", "SCM_FEEDBACK", 0),
                ("EPB_STATE", "EPB_STATUS", 0),
                ("BRAKE_HOLD_ACTIVE", "VSA_STATUS", 0)]
    checks += [("EPB_STATUS", 50)]
  elif CP.carFingerprint == CAR.PILOT:
    signals += [("MAIN_ON", "SCM_BUTTONS", 0),
                ("CAR_GAS", "GAS_PEDAL_2", 0)]

  # add gas interceptor reading if we are using it
  if CP.enableGasInterceptor:
    signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR", 0))
    checks.append(("GAS_SENSOR", 50))

  return signals, checks


def get_can_parser(CP):
  signals, checks = get_can_signals(CP)
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)


class CarState(object):
  def __init__(self, CP):
    self.CP = CP
    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = self.can_define.dv["GEARBOX"]["GEAR_SHIFTER"]

    self.user_gas, self.user_gas_pressed = 0., 0
    self.brake_switch_prev = 0
    self.brake_switch_ts = 0
    self.zero_lane_counter = 0
    self.lane11 = 0.
    self.lane12 = 0.
    self.lane21 = 0.
    self.lane22 = 0.
    self.lane31 = 0.
    self.lane32 = 0.
    self.lane41 = 0.
    self.lane42 = 0.
    self.lane51 = 0.
    self.lane52 = 0.
    self.lane61 = 0.
    self.lane62 = 0.
    self.lane71 = 0.
    self.lane72 = 0.
    self.lane81 = 0.
    self.lane82 = 0.
    self.cruise_buttons = 0
    self.cruise_setting = 0
    self.v_cruise_pcm_prev = 0
    self.blinker_on = 0
    self.speed_limit = 0
    self.speed_limit_prev = 0
    self.new_cruise_target_speed = 0

    self.left_blinker_on = 0
    self.right_blinker_on = 0

    self.stopped = 0

    # vEgo kalman filter
    dt = 0.01
    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, dt], [0.0, 1.0]],
                         C=[[1.0, 0.0]],
                         K=[[0.12287673], [0.29666309]])
    self.v_ego = 0.0

  def update(self, cp):

    # copy can_valid
    self.can_valid = cp.can_valid

    # car params
    v_weight_v = [0., 1.]  # don't trust smooth speed at low values to avoid premature zero snapping
    v_weight_bp = [1., 6.]   # smooth blending, below ~0.6m/s the smooth speed snaps to zero

    # update prevs, update must run once per loop
    self.prev_cruise_buttons = self.cruise_buttons
    self.prev_cruise_setting = self.cruise_setting
    self.prev_blinker_on = self.blinker_on

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on

    # ******************* parse out can *******************

    if self.CP.carFingerprint in (CAR.ACCORD, CAR.ACCORD_15, CAR.ACCORDH): # TODO: find wheels moving bit in dbc
      self.standstill = cp.vl["ENGINE_DATA"]['XMISSION_SPEED'] < 0.1
      self.door_all_closed = not cp.vl["SCM_FEEDBACK"]['DRIVERS_DOOR_OPEN']
    else:
      self.standstill = not cp.vl["STANDSTILL"]['WHEELS_MOVING']
      self.door_all_closed = not any([cp.vl["DOORS_STATUS"]['DOOR_OPEN_FL'], cp.vl["DOORS_STATUS"]['DOOR_OPEN_FR'],
                                      cp.vl["DOORS_STATUS"]['DOOR_OPEN_RL'], cp.vl["DOORS_STATUS"]['DOOR_OPEN_RR']])
    self.seatbelt = not cp.vl["SEATBELT_STATUS"]['SEATBELT_DRIVER_LAMP'] and cp.vl["SEATBELT_STATUS"]['SEATBELT_DRIVER_LATCHED']

    # 2 = temporary; 3 = TBD; 4 = temporary, hit a bump; 5 = (permanent); 6 = temporary; 7 = (permanent)
    # TODO: Use values from DBC to parse this field
    self.steer_error = cp.vl["STEER_STATUS"]['STEER_STATUS'] not in [0, 2, 3, 4, 6]
    self.steer_not_allowed = cp.vl["STEER_STATUS"]['STEER_STATUS'] != 0
    self.steer_warning = cp.vl["STEER_STATUS"]['STEER_STATUS'] not in [0, 3]   # 3 is low speed lockout, not worth a warning
    self.brake_error = cp.vl["STANDSTILL"]['BRAKE_ERROR_1'] or cp.vl["STANDSTILL"]['BRAKE_ERROR_2']
    self.esp_disabled = cp.vl["VSA_STATUS"]['ESP_DISABLED']

    # calc best v_ego estimate, by averaging two opposite corners
    speed_factor = SPEED_FACTOR[self.CP.carFingerprint]
    self.v_wheel_fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel = (self.v_wheel_fl+self.v_wheel_fr+self.v_wheel_rl+self.v_wheel_rr)/4.

    # blend in transmission speed at low speed, since it has more low speed accuracy
    self.v_weight = interp(self.v_wheel, v_weight_bp, v_weight_v)
    speed = (1. - self.v_weight) * cp.vl["ENGINE_DATA"]['XMISSION_SPEED'] * CV.KPH_TO_MS * speed_factor + \
      self.v_weight * self.v_wheel

    if abs(speed - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_x = [[speed], [0.0]]

    self.v_ego_raw = speed
    v_ego_x = self.v_ego_kf.update(speed)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])

    # this is a hack for the interceptor. This is now only used in the simulation
    # TODO: Replace tests by toyota so this can go away
    if self.CP.enableGasInterceptor:
      self.user_gas = cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS']
      self.user_gas_pressed = self.user_gas > 0 # this works because interceptor read < 0 when pedal position is 0. Once calibrated, this will change

    self.gear = 0 if self.CP.carFingerprint == CAR.CIVIC else cp.vl["GEARBOX"]['GEAR']
    self.angle_steers = cp.vl["STEERING_SENSORS"]['STEER_ANGLE']
    self.angle_steers_rate = cp.vl["STEERING_SENSORS"]['STEER_ANGLE_RATE']

    self.cruise_setting = cp.vl["SCM_BUTTONS"]['CRUISE_SETTING']
    self.cruise_buttons = cp.vl["SCM_BUTTONS"]['CRUISE_BUTTONS']

    self.blinker_on = cp.vl["SCM_FEEDBACK"]['LEFT_BLINKER'] or cp.vl["SCM_FEEDBACK"]['RIGHT_BLINKER']
    self.left_blinker_on = cp.vl["SCM_FEEDBACK"]['LEFT_BLINKER']
    self.right_blinker_on = cp.vl["SCM_FEEDBACK"]['RIGHT_BLINKER']

    if self.CP.carFingerprint in (CAR.CIVIC, CAR.ODYSSEY, CAR.CRV_5G, CAR.ACCORD, CAR.ACCORD_15, CAR.ACCORDH, CAR.CIVIC_HATCH):
      self.park_brake = cp.vl["EPB_STATUS"]['EPB_STATE'] != 0
      self.brake_hold = cp.vl["VSA_STATUS"]['BRAKE_HOLD_ACTIVE']
      self.main_on = cp.vl["SCM_FEEDBACK"]['MAIN_ON']

      self.lane11 = cp.vl["CUR_LANE_LEFT_1"]["PARM_1"]
      self.lane12 = cp.vl["CUR_LANE_LEFT_1"]["PARM_2"]
      self.lane13 = cp.vl["CUR_LANE_LEFT_1"]["PARM_3"]
      self.lane14 = cp.vl["CUR_LANE_LEFT_1"]["PARM_4"]
      self.lane15 = cp.vl["CUR_LANE_LEFT_1"]["PARM_5"]
      self.lane16 = cp.vl["CUR_LANE_LEFT_2"]["PARM_6"]
      self.lane17 = cp.vl["CUR_LANE_LEFT_2"]["PARM_7"]
      self.lane18 = cp.vl["CUR_LANE_LEFT_2"]["PARM_8"]
      self.lane19 = cp.vl["CUR_LANE_LEFT_2"]["PARM_9"]
      self.lane1A = cp.vl["CUR_LANE_LEFT_2"]["PARM_10"]
      self.lane31 = cp.vl["CUR_LANE_RIGHT_1"]["PARM_1"]
      self.lane32 = cp.vl["CUR_LANE_RIGHT_1"]["PARM_2"]
      self.lane33 = cp.vl["CUR_LANE_RIGHT_1"]["PARM_3"]
      self.lane34 = cp.vl["CUR_LANE_RIGHT_1"]["PARM_4"]
      self.lane35 = cp.vl["CUR_LANE_RIGHT_1"]["PARM_5"]
      self.lane36 = cp.vl["CUR_LANE_RIGHT_2"]["PARM_6"]
      self.lane37 = cp.vl["CUR_LANE_RIGHT_2"]["PARM_7"]
      self.lane38 = cp.vl["CUR_LANE_RIGHT_2"]["PARM_8"]
      self.lane39 = cp.vl["CUR_LANE_RIGHT_2"]["PARM_9"]
      self.lane3A = cp.vl["CUR_LANE_RIGHT_2"]["PARM_10"]
      self.lane51 = cp.vl["ADJ_LANE_LEFT_1"]["PARM_1"]
      self.lane52 = cp.vl["ADJ_LANE_LEFT_1"]["PARM_2"]
      self.lane53 = cp.vl["ADJ_LANE_LEFT_1"]["PARM_3"]
      self.lane54 = cp.vl["ADJ_LANE_LEFT_1"]["PARM_4"]
      self.lane55 = cp.vl["ADJ_LANE_LEFT_1"]["PARM_5"]
      self.lane56 = cp.vl["ADJ_LANE_LEFT_2"]["PARM_6"]
      self.lane57 = cp.vl["ADJ_LANE_LEFT_2"]["PARM_7"]
      self.lane58 = cp.vl["ADJ_LANE_LEFT_2"]["PARM_8"]
      self.lane59 = cp.vl["ADJ_LANE_LEFT_2"]["PARM_9"]
      self.lane5A = cp.vl["ADJ_LANE_LEFT_2"]["PARM_10"]
      self.lane71 = cp.vl["ADJ_LANE_RIGHT_1"]["PARM_1"]
      self.lane72 = cp.vl["ADJ_LANE_RIGHT_1"]["PARM_2"]
      self.lane73 = cp.vl["ADJ_LANE_RIGHT_1"]["PARM_3"]
      self.lane74 = cp.vl["ADJ_LANE_RIGHT_1"]["PARM_4"]
      self.lane75 = cp.vl["ADJ_LANE_RIGHT_1"]["PARM_5"]
      self.lane76 = cp.vl["ADJ_LANE_RIGHT_2"]["PARM_6"]
      self.lane77 = cp.vl["ADJ_LANE_RIGHT_2"]["PARM_7"]
      self.lane78 = cp.vl["ADJ_LANE_RIGHT_2"]["PARM_8"]
      self.lane79 = cp.vl["ADJ_LANE_RIGHT_2"]["PARM_9"]
      self.lane7A = cp.vl["ADJ_LANE_RIGHT_2"]["PARM_10"]
      self.steer_offset = cp.vl['STEERING_SENSORS']['STEER_ANGLE_OFFSET']

      self.total_lane_confidence = (self.lane14 + self.lane34 + self.lane54 + self.lane74) 
      if self.total_lane_confidence > 0:
        self.stock_lane_center = (((self.lane11 * self.lane14) + (self.lane31 * self.lane34) + (self.lane51 * self.lane54) + (self.lane71 * self.lane74)) / self.total_lane_confidence)
        self.stock_lane_curvature = (((self.lane17 * self.lane14) + (self.lane37 * self.lane34) + (self.lane57 * self.lane54) + (self.lane77 * self.lane74)) / self.total_lane_confidence)
      else:
        self.stock_lane_center = 0
        self.stock_lane_curvature = 0

      self.stock_steer_request = cp.vl["STEERING_CONTROL"]["STEER_TORQUE_REQUEST"]
      self.stock_steer_set_me_x00 = cp.vl["STEERING_CONTROL"]["GERNBY1"]
      self.stock_steer_set_me_x00_2 = cp.vl["STEERING_CONTROL"]["GERNBY2"]
      self.stock_steer_steer_torque = cp.vl["STEERING_CONTROL"]["STEER_TORQUE"]
      self.lkas_hud_solid_lanes = cp.vl["LKAS_HUD"]["SOLID_LANES"]
      self.lkas_hud_steering_required = cp.vl["LKAS_HUD"]["STEERING_REQUIRED"]
      self.lkas_hud_GERNBY4 = cp.vl["LKAS_HUD"]["GERNBY4"]
      self.lkas_hud_GERNBY3 = cp.vl["LKAS_HUD"]["GERNBY3"]
      self.lkas_hud_dashed_lanes = cp.vl["LKAS_HUD"]["DASHED_LANES"]      
      self.lkas_hud_GERNBY1 = cp.vl["LKAS_HUD"]["GERNBY1"]      
      self.lkas_hud_GERNBY2 = cp.vl["LKAS_HUD"]["GERNBY2"]      
      self.lkas_hud_LKAS_PROBLEM = cp.vl["LKAS_HUD"]["LKAS_PROBLEM"]      
      self.lkas_hud_LKAS_OFF = cp.vl["LKAS_HUD"]["LKAS_OFF"]      
      self.lkas_hud_LDW_RIGHT = cp.vl["LKAS_HUD"]["LDW_RIGHT"]      
      #self.lkas_hud_LDW_LEFT = cp.vl["LKAS_HUD"]["LDW_LEFT"]      
      self.lkas_hud_BEEP = cp.vl["LKAS_HUD"]["BEEP"]      
      self.lkas_hud_LDW_ON = cp.vl["LKAS_HUD"]["LDW_ON"]      
      self.lkas_hud_LDW_OFF = cp.vl["LKAS_HUD"]["LDW_OFF"]      
      self.lkas_hud_CLEAN_WINDSHIELD = cp.vl["LKAS_HUD"]["CLEAN_WINDSHIELD"]      
      self.lkas_hud_DTC = cp.vl["LKAS_HUD"]["DTC"]      
      self.lkas_hud_CAM_TEMP_HIGH = cp.vl["LKAS_HUD"]["CAM_TEMP_HIGH"]      
      self.radar_hud_gernby1 = cp.vl["RADAR_HUD"]['GERNBY1']
      self.radar_hud_gernby2 = cp.vl["RADAR_HUD"]['GERNBY2']
      self.radar_hud_gernby3 = cp.vl["RADAR_HUD"]['GERNBY3']
      self.radar_hud_gernby4 = cp.vl["RADAR_HUD"]['GERNBY4']
      self.radar_hud_gernby5 = cp.vl["RADAR_HUD"]['GERNBY5']
      self.radar_hud_lead_distance = cp.vl["RADAR_HUD"]['LEAD_DISTANCE']
      self.radar_hud_gernby6 = cp.vl["RADAR_HUD"]['GERNBY6']
      self.radar_hud_CMBS_OFF = cp.vl["RADAR_HUD"]['CMBS_OFF']
      self.radar_hud_RESUME_INSTRUCTION = cp.vl["RADAR_HUD"]['RESUME_INSTRUCTION']

      #  self.zero_lane_counter = 0
      #else:  
      #  self.zero_lane_counter += 1
    else:
      self.park_brake = 0  # TODO
      self.brake_hold = 0  # TODO
      self.main_on = cp.vl["SCM_BUTTONS"]['MAIN_ON']

    can_gear_shifter = int(cp.vl["GEARBOX"]['GEAR_SHIFTER'])
    self.gear_shifter = parse_gear_shifter(can_gear_shifter, self.shifter_values)

    self.pedal_gas = cp.vl["POWERTRAIN_DATA"]['PEDAL_GAS']
    # crv doesn't include cruise control
    if self.CP.carFingerprint in (CAR.CRV, CAR.ODYSSEY, CAR.ACURA_RDX, CAR.RIDGELINE, CAR.PILOT_2019):
      self.car_gas = self.pedal_gas
    else:
      self.car_gas = cp.vl["GAS_PEDAL_2"]['CAR_GAS']

    self.steer_torque_driver = cp.vl["STEER_STATUS"]['STEER_TORQUE_SENSOR']
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD[self.CP.carFingerprint]

    self.brake_switch = cp.vl["POWERTRAIN_DATA"]['BRAKE_SWITCH']

    if self.CP.radarOffCan:
      self.stopped = cp.vl["ACC_HUD"]['CRUISE_SPEED'] == 252.
      self.cruise_speed_offset = calc_cruise_offset(0, self.v_ego)
      if self.CP.carFingerprint in (CAR.CIVIC_HATCH, CAR.ACCORDH):
        self.brake_switch = cp.vl["POWERTRAIN_DATA"]['BRAKE_SWITCH']
        self.brake_pressed = cp.vl["POWERTRAIN_DATA"]['BRAKE_PRESSED'] or \
                          (self.brake_switch and self.brake_switch_prev and \
                          cp.ts["POWERTRAIN_DATA"]['BRAKE_SWITCH'] != self.brake_switch_ts)
        self.brake_switch_prev = self.brake_switch
        self.brake_switch_ts = cp.ts["POWERTRAIN_DATA"]['BRAKE_SWITCH']
      else:
        self.brake_pressed = cp.vl["BRAKE_MODULE"]['BRAKE_PRESSED']
      # On set, cruise set speed pulses between 254~255 and the set speed prev is set to avoid this.
      self.v_cruise_pcm = self.v_cruise_pcm_prev if cp.vl["ACC_HUD"]['CRUISE_SPEED'] > 160.0 else cp.vl["ACC_HUD"]['CRUISE_SPEED']
      self.v_cruise_pcm_prev = self.v_cruise_pcm
      temp_speed_limit = max(20, cp.vl["CAMERA_MESSAGES"]['SPEED_LIMIT'])
      if self.v_cruise_pcm > 0 and temp_speed_limit > 20 and temp_speed_limit != self.speed_limit:
        print (self.v_cruise_pcm, temp_speed_limit)
        self.new_cruise_target_speed = int(max(temp_speed_limit, min(1.1 * temp_speed_limit, self.v_cruise_pcm)))
        self.speed_limit = temp_speed_limit
    else:
      self.brake_switch = cp.vl["POWERTRAIN_DATA"]['BRAKE_SWITCH']
      self.cruise_speed_offset = calc_cruise_offset(cp.vl["CRUISE_PARAMS"]['CRUISE_SPEED_OFFSET'], self.v_ego)
      self.v_cruise_pcm = cp.vl["CRUISE"]['CRUISE_SPEED_PCM']
      # brake switch has shown some single time step noise, so only considered when
      # switch is on for at least 2 consecutive CAN samples
      self.brake_pressed = cp.vl["POWERTRAIN_DATA"]['BRAKE_PRESSED'] or \
                         (self.brake_switch and self.brake_switch_prev and \
                         cp.ts["POWERTRAIN_DATA"]['BRAKE_SWITCH'] != self.brake_switch_ts)
      self.brake_switch_prev = self.brake_switch
      self.brake_switch_ts = cp.ts["POWERTRAIN_DATA"]['BRAKE_SWITCH']

    self.user_brake = cp.vl["VSA_STATUS"]['USER_BRAKE']
    self.pcm_acc_status = cp.vl["POWERTRAIN_DATA"]['ACC_STATUS']
    self.hud_lead = cp.vl["ACC_HUD"]['HUD_LEAD']


# carstate standalone tester
if __name__ == '__main__':
  import zmq
  context = zmq.Context()

  class CarParams(object):
    def __init__(self):
      self.carFingerprint = "HONDA CIVIC 2016 TOURING"
      self.enableGasInterceptor = 0
  CP = CarParams()
  CS = CarState(CP)

  # while 1:
  #   CS.update()
  #   time.sleep(0.01)
