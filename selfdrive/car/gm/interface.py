#!/usr/bin/env python
from cereal import car
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.gm.values import DBC, CAR, STOCK_CONTROL_MSGS, AUDIO_HUD, SUPERCRUISE_CARS, AccState
from selfdrive.car.gm.carstate import CarState, CruiseButtons, get_powertrain_can_parser, get_chassis_can_parser
import selfdrive.kegman_conf as kegman


try:
  from selfdrive.car.gm.carcontroller import CarController
except ImportError:
  CarController = None

class CanBus(object):
  def __init__(self):
    self.powertrain = 0
    self.obstacle = 1
    self.chassis = 2
    self.sw_gmlan = 3

class CarInterface(object):
  def __init__(self, CP, sendcan=None):
    self.angleSteersoffset = float(kegman.conf['angle_steers_offset'])  # deg offset
    self.CP = CP
    self.frame = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
    self.can_invalid_count = 0
    self.acc_active_prev = 0
    self.cruise_enabled_prev = False

    # *** init the major players ***
    canbus = CanBus()
    self.CS = CarState(CP, canbus)
    self.VM = VehicleModel(CP)
    self.pt_cp = get_powertrain_can_parser(CP, canbus)
    self.ch_cp = get_chassis_can_parser(CP, canbus)
    self.ch_cp_dbc_name = DBC[CP.carFingerprint]['chassis']

    # sending if read only is False
    if sendcan is not None:
      self.sendcan = sendcan
      self.CC = CarController(canbus, CP.carFingerprint, CP.enableCamera)

  @staticmethod
  def compute_gb(accel, speed):
    # Ripped from compute_gb_honda in Honda's interface.py. Works well off shelf but may need more tuning
    creep_brake = 0.0
    creep_speed = 2.68
    creep_brake_value = 0.10
    if speed < creep_speed:
      creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
    return float(accel) / 4.8 - creep_brake

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    return 1.0

  @staticmethod
  def get_params(candidate, fingerprint):
    ret = car.CarParams.new_message()

    ret.carName = "gm"
    ret.carFingerprint = candidate

    ret.enableCruise = False

    # Presence of a camera on the object bus is ok.
    # Have to go passive if ASCM is online (ACC-enabled cars),
    # or camera is on powertrain bus (LKA cars without ACC).
    ret.enableCamera = not any(x for x in STOCK_CONTROL_MSGS[candidate] if x in fingerprint)
    ret.openpilotLongitudinalControl = ret.enableCamera

    std_cargo = 136

    if candidate == CAR.VOLT:
      # supports stop and go, but initial engage must be above 18mph (which include conservatism)
      ret.minEnableSpeed = 7 * CV.MPH_TO_MS
      # kg of standard extra cargo to count for driver, gas, etc...
      ret.mass = 1607. + std_cargo
      ret.safetyModel = car.CarParams.SafetyModels.gm
      ret.wheelbase = 2.69
      ret.steerRatio = 16.17 #0.5.10
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4 # wild guess

    elif candidate == CAR.MALIBU:
      # supports stop and go, but initial engage must be above 18mph (which include conservatism)
      ret.minEnableSpeed = 7 * CV.MPH_TO_MS
      ret.mass = 1496 + std_cargo
      ret.safetyModel = car.CarParams.SafetyModels.gm
      ret.wheelbase = 2.83
      ret.steerRatio = 15.8
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4 # wild guess

    elif candidate == CAR.HOLDEN_ASTRA:
      # kg of standard extra cargo to count for driver, gas, etc...
      ret.mass = 1363. + std_cargo
      ret.wheelbase = 2.662
      # Remaining parameters copied from Volt for now
      ret.centerToFront = ret.wheelbase * 0.4
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.safetyModel = car.CarParams.SafetyModels.gm
      ret.steerRatio = 15.75 #0.5.10
      ret.steerRatioRear = 0.

    elif candidate == CAR.ACADIA:
      ret.minEnableSpeed = -1. # engage speed is decided by pcm
      ret.mass = 4353. * CV.LB_TO_KG + std_cargo
      ret.safetyModel = car.CarParams.SafetyModels.gm
      ret.wheelbase = 2.86
      ret.steerRatio = 14.4  #end to end is 13.46
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4

    elif candidate == CAR.BUICK_REGAL:
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.mass = 3779. * CV.LB_TO_KG + std_cargo # (3849+3708)/2
      ret.safetyModel = car.CarParams.SafetyModels.gm
      ret.wheelbase = 2.83 #111.4 inches in meters
      ret.steerRatio = 14.4 # guess for tourx
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4 # guess for tourx

    elif candidate == CAR.CADILLAC_ATS:
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.mass = 1601. + std_cargo
      ret.safetyModel = car.CarParams.SafetyModels.gm
      ret.wheelbase = 2.78
      ret.steerRatio = 15.3
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.49

    elif candidate == CAR.CADILLAC_CT6:
      # engage speed is decided by pcm
      ret.minEnableSpeed = -1.
      # kg of standard extra cargo to count for driver, gas, etc...
      ret.mass = 4016. * CV.LB_TO_KG + std_cargo
      ret.safetyModel = car.CarParams.SafetyModels.cadillac
      ret.wheelbase = 3.11
      ret.steerRatio = 14.6   # it's 16.3 without rear active steering
      ret.steerRatioRear = 0. # TODO: there is RAS on this car!
      ret.centerToFront = ret.wheelbase * 0.465


    # hardcoding honda civic 2016 touring params so they can be used to
    # scale unknown params for other cars
    mass_civic = 2923. * CV.LB_TO_KG + std_cargo
    wheelbase_civic = 2.70
    centerToFront_civic = wheelbase_civic * 0.4
    centerToRear_civic = wheelbase_civic - centerToFront_civic
    rotationalInertia_civic = 2500
    tireStiffnessFront_civic = 85400
    tireStiffnessRear_civic = 90000

    centerToRear = ret.wheelbase - ret.centerToFront
    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = rotationalInertia_civic * \
                            ret.mass * ret.wheelbase**2 / (mass_civic * wheelbase_civic**2)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront = tireStiffnessFront_civic * \
                             ret.mass / mass_civic * \
                             (centerToRear / ret.wheelbase) / (centerToRear_civic / wheelbase_civic)
    ret.tireStiffnessRear = tireStiffnessRear_civic * \
                            ret.mass / mass_civic * \
                            (ret.centerToFront / ret.wheelbase) / (centerToFront_civic / wheelbase_civic)

    # same tuning for Volt and CT6 for now
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.00]]
    ret.lateralTuning.pid.kf = 0.00004   # full torque for 20 deg at 80mph means 0.00007818594

    ret.steerMaxBP = [0.]  # m/s
    ret.steerMaxV = [1.]
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [.5]
    ret.brakeMaxBP = [0.]
    ret.brakeMaxV = [1.]


    ret.longitudinalTuning.kpBP = [0., 5., 55.]
    ret.longitudinalTuning.kpV = [2., 1., 0.5]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.3]
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]


    ret.steerLimitAlert = True

    ret.stoppingControl = True
    # Volt PID controller gets upset in heavy low speed stop-go traffic as startAccel interferes with it.
    ret.startAccel = 0.0

    ret.steerActuatorDelay = 0.1  # Default delay, not measured yet
    ret.steerRateCost = 1.0
    ret.steerControlType = car.CarParams.SteerControlType.torque

    return ret

  # returns a car.CarState
  def update(self, c):

    self.pt_cp.update(int(sec_since_boot() * 1e9), False)
    self.ch_cp.update(int(sec_since_boot() * 1e9), False)
    self.CS.update(self.pt_cp, self.ch_cp)

    # create message
    ret = car.CarState.new_message()

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.aEgo = self.CS.a_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.yawRate = self.VM.yaw_rate(self.CS.angle_steers * CV.DEG_TO_RAD, self.CS.v_ego)
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # gas pedal information.
    ret.gas = self.CS.pedal_gas / 254.0
    ret.gasPressed = self.CS.user_gas_pressed

    # brake pedal
    ret.brake = self.CS.user_brake / 0xd0
    ret.brakePressed = self.CS.brake_pressed
    ret.brakeLights = self.CS.frictionBrakesActive
    # steering wheel
    ret.steeringAngle = self.CS.angle_steers + self.angleSteersoffset

    # torque and user override. Driver awareness
    # timer resets when the user uses the steering wheel.
    ret.steeringPressed = self.CS.steer_override
    ret.steeringTorque = self.CS.steer_torque_driver

    # cruise state
    ret.cruiseState.available = bool(self.CS.main_on)
    cruiseEnabled = self.CS.pcm_acc_status != AccState.OFF
    ret.cruiseState.enabled = cruiseEnabled
    ret.cruiseState.standstill = False

    ret.leftBlinker = self.CS.left_blinker_on
    ret.rightBlinker = self.CS.right_blinker_on
    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt
    ret.gearShifter = self.CS.gear_shifter
    ret.readdistancelines = self.CS.follow_level
    ret.genericToggle = False
    ret.laneDepartureToggle = False
    ret.distanceToggle = self.CS.follow_level
    ret.accSlowToggle = False
    ret.blindspot = self.CS.blind_spot_on

    buttonEvents = []

    # blinkers
    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'leftBlinker'
      be.pressed = self.CS.left_blinker_on
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'rightBlinker'
      be.pressed = self.CS.right_blinker_on
      buttonEvents.append(be)

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'unknown'
      if self.CS.cruise_buttons != CruiseButtons.UNPRESS:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == CruiseButtons.RES_ACCEL:
        if not (cruiseEnabled and self.CS.standstill):
          be.type = 'accelCruise' # Suppress resume button if we're resuming from stop so we don't adjust speed.
      elif but == CruiseButtons.DECEL_SET:
        be.type = 'decelCruise'
        if not cruiseEnabled and not self.CS.lkMode:
          self.CS.lkMode = True
      elif but == CruiseButtons.CANCEL:
        be.type = 'cancel'
      elif but == CruiseButtons.MAIN:
        be.type = 'altButton3'
      buttonEvents.append(be)

    ret.buttonEvents = buttonEvents
    
    if self.CS.lka_button and self.CS.lka_button != self.CS.prev_lka_button:
      if self.CS.lkMode:
        self.CS.lkMode = False
      else:
        self.CS.lkMode = True
        
    if self.CS.distance_button and self.CS.distance_button != self.CS.prev_distance_button:
      self.CS.follow_level -= 1
      if self.CS.follow_level < 1:
        self.CS.follow_level = 3
      kegman.save({'lastTrMode': int(self.CS.follow_level)})  # write last distance bar setting to file
    ret.gasbuttonstatus = self.CS.gasMode
    events = []
    if not self.CS.can_valid:
      self.can_invalid_count += 1
      if self.can_invalid_count >= 5:
        events.append(create_event('commIssue', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    else:
      self.can_invalid_count = 0

    if ret.cruiseState.enabled and not self.cruise_enabled_prev:
      disengage_event = True
    else:
      disengage_event = False

    if self.CS.steer_error:
      events.append(create_event('steerUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))
    if self.CS.steer_not_allowed:
      events.append(create_event('steerTempUnavailable', [ET.NO_ENTRY, ET.WARNING]))
    if ret.doorOpen and disengage_event:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.seatbeltUnlatched and disengage_event:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))

    if self.CS.car_fingerprint in SUPERCRUISE_CARS:
      if self.CS.acc_active and not self.acc_active_prev:
        events.append(create_event('pcmEnable', [ET.ENABLE]))
      if not self.CS.acc_active:
        events.append(create_event('pcmDisable', [ET.USER_DISABLE]))

    else:
      if self.CS.brake_error:
        events.append(create_event('brakeUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))
      if not self.CS.gear_shifter_valid:
        events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
      if self.CS.esp_disabled:
        events.append(create_event('espDisabled', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
      if not self.CS.main_on:
        events.append(create_event('wrongCarMode', [ET.NO_ENTRY, ET.USER_DISABLE]))
      if self.CS.gear_shifter == 3:
        events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
      if ret.vEgo < self.CP.minEnableSpeed:
        events.append(create_event('speedTooLow', [ET.NO_ENTRY]))
      if self.CS.park_brake:
        events.append(create_event('parkBrake', [ET.NO_ENTRY, ET.USER_DISABLE]))
      # disable on pedals rising edge or when brake is pressed and speed isn't zero
      if (ret.gasPressed and not self.gas_pressed_prev) or \
        (ret.brakePressed): # and (not self.brake_pressed_prev or ret.vEgo > 0.001)):
        events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))
      if ret.gasPressed:
        events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))
      if ret.cruiseState.standstill:
        events.append(create_event('resumeRequired', [ET.WARNING]))
      if self.CS.pcm_acc_status == AccState.FAULTED:
        events.append(create_event('controlsFailed', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))

      # handle button presses
      for b in ret.buttonEvents:
        # do enable on both accel and decel buttons
        if b.type in ["accelCruise", "decelCruise"] and not b.pressed:
          events.append(create_event('buttonEnable', [ET.ENABLE]))
        # do disable on button down
        if b.type == "cancel" and b.pressed:
          events.append(create_event('buttonCancel', [ET.USER_DISABLE]))

    ret.events = events

    # update previous brake/gas pressed
    self.acc_active_prev = self.CS.acc_active
    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed
    self.cruise_enabled_prev = ret.cruiseState.enabled

    # cast to reader so it can't be modified
    return ret.as_reader()

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):
    hud_v_cruise = c.hudControl.setSpeed
    if hud_v_cruise > 70:
      hud_v_cruise = 0

    chime, chime_count = AUDIO_HUD[c.hudControl.audibleAlert.raw]

    # For Openpilot, "enabled" includes pre-enable.
    # In GM, PCM faults out if ACC command overlaps user gas.
    enabled = c.enabled and not self.CS.user_gas_pressed

    self.CC.update(self.sendcan, enabled, self.CS, self.frame, \
      c.actuators,
      hud_v_cruise, c.hudControl.lanesVisible, \
      c.hudControl.leadVisible, \
      chime, chime_count)

    self.frame += 1
