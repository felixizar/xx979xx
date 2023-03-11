from cereal import car
# from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfahda_mfc, create_acc_commands, create_acc_opt, create_frt_radar_opt, \
                                            create_mdps12, create_spas11, create_spas12, create_ems11
from selfdrive.car.hyundai.values import Buttons, CarControllerParams, CAR, FEATURES
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState


def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):
  sys_warning = (visual_alert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  sys_state = 1
  if left_lane and right_lane or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in (CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in (CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.p = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)

    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.steer_rate_limited = False
    self.last_resume_frame = 0
    self.accel = 0
    self.lkas11_cnt = 0
    self.scc12_cnt = 0
    self.last_lead_distance = 0
    if CP.spasEnabled:
      self.en_cnt = 0
      self.apply_steer_ang = 0.0
      self.en_spas = 3
      self.mdps11_stat_last = 0
      self.spas_always = False

  def update(self, c, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, hud_speed,
             left_lane, right_lane, left_lane_depart, right_lane_depart):
    # Steering Torque
    new_steer = int(round(actuators.steer * self.p.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
    self.steer_rate_limited = new_steer != apply_steer

    # SPAS limit angle extremes for safety
    if CS.spas_enabled:
      apply_steer_ang_req = clip(actuators.steerAngle, -1*(self.p.STEER_ANG_MAX), self.p.STEER_ANG_MAX)
      # SPAS limit angle rate for safety
      if abs(self.apply_steer_ang - apply_steer_ang_req) > self.p.STEER_ANG_MAX_RATE:
        if apply_steer_ang_req > self.apply_steer_ang:
          self.apply_steer_ang += self.p.STEER_ANG_MAX_RATE
        else:
          self.apply_steer_ang -= self.p.STEER_ANG_MAX_RATE
      else:
        self.apply_steer_ang = apply_steer_ang_req
    spas_active = CS.spas_enabled and enabled and (self.spas_always or CS.out.vEgo < 7.0) # 25km/h

    # disable when temp fault is active, or below LKA minimum speed
    lkas_active = c.active and not CS.out.steerWarning and CS.out.vEgo >= CS.CP.minSteerSpeed and \
    (CS.CP.maxSteeringAngleDeg > 0 and abs(CS.out.steeringAngleDeg) < CS.CP.maxSteeringAngleDeg) and not spas_active

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning = \
      process_hud_alert(enabled, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    clu11_speed = CS.clu11["CF_Clu_Vanz"]
    enabled_speed = 38 if CS.is_set_speed_in_mph  else 60
    if clu11_speed > enabled_speed or not lkas_active:
      enabled_speed = clu11_speed

    # if not(min_set_speed < set_speed < 255 * CV.KPH_TO_MS):
    #   set_speed = min_set_speed
    # set_speed *= CV.MS_TO_MPH if CS.is_set_speed_in_mph else CV.MS_TO_KPH

    can_sends = []

    # tester present - w/ no response (keeps radar disabled)
    if CS.CP.openpilotLongitudinalControl:
      if (frame % 100) == 0:
        can_sends.append([0x7D0, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0])

    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled, left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, 0))
    if CS.mdps_bus == 1 or CS.scc_bus == 1: # send lkas11 bus 1 if mdps or scc is on bus 1
      can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled, left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, 1))
    if frame % 2 and CS.mdps_bus: # send clu11 to mdps if it is not on bus 0
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.NONE, enabled_speed, CS.mdps_bus))
    if not CS.CP.openpilotLongitudinalControl:
      if pcm_cancel_cmd:
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL, clu11_speed, CS.scc_bus))
      elif CS.out.cruiseState.standstill:
        # run only first time when the car stopped
        if self.last_lead_distance == 0:
          # get the lead distance from the Radar
          self.last_lead_distance = CS.lead_distance
          self.resume_cnt = 0
        # when lead car starts moving, create 6 RES msgs
        elif CS.lead_distance != self.last_lead_distance and (frame - self.last_resume_frame) > 5:
          can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL, clu11_speed, CS.scc_bus))
          self.resume_cnt += 1
          # interval after 6 msgs
          if self.resume_cnt > 5:
            self.last_resume_frame = frame
            self.clu11_cnt = 0
      # reset lead distnce after the car starts moving
      elif self.last_lead_distance != 0:
        self.last_lead_distance = 0
    # send mdps12 to LKAS to prevent LKAS error
    if CS.mdps_bus == 1:
      can_sends.append(create_mdps12(self.packer, frame, CS.mdps12))
    if frame % 2 == 0 and CS.CP.openpilotLongitudinalControl:
      lead_visible = c.hudControl.leadVisible
      accel = actuators.accel if c.active else 0

      jerk = clip(2.0 * (accel - CS.out.aEgo), -12.7, 12.7)

      if accel < 0:
        accel = interp(accel - CS.out.aEgo, [-1.0, -0.5], [2 * accel, accel])

      accel = clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

      stopping = (actuators.longControlState == LongCtrlState.stopping)
      set_speed_in_units = hud_speed * (CV.MS_TO_MPH if CS.clu11["CF_Clu_SPEED_UNIT"] == 1 else CV.MS_TO_KPH)
      can_sends.extend(create_acc_commands(self.packer, enabled, accel, jerk, int(frame / 2), lead_visible, set_speed_in_units, stopping, self.car_fingerprint))
      self.accel = accel

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in FEATURES["send_lfa_mfa"]:
      can_sends.append(create_lfahda_mfc(self.packer, enabled))

    # 5 Hz ACC options
    if frame % 20 == 0 and CS.CP.openpilotLongitudinalControl and self.car_fingerprint in FEATURES["use_fca"]:
      can_sends.extend(create_acc_opt(self.packer))

    # 2 Hz front radar options
    if frame % 50 == 0 and CS.CP.openpilotLongitudinalControl:
      can_sends.append(create_frt_radar_opt(self.packer))

    if CS.spas_enabled:
      if CS.mdps_bus:
        can_sends.append(create_ems11(self.packer, CS.ems11, spas_active))

      # SPAS11 50hz
      if (frame % 2) == 0:
        if CS.mdps11_stat == 7 and not self.mdps11_stat_last == 7:
          self.en_spas = 7
          self.en_cnt = 0

        if self.en_spas == 7 and self.en_cnt >= 8:
          self.en_spas = 3
          self.en_cnt = 0

        if self.en_cnt < 8 and spas_active:
          self.en_spas = 4
        elif self.en_cnt >= 8 and spas_active:
          self.en_spas = 5

        if not spas_active:
          self.apply_steer_ang = CS.mdps11_strang
          self.en_spas = 3
          self.en_cnt = 0

        self.mdps11_stat_last = CS.mdps11_stat
        self.en_cnt += 1
        can_sends.append(create_spas11(self.packer, self.car_fingerprint, (frame // 2), self.en_spas, self.apply_steer_ang, CS.mdps_bus))

      # SPAS12 20Hz
      if (frame % 5) == 0:
        can_sends.append(create_spas12(CS.mdps_bus))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.p.STEER_MAX
    new_actuators.accel = self.accel

    return new_actuators, can_sends
