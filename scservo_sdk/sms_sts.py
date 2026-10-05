#!/usr/bin/env python

from .scservo_def import *
from .protocol_packet_handler import *
from .group_sync_read import *
from .group_sync_write import *

#波特率定义
SMS_STS_1M = 0
SMS_STS_0_5M = 1
SMS_STS_250K = 2
SMS_STS_128K = 3
SMS_STS_115200 = 4
SMS_STS_76800 = 5
SMS_STS_57600 = 6
SMS_STS_38400 = 7

# Memory table definition.
# ------- Version (read-only) -------
SMS_STS_FIRMWARE_MAJOR = 0
SMS_STS_FIRMWARE_MINOR = 1
SMS_STS_END = 2
SMS_STS_MODEL_L = 3
SMS_STS_MODEL_H = 4

# ------- EPROM (read/write) -------
SMS_STS_ID = 5
SMS_STS_BAUD_RATE = 6
SMS_STS_RESPONSE_LEVEL = 8
SMS_STS_MIN_ANGLE_LIMIT_L = 9
SMS_STS_MIN_ANGLE_LIMIT_H = 10
SMS_STS_MAX_ANGLE_LIMIT_L = 11
SMS_STS_MAX_ANGLE_LIMIT_H = 12
SMS_STS_MAX_TEMPERATURE = 13
SMS_STS_MAX_VOLTAGE = 14
SMS_STS_MIN_VOLTAGE = 15
SMS_STS_MAX_TORQUE_L = 16
SMS_STS_MAX_TORQUE_H = 17
SMS_STS_PHASE = 18
SMS_STS_UNLOAD_CONDITION = 19
SMS_STS_LED_ALARM_CONDITION = 20
SMS_STS_POSITION_P = 21
SMS_STS_POSITION_D = 22
SMS_STS_POSITION_I = 23
SMS_STS_MIN_START_TORQUE = 24
SMS_STS_INTEGRAL_LIMIT = 25
SMS_STS_CW_DEAD = 26
SMS_STS_CCW_DEAD = 27
SMS_STS_PROTECTION_CURRENT_L = 28
SMS_STS_PROTECTION_CURRENT_H = 29
SMS_STS_ANGLE_RESOLUTION = 30
SMS_STS_OFS_L = 31
SMS_STS_OFS_H = 32
SMS_STS_MODE = 33
SMS_STS_HOLD_TORQUE = 34
SMS_STS_PROTECTION_TIME = 35
SMS_STS_OVERLOAD_TORQUE = 36
SMS_STS_SPEED_P = 37
SMS_STS_OVER_CURRENT_TIME = 38
SMS_STS_SPEED_I = 39

# ------- SRAM (read/write) -------
SMS_STS_TORQUE_ENABLE = 40
SMS_STS_TORQUE_DISABLE = 0
SMS_STS_TORQUE_ON = 1
SMS_STS_TORQUE_DAMPING = 2
SMS_STS_ACC = 41
SMS_STS_GOAL_POSITION_L = 42
SMS_STS_GOAL_POSITION_H = 43
SMS_STS_GOAL_PWM_L = 44
SMS_STS_GOAL_PWM_H = 45
SMS_STS_GOAL_SPEED_L = 46
SMS_STS_GOAL_SPEED_H = 47
SMS_STS_GOAL_TORQUE_L = 48
SMS_STS_GOAL_TORQUE_H = 49
SMS_STS_LOCK = 55

# ------- SRAM (read-only) -------
SMS_STS_PRESENT_POSITION_L = 56
SMS_STS_PRESENT_POSITION_H = 57
SMS_STS_PRESENT_SPEED_L = 58
SMS_STS_PRESENT_SPEED_H = 59
SMS_STS_PRESENT_LOAD_L = 60
SMS_STS_PRESENT_LOAD_H = 61
SMS_STS_PRESENT_VOLTAGE = 62
SMS_STS_PRESENT_TEMPERATURE = 63
SMS_STS_MOVING = 66
SMS_STS_PRESENT_CURRENT_L = 69
SMS_STS_PRESENT_CURRENT_H = 70
SMS_STS_PRESENT_LOAD_DIRECTION_BIT = 10
SMS_STS_PRESENT_POSITION_DIRECTION_BIT = 15
SMS_STS_PRESENT_SPEED_DIRECTION_BIT = 15
SMS_STS_CURRENT_UNIT_MA = 6.5

class sms_sts(protocol_packet_handler):
    def __init__(self, portHandler):
        protocol_packet_handler.__init__(self, portHandler, 0)
        self.groupSyncWrite = GroupSyncWrite(self, SMS_STS_ACC, 9)

    def WritePosEx(self, scs_id, position, speed, acc, torque=1000):
        # Current STS map: 41=acc, 42-43=position, 44-45=PWM,
        # 46-47=speed, 48-49=torque limit.
        position = self.scs_toscs(position, 15)
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position),
                    0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed),
                    self.scs_lobyte(torque), self.scs_hibyte(torque)]
        return self.writeTxRx(scs_id, SMS_STS_ACC, len(txpacket), txpacket)

    def WritePosTorque(self, scs_id, position, speed, acc, torque):
        return self.WritePosEx(scs_id, position, speed, acc, torque)

    def ReadPos(self, scs_id):
        scs_present_position, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L)
        return self.scs_tohost(scs_present_position, 15), scs_comm_result, scs_error

    def ReadSpeed(self, scs_id):
        scs_present_speed, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SMS_STS_PRESENT_SPEED_L)
        return self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadLoad(self, scs_id):
        present_load, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SMS_STS_PRESENT_LOAD_L)
        return self.scs_tohost(present_load, SMS_STS_PRESENT_LOAD_DIRECTION_BIT), scs_comm_result, scs_error

    def ReadCurrent(self, scs_id):
        return self.read2ByteTxRx(scs_id, SMS_STS_PRESENT_CURRENT_L)

    def ReadPosSpeed(self, scs_id):
        scs_present_position_speed, scs_comm_result, scs_error = self.read4ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L)
        scs_present_position = self.scs_loword(scs_present_position_speed)
        scs_present_speed = self.scs_hiword(scs_present_position_speed)
        return self.scs_tohost(scs_present_position, 15), self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadMoving(self, scs_id):
        moving, scs_comm_result, scs_error = self.read1ByteTxRx(scs_id, SMS_STS_MOVING)
        return moving, scs_comm_result, scs_error

    def SyncWritePosEx(self, scs_id, position, speed, acc, torque=1000):
        position = self.scs_toscs(position, 15)
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position),
                    0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed),
                    self.scs_lobyte(torque), self.scs_hibyte(torque)]
        return self.groupSyncWrite.addParam(scs_id, txpacket)

    def RegWritePosEx(self, scs_id, position, speed, acc, torque=1000):
        position = self.scs_toscs(position, 15)
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position),
                    0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed),
                    self.scs_lobyte(torque), self.scs_hibyte(torque)]
        return self.regWriteTxRx(scs_id, SMS_STS_ACC, len(txpacket), txpacket)

    def RegAction(self):
        return self.action(BROADCAST_ID)

    def WheelMode(self, scs_id):
        return self.write1ByteTxRx(scs_id, SMS_STS_MODE, 1)

    def WriteSpec(self, scs_id, speed, acc, torque=1000):
        speed = self.scs_toscs(speed, 15)
        txpacket = [acc, 0, 0, 0, 0,
                    self.scs_lobyte(speed), self.scs_hibyte(speed),
                    self.scs_lobyte(torque), self.scs_hibyte(torque)]
        return self.writeTxRx(scs_id, SMS_STS_ACC, len(txpacket), txpacket)

    def LockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, SMS_STS_LOCK, 1)

    def unLockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, SMS_STS_LOCK, 0)

