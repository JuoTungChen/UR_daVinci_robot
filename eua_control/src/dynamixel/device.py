from __future__ import division

import math
import itertools

from .conversions import *
from .packet import *
from .param import *


def _pairwise(iterable):
    # s -> (s0,s1), (s1,s2), (s2, s3), ...
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)


def _params_contiguous_groups(params):
    groups = [set(),]

    for a, b in _pairwise(sorted(params, key=lambda p: p.address)):
        if a.address + a.size == b.address:
            groups[-1].update([a, b])
        else:
            groups.append(set([b]))

    # return [sorted(list(g), key=lambda p: p.address) for g in groups]
    return groups


class Device(object):
    def __init__(self, io, protocol=1):
        self.io = io
        self.protocol = protocol

    def __str__(self):
        return '{} (id: {}) on {}'.format(self.type_name, self.id, self.io.ser.name)

    def read_param_single(self, name, raw=False):
        """Read a single parameter from the device
        """
        p = self.cc[name]
        req = Packet(self.id, READ_DATA, [p.address, p.size])
        resp = self.io.sync_request(req, Packet.length_total_with_no_payload + p.size)
        x = p.frompayload(resp.payload)

        if not raw and p.to_human is not None:
            return p.to_human(x)
        else:
            return x

    def _read_params_contiguous(self, params, raw=False):
        """Read contiguous parameters in a single operation

        This reads a number of registers for a single device and does not use
        the BULK_READ instruction.
        """
        params = sorted(params, key=lambda p: p.address)
        total_size = sum([p.size for p in params])

        if total_size > 250:
            raise Exception("Trying to read too many ({}) bytes at once".format(total_size))

        req = Packet(self.id, READ_DATA, [params[0].address, total_size])
        resp = self.io.sync_request(req, Packet.length_total_with_no_payload + total_size)

        # Parse response packet payload
        d = {}
        offset = 0

        for p in params:
            x = p.frompayload(resp.payload[offset:offset+p.size])

            if not raw and p.to_human is not None:
                d[p.name] = p.to_human(x)
            else:
                d[p.name] = x

            offset += p.size

        return d

    def read_params(self, names, raw=False):
        """Read a sequence of parameters

        Reads are performed in chunks of parameters which are contiguous in
        device memory for better utilization of the half-duplex serial line.
        """
        params = [self.cc[n] for n in names]
        groups = _params_contiguous_groups(params)
        d = {}

        for g in groups:
            d.update(self._read_params_contiguous(g, raw))

        return d

    def dump_cc(self, raw=False):
        return self.read_params(self.cc.keys(), raw)

    def write_param_single(self, name, val, raw=False):
        """Write a single parameter to the device
        """
        p = self.cc[name]

        if not raw and p.to_device is not None:
            val = p.to_device(val)

        if p.readonly:
            raise AttributeError("Setting '{}' to '{}' failed: Param is read-only".format(name, val))

        if p.values_raw is not None and not val in p.values_raw:
            raise ValueError("Setting '{}' to '{}' failed: Must be one of {}".format(name, val, p.values_raw))

        if p.range_raw is not None and not (p.range_raw[0] <= val <= p.range_raw[1]):
            raise ValueError("Setting '{}' to '{}' failed: Out of range {}".format(name, val, p.range_raw))

        req = Packet(self.id, WRITE_DATA)
        req.payload = [p.address] + p.topayload(val)
        self.io.sync_request(req, Packet.length_total_with_no_payload)

    def _write_params_contiguous(self, params, d, raw=False):
        """
        """
        params = sorted(params, key=lambda p: p.address)
        size = sum([p.size for p in params])

        if size > 250:
            raise Exception("Trying to write too many ({}) bytes at once".format(size))

        req = Packet(self.id, WRITE_DATA, [params[0].address])

        for p in params:
            val = d[p.name]

            if p.readonly:
                raise AttributeError("Setting '{}' to '{}' failed: Param is read-only".format(p.name, val))

            if not raw and p.to_device is not None:
                val = p.to_device(val)

            if p.values_raw is not None and not val in p.values_raw:
                raise ValueError("Setting '{}' to '{}' failed: Must be one of {}".format(name, val, p.values_raw))

            if p.range_raw is not None and not (p.range_raw[0] <= val <= p.range_raw[1]):
                raise ValueError("Setting '{}' to '{}' failed: Out of range {}".format(p.name, val, p.range_raw))

            req.payload.extend(p.topayload(val))

        self.io.sync_request(req, Packet.length_total_with_no_payload)

    def write_params(self, names_values, raw=False):
        """Write a sequence of parameters

        Writes are performed in chunks of parameters which are contiguous in
        device memory for better utilization of the half-duplex serial line.
        """
        params = [self.cc[n] for n in names_values]
        groups = _params_contiguous_groups(params)

        for g in groups:
            self._write_params_contiguous(g, names_values, raw)


class AX12(Device):
    """
    AX-12A/AX-12W/AX_18A

    http://emanual.robotis.com/docs/en/dxl/ax/ax-12a
    """
    type_name = 'AX-12'
    encoder_ticks = 1024
    angle_range = (0, math.radians(300))
    angle_unit = (angle_range[1] - angle_range[0]) / (encoder_ticks - 1)
    rpm_per_tick = 0.111  # approximate value, depends on max RPM, and thus supply voltage

    # List of control table parameters and corresponding unit conversions
    params = [
        # EEPROM
        ParamWord(0, 'model_number', readonly=True),
        ParamByte(2, 'firmware_version', readonly=True),
        ParamByte(3, 'id', range_raw=(0, 252)),
        ParamByte(4, 'baud_rate', range_raw=(1, 207), to_human=lambda x: 2000000/(x+1), to_device=lambda x: int(round(2000000/x-1))),
        ParamByte(5, 'return_delay_time', range_raw=(0, 254), to_human=lambda x: x*2, to_device=lambda x: int(round(x/2))),
        ParamWord(6, 'cw_angle_limit', range_raw=(0, encoder_ticks-1), to_human=lambda x: x*AX12.angle_unit, to_device=lambda x: int(round(x/AX12.angle_unit))),
        ParamWord(8, 'ccw_angle_limit', range_raw=(0, encoder_ticks-1), to_human=lambda x: x*AX12.angle_unit, to_device=lambda x: int(round(x/AX12.angle_unit))),
        ParamByte(11, 'temparature_limit', readonly=True),
        ParamByte(12, 'min_voltage_limit', range_raw=(50, 160), to_human=lambda x: x/10, to_device=lambda x: int(round(x*10))),
        ParamByte(13, 'max_voltage_limit', range_raw=(50, 160), to_human=lambda x: x/10, to_device=lambda x: int(round(x*10))),
        ParamWord(14, 'max_torque', range_raw=(0, 1023), to_human=lambda x: x/1023, to_device=lambda x: int(round(x*1023))),
        ParamByte(16, 'status_return_level', values_raw=(0, 1, 2)),
        ParamByte(17, 'alarm_led'),
        ParamByte(18, 'shutdown'),
        # RAM
        ParamByte(24, 'torque_enable', values_raw=(0, 1), to_human=lambda x: bool(x), to_device=lambda x: int(x)),
        ParamByte(25, 'led', values_raw=(0, 1), to_human=lambda x: bool(x), to_device=lambda x: int(x)),
        ParamByte(26, 'cw_compliance_margin', range_raw=(0, 255), to_human=lambda x: x*AX12.angle_unit, to_device=lambda x: int(round(x/AX12.angle_unit))),
        ParamByte(27, 'ccw_compliance_margin', range_raw=(0, 255), to_human=lambda x: x*AX12.angle_unit, to_device=lambda x: int(round(x/AX12.angle_unit))),
        ParamByte(28, 'cw_compliance_slope', range_raw=(0, 255), to_human=cslope2step, to_device=step2cslope),  # data value to step #
        ParamByte(29, 'ccw_compliance_slope', range_raw=(0, 255), to_human=cslope2step, to_device=step2cslope),  # data value to step #
        ParamWord(30, 'goal_position', range_raw=(0, encoder_ticks-1), to_human=lambda x: x*AX12.angle_unit, to_device=lambda x: int(round(x/AX12.angle_unit))),  # 0.29 deg to rad
        ParamWord(32, 'moving_speed', range_raw=(0, 2047), to_human=lambda x: rpm2radsec(dir2signed(x)*AX12.rpm_per_tick), to_device=lambda x: signed2dir(int(round(radsec2rpm(x/AX12.rpm_per_tick))))),
        ParamWord(34, 'torque_limit', range_raw=(0, 1023), to_human=lambda x: x/1023, to_device=lambda x: int(round(x*1023))),
        ParamWord(36, 'present_position', range_raw=(0, encoder_ticks-1), to_human=lambda x: x*AX12.angle_unit, readonly=True),
        ParamWord(38, 'present_speed', range_raw=(0, 2047), to_human=lambda x: rpm2radsec(dir2signed(x)*AX12.rpm_per_tick), readonly=True),
        ParamWord(40, 'present_load', range_raw=(0, 2047), to_human=lambda x: dir2signed(x)/1023, readonly=True),
        ParamByte(42, 'present_voltage', to_human=lambda x: x/10, readonly=True),
        ParamByte(43, 'present_temperature', range_raw=(0, 255), readonly=True),
        ParamByte(44, 'registered_instruction', values_raw=(0, 1), to_human=lambda x: bool(x), readonly=True),
        ParamByte(46, 'moving', values_raw=(0, 1), to_human=lambda x: bool(x), readonly=True),
        ParamByte(47, 'lock', values_raw=(0, 1), to_human=lambda x: bool(x)),
        ParamWord(48, 'punch', range_raw=(32, 1023)),
    ]

    # Control table indexing parameters by name
    cc = {p.name: p for p in params}

    def __init__(self, io, ident):
        super(AX12, self).__init__(io)
        self.id = ident


class MX28(Device):
    """
    MX-28T/R/AT/AR with protocol 1.0 control table

    http://emanual.robotis.com/docs/en/dxl/mx/mx-28
    """
    type_name = 'MX-28'
    encoder_ticks = 4096
    angle_range = (0, 2*math.pi)
    angle_unit = (angle_range[1] - angle_range[0]) / (encoder_ticks - 1)
    rpm_per_tick = 0.114  # approximate value, depends on max RPM, and thus supply voltage

    # List of control table parameters and corresponding unit to_humanersions
    params = [
        # EEPROM
        ParamWord(0, 'model_number', readonly=True),
        ParamByte(2, 'firmware_version', readonly=True),
        ParamByte(3, 'id', range_raw=(0, 252)),
        ParamByte(4, 'baud_rate', range_raw=(1, 207), to_human=lambda x: 2000000/(x+1), to_device=lambda x: int(round(2000000/x-1))),
        ParamByte(5, 'return_delay_time', range_raw=(0, 254), to_human=lambda x: x*2, to_device=lambda x: int(round(x/2))),
        ParamWord(6, 'cw_angle_limit', range_raw=(0, encoder_ticks-1), to_human=lambda x: x*MX28.angle_unit, to_device=lambda x: int(round(x/MX28.angle_unit))),
        ParamWord(8, 'ccw_angle_limit', range_raw=(0, encoder_ticks-1), to_human=lambda x: x*MX28.angle_unit, to_device=lambda x: int(round(x/MX28.angle_unit))),
        ParamByte(11, 'temparature_limit', readonly=True),
        ParamByte(12, 'min_voltage_limit', range_raw=(50, 160), to_human=lambda x: x/10, to_device=lambda x: int(round(x*10))),
        ParamByte(13, 'max_voltage_limit', range_raw=(50, 160), to_human=lambda x: x/10, to_device=lambda x: int(round(x*10))),
        ParamWord(14, 'max_torque', range_raw=(0, 1023), to_human=lambda x: x/1023, to_device=lambda x: int(round(x*1023))),
        ParamByte(16, 'status_return_level', values_raw=(0, 1, 2)),
        ParamByte(17, 'alarm_led'),
        ParamByte(18, 'shutdown'),
        ParamWord(20, 'multiturn_offset', range_raw=(-24576, 24576)),
        ParamByte(22, 'resolution_divider', range_raw=(1, 4)),
        # RAM
        ParamByte(24, 'torque_enable', values_raw=(0, 1), to_human=lambda x: bool(x), to_device=lambda x: int(x)),
        ParamByte(25, 'led', values_raw=(0, 1), to_human=lambda x: bool(x), to_device=lambda x: int(x)),
        ParamByte(26, 'd_gain', range_raw=(0, 254)),  # FIXME: conversion functions
        ParamByte(27, 'i_gain', range_raw=(0, 254)),
        ParamByte(28, 'p_gain', range_raw=(0, 254)),
        # ParamWord(30, 'goal_position', range_raw=(-28672, 28672), to_human=lambda x: x*MX28.angle_unit, to_device=lambda x: int(round(x/MX28.angle_unit))),  # 0.29 deg to rad
        ParamWord(30, 'goal_position', range_raw=(-28672, 28672), to_human=lambda x: tc_bin2int(x, 16)*MX28.angle_unit, to_device=lambda x: tc_int2bin(int(round(x/MX28_2.angle_unit)), 16)),
        ParamWord(32, 'moving_speed', range_raw=(0, 2047), to_human=lambda x: rpm2radsec(dir2signed(x)*MX28.rpm_per_tick), to_device=lambda x: signed2dir(int(round(radsec2rpm(x/MX28.rpm_per_tick))))),
        ParamWord(34, 'torque_limit', range_raw=(0, 1023), to_human=lambda x: x/1023, to_device=lambda x: int(round(x*1023))),
        # ParamWord(36, 'present_position', range_raw=(0, encoder_ticks-1), to_human=lambda x: x*MX28.angle_unit, readonly=True),
        ParamWord(36, 'present_position', range_raw=(0, encoder_ticks-1), to_human=lambda x: tc_bin2int(x, 16)*MX28.angle_unit, readonly=True),
        ParamWord(38, 'present_speed', range_raw=(0, 2047), to_human=lambda x: rpm2radsec(dir2signed(x)*MX28.rpm_per_tick), readonly=True),
        ParamWord(40, 'present_load', range_raw=(0, 2047), to_human=lambda x: dir2signed(x)/1023, readonly=True),
        ParamByte(42, 'present_voltage', to_human=lambda x: x/10, readonly=True),
        ParamByte(43, 'present_temperature', range_raw=(0, 255), readonly=True),
        ParamByte(44, 'registered_instruction', values_raw=(0, 1), to_human=lambda x: bool(x), readonly=True),
        ParamByte(46, 'moving', values_raw=(0, 1), to_human=lambda x: bool(x), readonly=True),
        ParamByte(47, 'lock', values_raw=(0, 1), to_human=lambda x: bool(x)),
        ParamWord(48, 'punch', range_raw=(32, 1023)),
        ParamWord(50, 'realtime_tick', readonly=True, range_raw=(0, 32767)),
        # ParamByte(73, 'goal_acceleration'),  # TODO
    ]

    # Control table indexing parameters by name
    cc = {p.name: p for p in params}

    def __init__(self, io, ident):
        super(MX28, self).__init__(io)
        self.id = ident


class MX28_2(Device):
    """
    MX-28T/R/AT/AR with protocol 2.0 control table

    http://emanual.robotis.com/docs/en/dxl/mx/mx-28-2
    """
    type_name = 'MX-28(2.0)'
    encoder_ticks = 4096
    angle_range = (0, 2*math.pi)
    angle_unit = (angle_range[1] - angle_range[0]) / (encoder_ticks - 1)
    vel_per_val = 0.229  # rev/min, approximate value
    acc_per_val = 214.577  # rev/min^2

    # List of control table parameters and corresponding unit conversions
    params = [
        # EEPROM
        ParamWord(0, 'model_number', readonly=True),
        ParamDoubleWord(2, 'model_information', readonly=True),
        ParamByte(6, 'firmware_version', readonly=True),
        ParamByte(7, 'id', range_raw=(0, 252)),
        ParamByte(8, 'baud_rate', range_raw=(0, 7), to_human=lambda x: baud_map[x], to_device=lambda x: baud_map[x]),
        ParamByte(9, 'return_delay_time', range_raw=(0, 254), to_human=lambda x: x*2, to_device=lambda x: int(round(x/2))),
        ParamByte(10, 'drive_mode'),
        ParamByte(11, 'operating_mode', values_raw=(1, 3, 4, 16)),
        ParamByte(12, 'secondary_id'),
        ParamByte(13, 'protocol_type', values_raw=(1, 2)),
        ParamDoubleWord(20, 'homing_offset', range_raw=(-1044479, 1044479), to_human=lambda x: tc_bin2int(x, 32)*MX28_2.angle_unit, to_device=lambda x: tc_int2bin(int(round(x/MX28_2.angle_unit)), 32)),
        ParamDoubleWord(24, 'moving_threshold', range_raw=(0, 1023)),
        ParamByte(31, 'temparature_limit', range_raw=(0, 80)),
        ParamWord(32, 'min_voltage_limit', range_raw=(95, 160), to_human=lambda x: x/10, to_device=lambda x: int(round(x*10))),
        ParamWord(34, 'max_voltage_limit', range_raw=(95, 160), to_human=lambda x: x/10, to_device=lambda x: int(round(x*10))),
        ParamWord(36, 'pwm_limit', range_raw=(0, 885), to_human=lambda x: x/885*100, to_device=lambda x: x*885/100),
        ParamDoubleWord(40, 'acceleration_limit', range_raw=(0, 32767), to_human=lambda x: rpmsq2radsecsq(x*MX28_2.acc_per_val), to_device=lambda x: int(round(radsecsq2rpmsq(x/MX28_2.acc_per_val)))),
        ParamDoubleWord(44, 'velocity_limit', range_raw=(0, 1023), to_human=lambda x: rpm2radsec(x*MX28_2.vel_per_val), to_device=lambda x: int(round(radsec2rpm(x/MX28_2.vel_per_val)))),
        ParamDoubleWord(48, 'max_position_limit', range_raw=(0, encoder_ticks-1), to_human=lambda x: x*MX28_2.angle_unit, to_device=lambda x: int(round(x/MX28_2.angle_unit))),
        ParamDoubleWord(52, 'min_position_limit', range_raw=(0, encoder_ticks-1), to_human=lambda x: x*MX28_2.angle_unit, to_device=lambda x: int(round(x/MX28_2.angle_unit))),
        ParamByte(63, 'shutdown'),
        # RAM
        ParamByte(64, 'torque_enable', values_raw=(0, 1), to_human=lambda x: bool(x), to_device=lambda x: int(x)),
        ParamByte(65, 'led', values_raw=(0, 1), to_human=lambda x: bool(x), to_device=lambda x: int(x)),
        ParamByte(68, 'status_return_level', values_raw=(0, 1, 2)),
        ParamByte(69, 'registered_instruction', to_human=lambda x: bool(x), readonly=True),
        ParamByte(70, 'hardware_error_status', readonly=True),
        ParamWord(76, 'velocity_i_gain'),  # TODO
        ParamWord(78, 'velocity_p_gain'),  # TODO
        ParamWord(80, 'position_d_gain'),  # TODO
        ParamWord(82, 'position_i_gain'),  # TODO
        ParamWord(84, 'position_p_gain'),  # TODO
        ParamWord(88, 'feedforward_2nd_gain'),  # TODO
        ParamWord(90, 'feedforward_1st_gain'),  # TODO
        ParamByte(98, 'bus_watchdog'),
        ParamWord(100, 'goal_pwm', range_raw=(0, 885), to_human=lambda x: x/885*100, to_device=lambda x: x*885/100),
        ParamDoubleWord(104, 'goal_velocity', range_raw=(-1023, 1023), to_human=lambda x: rpm2radsec(tc_bin2int(x, 32)*MX28_2.vel_per_val), to_device=lambda x: int(round(radsec2rpm(tc_int2bin(x, 32)/MX28_2.vel_per_val)))),
        ParamDoubleWord(108, 'profile_acceleration', range_raw=(0, 32767), to_human=lambda x: rpmsq2radsecsq(x*MX28_2.acc_per_val), to_device=lambda x: int(round(radsecsq2rpmsq(x/MX28_2.acc_per_val)))),
        ParamDoubleWord(112, 'profile_velocity', to_human=lambda x: rpm2radsec(x*MX28_2.vel_per_val), to_device=lambda x: int(round(radsec2rpm(x/MX28_2.vel_per_val)))),  # TODO support time-based profile
        ParamDoubleWord(116, 'goal_position', range_raw=(0, encoder_ticks-1), to_human=lambda x: x*MX28_2.angle_unit, to_device=lambda x: int(round(x/MX28_2.angle_unit))),  # TODO support wheel/extended control modes
        ParamWord(120, 'realtime_tick', readonly=True),
        ParamByte(122, 'moving', to_human=lambda x: bool(x), readonly=True),
        ParamByte(123, 'moving_status', readonly=True),
        ParamWord(124, 'present_pwm', readonly=True),
        ParamWord(126, 'present_load', to_human=lambda x: tc_bin2int(x, 16)/100, readonly=True),
        ParamDoubleWord(128, 'present_velocity', to_human=lambda x: rpm2radsec(tc_bin2int(x, 32)*MX28_2.vel_per_val), readonly=True),
        ParamDoubleWord(132, 'present_position', to_human=lambda x: x*MX28_2.angle_unit, readonly=True),
        ParamDoubleWord(136, 'velocity_trajectory', readonly=True),  # TODO
        ParamDoubleWord(140, 'position_trajectory', readonly=True),  # TODO
        ParamWord(144, 'present_input_voltage', to_human=lambda x: x/10, readonly=True),
        ParamWord(146, 'present_temperature', readonly=True),
    ]

    # Control table indexing parameters by name
    cc = {p.name: p for p in params}

    def __init__(self, io, ident):
        super(MX28_2, self).__init__(io)
        self.id = ident


# Map model numbers to device types
device_models = {
    12: AX12,
    29: MX28,
    30: MX28_2,
}
