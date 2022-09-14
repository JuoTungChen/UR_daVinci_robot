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
        ParamWord(30, 'goal_position', range_raw=(0, 65535), to_human=lambda x: tc_bin2int(x, 16)*MX28.angle_unit, to_device=lambda x: tc_int2bin(int(round(x/MX28.angle_unit)), 16)),
        ParamWord(32, 'moving_speed', range_raw=(0, 2047), to_human=lambda x: rpm2radsec(dir2signed(x)*MX28.rpm_per_tick), to_device=lambda x: signed2dir(int(round(radsec2rpm(x/MX28.rpm_per_tick))))),
        ParamWord(34, 'torque_limit', range_raw=(0, 1023), to_human=lambda x: x/1023, to_device=lambda x: int(round(x*1023))),
        ParamWord(36, 'present_position', range_raw=(0, 65535), to_human=lambda x: tc_bin2int(x, 16)*MX28.angle_unit, readonly=True),
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


# Map model numbers to device types
device_models = {
    12: AX12,
    29: MX28,
}
