import math
import serial
import threading

from .device import *
from .packet import *
from .param import *


# ID constants
ID_MIN       = 0
ID_MAX       = 252
ID_USB2AX    = 253
ID_BROADCAST = 254


class DxlIO(object):
    """Class to synchronize serial port access
    """
    def __init__(self, port, baudrate):
        # Configure a long-ish timeout value since for each read() we know the
        # expected length of the response packet
        self.ser = serial.PosixPollSerial(port, baudrate, timeout=0.1)
        self.mutex = threading.Lock()

    def __del__(self):
        self.close()

    def close(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
            self.ser.close()

    def _request(self, request, expected_response=None):
        self.ser.write(request.serialize())

        if not expected_response:
            return

        data = self.ser.read(expected_response)

        if len(data) != expected_response:
            raise Exception("Read {} bytes (after timeout), but expected {}".format(len(data), expected_response))

        response = Packet().deserialize(data)
        return response

    def sync_request(self, request, expected_response=None):
        with self.mutex:
            return self._request(request, expected_response)


class Chain(object):
    """A chain of Dynamixel devices on a single port
    """
    def __init__(self, port, baudrate, device_ids=None):
        self.io = DxlIO(port, baudrate)
        self.devices = []
        self._find_devices(device_ids)

    def _find_devices(self, device_ids):
        # Scan for attached devices by pinging ID's. Ping packets may or may not
        # get a response, depending on whether a device with the given ID exists.
        with self.io.mutex:
            ids = device_ids if device_ids else list(range(ID_MIN, ID_MAX+1))

            for i in ids:
                if self._ping(i):
                    model = self._read_model_number(i)

                    try:
                        self.devices.append(device_models[model](self.io, i))
                    except KeyError:
                        # TODO: Handle other Dynamixel device types
                        raise Exception("I don't know how to handle device '{}' with model number '{}'".format(i, model))

    def _ping(self, ident):
        p = Packet(ident, PING)
        self.io.ser.write(p.serialize())

        try:
            data = self.io.ser.read(Packet.length_total_with_no_payload)
            response = Packet()
            response.deserialize(data)
        except MissingDataError:
            # No response -> no servo with this ID attached
            return False
        except UnboundLocalError:
            # Bug in PySerial: https://github.com/pyserial/pyserial/issues/617
            return False
        else:
            return True

    def _read_model_number(self, ident):
        # Model number is at address 0 for all devices
        p = ParamWord(0, 'model_number', readonly=True)
        req = Packet(ident, READ_DATA, [p.address, p.size])
        resp = self.io._request(req, Packet.length_total_with_no_payload + p.size)
        return p.frompayload(resp.payload)

    def sync_write(self):
        """Write parameters on the same address and with the same length to
        multiple devices simultaneously
        """
        raise NotImplementedError()

    def bulk_read(self):
        """Read parameters of multiple devices from different addresses with
        different lengths simultaneously

        Only for MX series Dynamixels
        """
        raise NotImplementedError()

