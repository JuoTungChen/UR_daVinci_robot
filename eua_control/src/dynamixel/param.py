
class Param(object):
    def __init__(self, address, name, size, values_raw=None, range_raw=None, to_human=None, to_device=None, readonly=False):
        """A representation of a single parameter on a Dynamixel device
        """
        self.address = address
        self.name = name
        self.size = size
        self.values_raw = values_raw
        self.range_raw = range_raw if range_raw else (0, 2**size - 1)
        self.to_human = to_human
        self.to_device = to_device
        self.readonly = readonly

    def __repr__(self):
        return "Param(address={}, name='{}', size={})".format(self.address, self.name, self.size)

    def __hash__(self):
        return hash((self.address, self.name, self.size))

    def __eq__(self, other):
        if not isinstance(other, type(self)):
            return NotImplemented

        return (self.address == other.address) and (self.name == other.name) and (self.size == other.size)


class ParamByte(Param):
    def __init__(self, address, name, values_raw=None, range_raw=None, to_human=None, to_device=None, readonly=False):
        super(ParamByte, self).__init__(address, name, 1, values_raw, range_raw, to_human, to_device, readonly)
        self.frompayload = lambda x: x[0]
        self.topayload = lambda x: [x]


class ParamWord(Param):
    def __init__(self, address, name, values_raw=None, range_raw=None, to_human=None, to_device=None, readonly=False):
        super(ParamWord, self).__init__(address, name, 2, values_raw, range_raw, to_human, to_device, readonly)
        self.frompayload = lambda x: (x[1] << 8) | x[0]  # concatenate bytes
        self.topayload = lambda x: [x & 0xff, (x >> 8) & 0xff]  # split into separate bytes, little endian (LSB first)


class ParamDoubleWord(Param):
    def __init__(self, address, name, values_raw=None, range_raw=None, to_human=None, to_device=None, readonly=False):
        super(ParamDoubleWord, self).__init__(address, name, 4, values_raw, range_raw, to_human, to_device, readonly)
        self.frompayload = lambda x: (x[3] << 24) | (x[2] << 16) | (x[1] << 8) | x[0]
        self.topayload = lambda x: [x & 0xff, (x >> 8) & 0xff, (x >> 16) & 0xff, (x >> 24) & 0xff]
