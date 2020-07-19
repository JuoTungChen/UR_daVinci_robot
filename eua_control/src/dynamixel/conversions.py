import math


_two_pi = 2 * math.pi


baud_map = {
    0: 9600,
    1: 57600,
    2: 115200,
    3: 1000000,
    4: 2000000,
    5: 3000000,
    6: 4000000,
    7: 4500000,
}
baud_map.update(dict(reversed(i) for i in baud_map.items()))  # reverse mapping


def tc_bin2int(val, n):
    """Return the two's complement integer of an n-bit binary value
    """
    if (val & (1 << (n - 1))): # left-most bit is set
        val = val - (1 << n)

    return val


def tc_int2bin(val, n):
    """Return the binary representation of an n-bit two's complement int value
    """
    return val % (1 << n)


def dir2signed(word):
    val = word & 0x3ff  # magnitude is the 10 least significant bits
    cw = word & 0x400  # bit 10 is set if rotating to the clockwise direction
    return -val if cw else val


def signed2dir(val):
    word = abs(val) & 0x3ff  # magnitude is the 10 least significant bits

    if val < 0:
        word = (1 << 10) | word  # bit 10 is set if rotating to the clockwise direction

    return word


def rpm2radsec(x):
    # rev/min to rad/s
    return _two_pi / 60 * x


def radsec2rpm(x):
    # rad/s to rev/min
    return 60 / _two_pi * x


def rpmsq2radsecsq(x):
    # rev/min^2 to rad/s^2
    return _two_pi / 3600 * x


def radsecsq2rpmsq(x):
    # rad/s^2 to rev/min^2
    return 3600 / _two_pi * x


def cslope2step(val):
    if 0 <= val <= 3:
        return 1
    elif 4 <= val <= 7:
        return 2
    elif 8 <= val <= 15:
        return 3
    elif 16 <= val <= 31:
        return 4
    elif 32 <= val <= 63:
        return 5
    elif 64 <= val <= 127:
        return 6
    elif 128 <= val <= 254:
        return 7


def step2cslope(step):
    if step == 1:
        return 2
    elif step == 2:
        return 4
    elif step == 3:
        return 8
    elif step == 4:
        return 16
    elif step == 5:
        return 32
    elif step == 6:
        return 64
    elif step == 7:
        return 128
