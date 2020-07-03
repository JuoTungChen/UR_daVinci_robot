# Protocol 1.0 packet handling

# Instruction set
PING       = 1
READ_DATA  = 2
WRITE_DATA = 3
REG_WRITE  = 4
ACTION     = 5
RESET      = 6
SYNC_WRITE = 131  # 0x83
BULK_READ  = 146  # 0x92 (can only be used with MX series)

# USB2AX extended instructions
USB2AX_BOOTLOADER = 8  # Reboot USB2AX in bootloader mode
USB2AX_SYNC_READ  = 132  # Bulk read from several Dynamixels simultaneously

# Status return levels
RETURN_NONE = 0
RETURN_READ = 1
RETURN_ALL  = 2

# Error bits
ERROR_INPUT_VOLTAGE = (1 << 0)  # The applied voltage is out of the range of operating voltage set in the Control table
ERROR_ANGLE_LIMIT   = (1 << 1)  # Goal Position written is out of the range from CW Angle Limit to CCW Angle Limit
ERROR_OVERHEATING   = (1 << 2)  # Internal temperature of Dynamixel is out of the range of operating temperature set in the Control table
ERROR_RANGE         = (1 << 3)  # A command is out of the range for use
ERROR_CHECKSUM      = (1 << 4)  # The Checksum of the transmitted Instruction Packet is incorrect
ERROR_OVERLOAD      = (1 << 5)  # The current load cannot be controlled by the set Torque
ERROR_INSTRUCTION   = (1 << 6)  # Sending an undefined instruction or delivering the action command without the reg_write command


class DeserializationError(Exception):
    def __init__(self, message, raw_data):
        super(DeserializationError, self).__init__(message)
        self.bytes = raw_data

class MissingDataError(DeserializationError):
    pass

class ChecksumError(DeserializationError):
    pass


class ReturnError(Exception):
    def __init__(self, device_id, message):
        super(ReturnError, self).__init__(message)
        self.id = device_id

class ReturnInputVoltageError(ReturnError):
    pass

class ReturnAngleLimitError(ReturnError):
    pass

class ReturnOverheatingError(ReturnError):
    pass

class ReturnRangeError(ReturnError):
    pass

class ReturnChecksumError(ReturnError):
    pass

class ReturnOverloadError(ReturnError):
    pass

class ReturnInstructionError(ReturnError):
    pass


class Packet(object):
    # Protocol 1.0 packet fields
    HEADER0     = 0
    HEADER1     = 1
    ID          = 2
    LENGTH      = 3
    INSTRUCTION = 4
    ERROR       = 4
    PAYLOAD0    = 5

    # Instruction packet format:
    #  [0xff, 0xff, ID, LENGTH, INSTRUCTION, PAYLOAD 0, PAYLOAD 1, ..., PAYLOAD N, CHECKSUM]
    #    ------------    -------------------------------------------------------    ------
    #          |          ----------------          |                                  |
    #        Header               |              Message                            Trailer
    #                       Message header

    # Status (return) packet format:
    #  [0xff, 0xff, ID, LENGTH, ERROR, PAYLOAD 0, PAYLOAD 1, ..., PAYLOAD N, CHECKSUM]
    #    ------------    --------------------------------------------------    ------
    #          |          -----------            |                               |
    #        Header            |              Message                         Trailer
    #                    Message header

    # The LENGTH field indicates the length of the "Message" part

    length_header  = 3
    length_message_header = 2
    length_trailer = 1
    length_header_trailer = length_header + length_trailer
    length_total_with_no_payload = length_header_trailer + length_message_header

    def __init__(self, ident=None, instruction=None, payload=[]):
        self.id = ident
        self.instruction = instruction
        self.payload = payload

    def __str__(self):
        fmt = ("Packet\n"
               "  id: {:d}\n"
               "  length: {:d}\n"
               "  instruction/error: {:d}\n"
               "  payload: {}\n"
               "  checksum: {:d}")
        return fmt.format(self.id, self.length_message(), self.instruction, self.payload, self.checksum())

    def length_message(self):
        """Message length (in bytes) excluding packet header and trailer
        """
        return len(self.payload) + self.length_message_header

    def length_total(self):
        """Total length (in bytes) of the serialized message
        """
        return self.length_header_trailer + self.length_message()

    def checksum(self):
        """Checksum of the packet
        """
        return 0xff - ((self.id + self.length_message() + self.instruction + sum(self.payload)) & 0xff)

    def serialize(self):
        """Serialize to string of bytes
        """
        data = [0xff, 0xff, self.id, self.length_message(), self.instruction] + self.payload + [self.checksum()]
        return bytes(bytearray(data))

    def deserialize(self, raw_data):
        """De-serialize from string of bytes
        """
        if not raw_data or len(raw_data) < self.length_header_trailer:
            raise MissingDataError("No header data", raw_data)

        d = bytearray(raw_data)

        # Verify packet header
        if d[Packet.HEADER0] != 0xff or d[Packet.HEADER1] != 0xff:
            raise MissingDataError("Bad packet begin signal", raw_data)

        self.id = d[Packet.ID]
        self.instruction = d[Packet.INSTRUCTION] # a.k.a. error

        # Length
        dlen = self.length_header_trailer + d[Packet.LENGTH]

        if dlen > len(d):
            raise MissingDataError("Not enough data to de-serialize whole packet", raw_data)

        # Checksum
        chk = 0xff - (sum(d[Packet.ID:dlen-1]) & 0xff)

        if chk != d[dlen-1]:
            raise ChecksumError("The checksum of the returned Status Packet is incorrect", raw_data)

        # Check packet error bits
        if d[Packet.ERROR] & ERROR_INPUT_VOLTAGE:
            raise ReturnInputVoltageError(self.id, "The applied voltage is out of the range of operating voltage set in the control table")
        if d[Packet.ERROR] & ERROR_ANGLE_LIMIT:
            raise ReturnAngleLimitError(self.id, "Goal position written is out of the range from CW angle limit to CCW angle limit")
        if d[Packet.ERROR] & ERROR_OVERHEATING:
            raise ReturnOverheatingError(self.id, "Internal temperature of Dynamixel is out of the range of operating temperature set in the control table")
        if d[Packet.ERROR] & ERROR_RANGE:
            raise ReturnRangeError(self.id, "A command is out of the range for use")
        if d[Packet.ERROR] & ERROR_CHECKSUM:
            raise ReturnChecksumError(self.id, "The checksum of the transmitted instruction packet is incorrect")
        if d[Packet.ERROR] & ERROR_OVERLOAD:
            raise ReturnOverloadError(self.id, "The current load cannot be controlled by the set torque")
        if d[Packet.ERROR] & ERROR_INSTRUCTION:
            raise ReturnInstructionError(self.id, "Sending an undefined instruction or delivering the action command without the reg_write command")

        self.payload = list(d[Packet.PAYLOAD0:dlen-1])

        return self
