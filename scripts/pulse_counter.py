#!/usr/bin/env python3
import smbus


class MCP23017(object):
    """
    Description:
        Class for communicating with a Microchip MCP23017 via i2c interface.

    Args:
        i2c_bus (int): i2c bus number
        i2c_addr (byte): i2c bus address
    """
    def __init__(self, i2c_bus=1, i2c_addr=b'\x20'):

        self.i2c_bus = i2c_bus
        self.i2c_addr = i2c_addr

        # MCP23017 Registers
        self.MCP23017_IODIRA = b'\x00'
        self.MCP23017_IPOLA = b'\x02'
        self.MCP23017_GPINTENA = b'\x04'
        self.MCP23017_DEFVALA = b'\x06'
        self.MCP23017_INTCONA = b'\x08'
        self.MCP23017_IOCONA = b'\x0A'
        self.MCP23017_GPPUA = b'\x0C'
        self.MCP23017_INTFA = b'\x0E'
        self.MCP23017_INTCAPA = b'\x10'
        self.MCP23017_GPIOA = b'\x12'
        self.MCP23017_OLATA = b'\x14'

        self.MCP23017_IODIRB = b'\x01'
        self.MCP23017_IPOLB = b'\x03'
        self.MCP23017_GPINTENB = b'\x05'
        self.MCP23017_DEFVALB = b'\x07'
        self.MCP23017_INTCONB = b'\x09'
        self.MCP23017_IOCONB = b'\x0B'
        self.MCP23017_GPPUB = b'\x0D'
        self.MCP23017_INTFB = b'\x0F'
        self.MCP23017_INTCAPB = b'\x11'
        self.MCP23017_GPIOB = b'\x13'
        self.MCP23017_OLATB = b'\x15'

        self.MCP_OUTPUTS = b'\x00'
        self.MCP_INPUTS = b'\xFF'
        self.MCP_PULLUP_NONE = b'\x00'
        self.MCP_PULLUP_ALL = b'\xFF'

        try:
            # Try to open I2C bus
            self.bus = smbus.SMBus(self.i2c_bus)

        except Exception as e:
            print(f"could not open i2c bus: {e}")

    def set_direction(self, bank, value):
        if bank in ['a', 'A', self.MCP23017_IODIRA]:
            self.write_register(self.MCP23017_IODIRA, value)

        elif bank in ['b', 'B', self.MCP23017_IODIRB]:
            self.write_register(self.MCP23017_IODIRB, value)

    def set_direction_bit(self, bank, bit_number, bit_value):
        register = -1

        if bank in ['a', 'A', self.MCP23017_IODIRA]:
            register = self.MCP23017_IODIRA

        elif bank in ['b', 'B', self.MCP23017_IODIRB]:
            register = self.MCP23017_IODIRB

        if register != -1:
            self.write_register_bit(register, bit_number, bit_value)

    def set_pull_up(self, bank, value):
        if bank in ['a', 'A', self.MCP23017_GPPUA]:
            self.write_register(self.MCP23017_GPPUA, value)

        elif bank in ['b', 'B', self.MCP23017_GPPUB]:
            self.write_register(self.MCP23017_GPPUB, value)

    def set_pull_up_bit(self, bank, bit_number, bit_value):
        register = -1

        if bank in ['a', 'A', self.MCP23017_GPPUA]:
            register = self.MCP23017_GPPUA

        elif bank in ['b', 'B', self.MCP23017_GPPUB]:
            register = self.MCP23017_GPPUB

        if register != -1:
            self.write_register_bit(register, bit_number, bit_value)

    def set_polarity(self, bank, value):
        if bank in ['a', 'A', self.MCP23017_IPOLA]:
            self.write_register(self.MCP23017_IPOLA, value)

        elif bank in ['b', 'B', self.MCP23017_IPOLB]:
            self.write_register(self.MCP23017_IPOLB, value)

    def set_bit_polarity(self, bank, bit_number, bit_value):
        register = -1

        if bank in ['a', 'A', self.MCP23017_IPOLA]:
            register = self.MCP23017_IPOLA

        elif bank in ['b', 'B', self.MCP23017_IPOLB]:
            register = self.MCP23017_IPOLB

        if register != -1:
            self.write_register_bit(register, bit_number, bit_value)

    def enable_interrupt(self, bank, value):
        if bank in ['a', 'A', self.MCP23017_GPINTENA]:
            self.write_register(self.MCP23017_GPINTENA, value)

        elif bank in ['b', 'B', self.MCP23017_GPINTENB]:
            self.write_register(self.MCP23017_GPINTENB, value)

    def enable_interrupt_bit(self, bank, bit_number, bit_value):
        register = -1

        if bank in ['a', 'A', self.MCP23017_GPINTENA]:
            register = self.MCP23017_GPINTENA

        elif bank in ['b', 'B', self.MCP23017_GPINTENB]:
            register = self.MCP23017_GPINTENB

        if register != -1:
            self.write_register_bit(register, bit_number, bit_value)

    def set_interrupt_control(self, bank, value):
        if bank in ['a', 'A', self.MCP23017_INTCONA]:
            self.write_register(self.MCP23017_INTCONA, value)

        elif bank in ['b', 'B', self.MCP23017_INTCONB]:
            self.write_register(self.MCP23017_INTCONB, value)

    def set_interrupt_control_bit(self, bank, bit_number, bit_value):
        register = -1

        if bank in ['a', 'A', self.MCP23017_INTCONA]:
            register = self.MCP23017_INTCONA

        elif bank in ['b', 'B', self.MCP23017_INTCONB]:
            register = self.MCP23017_INTCONB

        if register != -1:
            self.write_register_bit(register, bit_number, bit_value)

    def read_io_bank(self, bank):
        if bank in ['a', 'A', self.MCP23017_GPIOA]:
            return self.read_register(self.MCP23017_GPIOA)

        elif bank in ['b', 'B', self.MCP23017_GPIOB]:
            return self.read_register(self.MCP23017_GPIOB)

        else:
            return -1

    def read_io_bit(self, bank, bit_number):
        if bank in ['a', 'A', self.MCP23017_GPIOA]:
            return self.read_io_bit(bank=self.MCP23017_GPIOA, bit_number=bit_number)

        elif bank in ['b', 'B', self.MCP23017_GPIOB]:
            return self.read_io_bit(bank=self.MCP23017_GPIOB, bit_number=bit_number)

        else:
            return -1

    def write_io_bank(self, bank, value):
        if bank in ['a', 'A', self.MCP23017_GPIOA]:
            self.write_register(self.MCP23017_GPIOA, value)

        elif bank in ['b', 'B', self.MCP23017_GPIOB]:
            self.write_register(self.MCP23017_GPIOB, value)

    def write_io_bit(self, bank, bit_number, bit_value):
        register = -1

        if bank in ['a', 'A', self.MCP23017_GPIOA]:
            register = self.MCP23017_GPIOA

        elif bank in ['b', 'B', self.MCP23017_GPIOB]:
            register = self.MCP23017_GPIOB

        if register != -1:
            self.write_register_bit(register, bit_number, bit_value)

    def read_register_bit(self, register, bit_number):
        if (self.read_register(register=register) & (1 << bit_number)) == 0:
            return False
        else:
            return True

    def write_register_bit(self, register, bit_number, bit_value):
        register_value = self.read_register(register=register)

        if bit_value:
            register_value |= (1 << bit_number)
        else:
            register_value &= ~(1 << bit_number)

        self.write_register(register=register, value=register_value)

    def read_register(self, register):
        return self.bus.read_byte_data(self.i2c_addr, register)

    def read_register_block(self, register, num_bytes):
        return self.bus.read_i2c_block_data(self.i2c_addr, register, num_bytes)

    def write_register(self, register, value):
        self.bus.write_byte_data(self.i2c_addr, register, value)

    def write_register_block(self, register, values):
        self.bus.write_byte_data(self.i2c_addr, register, values)


class PulseCounter(MCP23017):
    """
    Description:
        Class reading a Texas Instruments SN74LV8154 dual 16bit or single 32bit counter with a Microchip MCP23017.

    Args:
        i2c_bus (int): i2c bus number for the MCP23017 Class
        i2c_addr (byte): i2c bus address for the MCP23017 Class
    """
    def __init__(self, i2c_bus=1, i2c_addr=b'\x20'):
        super().__init__(i2c_bus=i2c_bus, i2c_addr=i2c_addr)

        # SN74LV8154 control pins
        self.control_bank = 'A'
        self.data_bank = 'B'
        self.control_register = self.MCP23017_GPIOA
        self.data_register = self.MCP23017_GPIOB
        self.set_all = b'\xff'
        self.select_GAL = b'\xdf'  # GAL, GAU,
        self.select_GAU = b'\xef'
        self.select_GBL = b'\xf7'
        self.select_GBU = b'\xfb'
        self.CCLR = 0

        # Control bank are outputs, no pull-ups
        self.set_direction(bank=self.control_bank, value=self.MCP_OUTPUTS)
        self.set_pull_up(bank=self.control_bank, value=self.MCP_PULLUP_NONE)

        # Set all on except CCLR
        self.write_register(register=self.control_register, value=b'\xfe')

        # Set CCLR to trigger reset
        self.write_register(register=self.control_register, value=self.set_all)

        # Reset RCLK to allow latching
        self.write_register(register=self.control_register, value=b'\xfd')

        # data bank are inputs, no pull-ups
        self.set_direction(self.data_bank, self.MCP_INPUTS)
        self.set_pull_up(self.data_bank, self.MCP_PULLUP_NONE)

    def reset_count(self):
        self.write_register_bit(register=self.control_register, bit_number=self.CCLR, bit_value=False)
        self.write_register_bit(register=self.control_register, bit_number=self.CCLR, bit_value=True)

    def read_16bit_counters(self, reset_after_read=False):

        # Set RCLK to latch counter to buffer
        self.write_register(register=self.control_register, value=self.set_all)

        if reset_after_read:
            # Cycle the CCLR output
            self.write_register(register=self.control_register, value=b'\xfe')
            self.write_register(register=self.control_register, value=self.set_all)

        # Reset GBU
        self.write_register(register=self.control_register, value=self.select_GBU)
        new_counter_value_b = self.read_register(register=self.data_register) << 8

        # Set GBU, Reset GBL
        self.write_register(register=self.control_register, value=self.select_GBL)
        new_counter_value_b += self.read_register(register=self.data_register)

        # Set GBL, Reset GAU
        self.write_register(register=self.control_register, value=self.select_GAU)
        new_counter_value_a = self.read_register(register=self.data_register) << 8

        # Set GAU, Reset GAL
        self.write_register(register=self.control_register, value=self.select_GAL)
        new_counter_value_a += self.read_register(register=self.data_register)

        # Set GAL, Reset RCLK
        self.write_register(register=self.control_register, value=b'\xfd')

        return new_counter_value_a, new_counter_value_b

    def read_32bit_counter(self, reset_after_read=False):

        # Set RCLK to latch counter to buffer
        self.write_register(register=self.control_register, value=self.set_all)

        if reset_after_read:
            # Cycle the RCLK output
            self.write_register(register=self.control_register, value=b'\xfe')
            self.write_register(register=self.control_register, value=self.set_all)

        # Reset GBU
        self.write_register(register=self.control_register, value=self.select_GBU)
        new_counter_value = self.read_register(register=self.data_register) << 24

        # Set GBU, Reset GBL
        self.write_register(register=self.control_register, value=self.select_GBL)
        new_counter_value += self.read_register(register=self.data_register) << 16

        # Set GBL, Reset GAU
        self.write_register(register=self.control_register, value=self.select_GAU)
        new_counter_value += self.read_register(register=self.data_register) << 8

        # Set GAU, Reset GAL
        self.write_register(register=self.control_register, value=self.select_GAL)
        new_counter_value += self.read_register(register=self.data_register)

        # Set GAL, Reset RCLK
        self.write_register(register=self.control_register, value=b'\xfd')

        return new_counter_value

