import logging
import argparse
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.server import (
    StartSerialServer
)

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusServerContext,
    ModbusSlaveContext,
    ModbusSparseDataBlock,
)
_logger = logging.getLogger(__file__)
_logger.setLevel("DEBUG")
# _logger.setLevel(logging.INFO)
from pymodbus import pymodbus_apply_logging_config
# from pymodbus.transaction import ModbusRtuFramer


class CallbackDataBlock(ModbusSequentialDataBlock):
    """A datablock that stores the new value in memory,.

    and passes the operation to a message queue for further processing.
    """

    def __init__(self, addr, values):
        """Initialize."""
        # self.queue = queue , queue'
        print("CallbackDataBlock")
        super().__init__(addr, values)

    def setValues(self, address, value):
        """Set the requested values of the datastore."""
        super().setValues(address, value)
        txt = f"Callback from setValues with address {address}, value {value}"
        _logger.debug(txt)

    def getValues(self, address, count=1):
        """Return the requested values from the datastore."""
        holding_registers = super().getValues(address, count)  # Function code 3 for holding registers
        incremented_values = [value + 1 for value in holding_registers[:min(count, 4)]]
        super().setValues(address, incremented_values)
        result = super().getValues(address, count=count)
        txt = f"Callback from getValues with address {address}, count {count}, data {result}"
        _logger.debug(txt)
        print("getValues", result)
        return result

    def validate(self, address, count=1):
        """Check to see if the request is in range."""
        result = super().validate(address, count=count)
        txt = f"Callback from validate with address {address}, count {count}, data {result}"
        _logger.debug(txt)
        return result

# Initialize the holding registers with values from 0x0 to 0x10
# initial_values = {i: 0 for i in range(0x0, 0x11)}
# initial_values = ModbusSequentialDataBlock(0x00, [1] * 10, )
initial_values = CallbackDataBlock(0x00, [0] * 0x20, )

# Create a Modbus slave context with the initial values
store = ModbusSlaveContext(
    hr=initial_values  # Addressing starts at 0
)

# Wrap the slave context in a server context
context = ModbusServerContext(slaves=store, single=True)


# Configure Modbus device identification
identity = ModbusDeviceIdentification()
identity.VendorName = 'ExampleVendor'
identity.ProductCode = 'ModbusServer'
identity.VendorUrl = 'http://example.com'
identity.ProductName = 'Modbus Serial Server'
identity.ModelName = 'ModbusServerModel'
identity.MajorMinorRevision = '1.0'

# Start the Modbus serial server
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Start Modbus Serial Server")
    parser.add_argument("--port", required=True, help="Serial port to use for the Modbus server")
    args = parser.parse_args()

    print("Starting Modbus Serial Server...")
    pymodbus_apply_logging_config("DEBUG")
    StartSerialServer(
        context,
        framer="rtu",
        identity=identity,
        port=args.port,  # Use the port from the command-line argument
        baudrate=115200,
        stopbits=1,
        bytesize=8,
        parity='N',
        timeout=1,
    )
