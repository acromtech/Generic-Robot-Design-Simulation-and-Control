"""
Class to interface with a CAN bus using the GS_USB adapter.

Dependencies:
- gs_usb: Python library for interacting with GS_USB CAN adapters.
  Install with: pip install gs_usb

Usage:
1. Create an instance of CanBusGsUsb by providing the port number and bitrate.
2. Send messages using sendMessage method.
3. Optionally, set up and shut down the device with setup and shutdown methods.
"""

import os

# Install the required dependency
os.system("pip install gs_usb")

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import (
    CAN_EFF_FLAG,
    CAN_ERR_FLAG,
    CAN_RTR_FLAG,
)

class CanBusGsUsb:

    def __init__(self, port: int, bitrate: int):
        """
        Initialize the CanBusGsUsb instance with port number and bitrate.

        Parameters:
            port (int): Port number of the GS_USB adapter.
            bitrate (int): Bitrate for the CAN bus communication.
        """
        self.port = port
        self.bitrate = bitrate

        # Find the GS_USB device
        self.devs = GsUsb.scan()
        if len(self.devs) == 0:
            print("Cannot find gs_usb device")
        self.dev = self.devs[self.port]

        # Configuration
        if not self.dev.set_bitrate(self.bitrate):
            print("Cannot set bitrate for gs_usb")
        else:
            print("Bitrate set to", self.bitrate)

    def sendMessage(self, id, data, debug = None):
        """
        Send a CAN message.

        Parameters:
            id: Message ID.
            data: Message data.
        """
        cpt = 0
        frame = GsUsbFrame(id, data)
        self.dev.send(frame)  # Send message
        while True:  # Read all the time
            iframe = GsUsbFrame()
            if self.dev.read(iframe, 1):
                if frame.data[0] == iframe.data[0]:
                    cpt = cpt + 1
                    if cpt == 1:
                        if debug!=None: print("TX  {}".format(frame))
                    else:
                        if debug!=None: print("RX  {}".format(iframe))
                        return iframe

    def setup(self):
        """
        Set up the GS_USB device.
        """
        self.dev.start()

    def shutdown(self):
        """
        Shut down the GS_USB device.
        """
        self.dev.stop()

    def set_port(self, port):
        """
        Set the port number.

        Parameters:
            port (int): Port number.
        """
        self.port = port

    def set_bitrate(self, bitrate):
        """
        Set the bitrate.

        Parameters:
            bitrate (int): Bitrate for CAN communication.
        """
        self.bitrate = bitrate
