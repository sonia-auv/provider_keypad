########## Bus - bus.py ##########
# Original Author: John Zeller
# Description: Bus initializes the port(s) for use, and offers them up as easy
#	       to access attributes. Additionally, there is the option to reset
#	       the port(s) by using the restart function.


class Bus(object):
    def __init__(self):
        self.gamepad = None
        self._open_serial_port()

    def restart(self):
        self.gamepad.close()
        self._open_serial_port()

    def _open_serial_port(self):
        self.gamepad = open('/dev/input/js0', 'r')
