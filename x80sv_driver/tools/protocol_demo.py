import sys
import logging
import struct
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton
from PyQt5.QtWidgets import QVBoxLayout, QFormLayout
from PyQt5.QtCore import QIODevice
from PyQt5.QtSerialPort import QSerialPort


# Force python 3:
assert sys.version_info.major == 3


class Protocol:
    """ Protocol packetizer
        0x5E 0x02 marks start
        0x5E 0x0D marks end
      #define INDEX_STX0              0
      #define INDEX_STX1              1
      #define INDEX_DES               2
      #define INDEX_SN                3
      #define INDEX_TYPE              4
      #define INDEX_LENGTH            5
      #define INDEX_DATA              6

  #define DATA_ACK                        0x01
  #define DATA_PING                       0x00
  #define DATA_URGENT_DATA        0x02
  #define DATA_SKIPPACKET         0x03
    """
    def __init__(self):
        self.logger = logging.getLogger('protocol')
        self.state = 0
        self.process_msg = None
        self.msg = bytearray()
        self.type_map = {}
        self.type_map[0xff] = 'system'
        self.type_map[0x7f] = 'sensor'
        self.type_map[40] = 'motor'

    def check_msg(self):
        dst = self.msg[0]
        sn = self.msg[1]
        typ = self.msg[2]
        ln = self.msg[3]
        data = self.msg[4:-1]
        # crc may be last byte?
        if ln == len(data):
            self.logger.debug('Valid packet: dst={}, sn={}, typ={}'.format(dst, sn, typ))

            # Check if callback is registered, and if so, call it:
            if self.process_msg:
                self.process_msg(typ, data)
        else:
            self.logger.error('Message length {} != {}'.format(ln, len(self.msg)))

    def handle_byte(self, b):
        if self.state == 0:
            if b == 0x5E:
                self.state = 1
            else:
                # Ignore byte
                pass
        elif self.state == 1:
            if b == 0x02:
                # Start of packet!
                self.msg = bytearray()
                self.state = 2
            else:
                self.state = 0
        elif self.state == 2:
            if b == 0x5e:
                # End of packet
                self.state = 3
            else:
                # data byte:
                self.msg.append(b)
        elif self.state == 3:
            if b == 0xD:
                # self.logger.debug('Got packet: {}'.format(self.msg))
                self.check_msg()
                self.state = 0
            else:
                self.logger.error('Invalid end of packet')
                self.state = 0
        else:
            raise NotImplementedError()

    def add_bytes(self, data):
        for b in data:
            self.handle_byte(b)


class DataPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QFormLayout(self)

    def add_properties(self, names):
        for prop in names:
            self.add_prop(prop)
        
    def add_prop(self, name):
        label = QLabel('NaN')
        self.layout.addRow(name, label)
        def getter(s):
            return label.text()
        def setter(s, v):
            label.setText(str(v))
        prop = property(getter, setter)
        setattr(self.__class__, name, prop)


class MotorDataPanel(DataPanel):
    def __init__(self):
        super().__init__()
        self.add_properties(['enc0', 'enc1', 'pos0', 'pos1'])


class InfoDataPanel(DataPanel):
    def __init__(self):
        super().__init__()
        self.add_properties(['msgcnt'])


class ControlPanel(QWidget):
    """ This control panel steers the dr robot protocol.
        It opens the serial port, and handles the protocol.
    """
    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger('controlpanel')
        l = QVBoxLayout(self)
        b = QPushButton("Do")
        b.clicked.connect(self.do_connect)
        l.addWidget(b)
        self.motor_panel = MotorDataPanel()
        self.info_panel = InfoDataPanel()
        l.addWidget(self.info_panel)
        l.addWidget(self.motor_panel)

        self.protocol = Protocol()
        self.protocol.process_msg = self.process_msg

        # Construct port:
        self.port = QSerialPort(self)
        self.port.readyRead.connect(self.do_read_bytes)
        self.logger.debug('Created class')

        # open port:
        self.do_open()

    def closeEvent(self, event):
        self.port.close()
        super().closeEvent(event)

    def do_open(self):
        self.logger.debug('Opening port')
        self.port.setPortName('/dev/ttyUSB0')
            
        self.logger.debug('Port name {}'.format(self.port.portName()))
        self.logger.debug('Baud rate: {}'.format(self.port.baudRate()))
        self.logger.debug('Data bits: {}'.format(self.port.dataBits()))
        res = self.port.open(QIODevice.ReadWrite)
        if not res:
            self.logger.error('Error opening port {}'.format(self.port.error()))
        if not self.port.setBaudRate(115200):
            self.logger.error('Error setting baudrate: {}'.format(self.port.error()))
        self.logger.debug('Port opened')
        self.logger.debug('Port name {}'.format(self.port.portName()))
        self.logger.debug('Baud rate: {}'.format(self.port.baudRate()))

    def do_read_bytes(self):
        data = self.port.read(100)
        # self.logger.debug('Read {} bytes'.format(data))
        self.protocol.add_bytes(data)

    def process_msg(self, typ, data):
        """
            Dissect the contents of the message.
              #define COMTYPE_MOTOR           40
        """
        COMTYPE_SYSTEM = 0xFF
        COMTYPE_MOTOR_SENSOR = 0x7B
        COMTYPE_CUSTOM_SENSOR = 0x7C
        COMTYPE_STANDARD_SENSOR = 0x7D
        COMTYPE_SENSOR = 0x7F
        if typ == COMTYPE_MOTOR_SENSOR:
            # format: 6x angles, 6x current, pos, vel, pos, vel
            enc0, enc1, _, _, _, _, cur0, cur1, _, _, _, _, pos0, vel0, pos1, vel1, bitmask = struct.unpack('<HHH HHH HHH HHH H H H H B', data)
            self.motor_panel.pos0 = pos0
            self.motor_panel.pos1 = pos1
            self.motor_panel.enc0 = enc0
            self.motor_panel.enc1 = enc1
        elif typ == COMTYPE_SENSOR:
            self.logger.warning('Sensor data')
        elif typ == COMTYPE_CUSTOM_SENSOR:
            self.logger.warning('Custom sensor data')
        elif typ == COMTYPE_STANDARD_SENSOR:
            self.logger.warning('Standard sensor data')
        else:
            self.logger.warning('Unknown packet type {}'.format(typ))

    def do_connect(self):
        self.logger.debug('Connect!')


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s|%(levelname)8s|%(name)0.8s|%(message)s')
    app = QApplication(sys.argv)
    w = ControlPanel()
    w.show()
    app.exec()
