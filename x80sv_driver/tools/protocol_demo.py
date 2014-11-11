import sys
import logging
import struct
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton
from PyQt5.QtWidgets import QVBoxLayout, QFormLayout
from PyQt5.QtCore import QIODevice
from PyQt5.QtSerialPort import QSerialPort


# Force python 3:
assert sys.version_info.major == 3

def crc8(data):
    register = 0
    for d in data:
        for _ in range(8):
            data_bit = d & 0x1
            sr_lsb = register & 0x1
            fb_bit = (data_bit ^ sr_lsb)

            # Update registers:
            register >>= 1
            if fb_bit == 1:
                register ^= 0x8C
            d >>= 1
            register &= 0xFF
            print('register={}'.format(hex(register)))
    print(hex(register))
    return register
    

assert crc8('123456789'.encode('ascii')) == 0xcc

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

  const unsigned char COM_TYPE_MOT = 0x01;
  const unsigned char COM_TYPE_MOT_PLUS = 0x8b;         // for Sentinel3 Hawk, and H20
    """
    COM_STX0 = 0x5E
    COM_STX1 = 0x02
    COM_ETX0 = 0x5E
    COM_ETX1 = 0x0D
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
            # self.logger.debug('Valid packet: dst={}, sn={}, typ={}'.format(dst, sn, typ))

            # Check if callback is registered, and if so, call it:
            if self.process_msg:
                self.process_msg(typ, data)
        else:
            self.logger.error('Message length {} != {}'.format(ln, len(self.msg)))

    def handle_byte(self, b):
        if self.state == 0:
            if b == self.COM_STX0:
                self.state = 1
            else:
                # Ignore byte
                pass
        elif self.state == 1:
            if b == self.COM_STX1:
                # Start of packet!
                self.msg = bytearray()
                self.state = 2
            else:
                self.state = 0
        elif self.state == 2:
            if b == self.COM_ETX0:
                # End of packet
                self.state = 3
            else:
                # data byte:
                self.msg.append(b)
        elif self.state == 3:
            if b == self.COM_ETX1:
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

    def make_msg(self, typ, data, dst=0, sn=0):
        data = bytearray(data)
        msg = bytearray([self.COM_STX0, self.COM_STX1, dst, sn, typ, len(data)]) + data + bytearray([0, self.COM_ETX0, self.COM_ETX1])
        # TODO: calculate CRC
        return msg


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
        self.add_properties(['enc0', 'enc1', 'pos0', 'pos1', 'vel0', 'vel1'])


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
        b = QPushButton("Gas")
        b.clicked.connect(self.do_go)
        l.addWidget(b)

        b2 = QPushButton("Bremsen")
        b2.clicked.connect(self.do_stop)
        l.addWidget(b2)

        self.motor_panel = MotorDataPanel()
        self.info_panel = InfoDataPanel()
        l.addWidget(self.info_panel)
        l.addWidget(self.motor_panel)

        self.protocol = Protocol()
        self.protocol.process_msg = self.process_msg
        self.msg_cntr = 0

        # Construct port:
        self.port = QSerialPort(self)
        self.port.readyRead.connect(self.do_read_bytes)
        self.logger.debug('Created class')

        # open port:
        self.do_open()
        self.do_go()

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
        self.msg_cntr += 1
        self.info_panel.msgcnt = self.msg_cntr
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
            self.motor_panel.vel0 = vel0
            self.motor_panel.vel1 = vel1
        elif typ == COMTYPE_SENSOR:
            #self.logger.warning('Sensor data')
            pass
        elif typ == COMTYPE_CUSTOM_SENSOR:
            #self.logger.warning('Custom sensor data')
            pass
        elif typ == COMTYPE_STANDARD_SENSOR:
            #self.logger.warning('Standard sensor data')
            pass
        else:
            self.logger.warning('Unknown packet type {}'.format(typ))

    def send_msg(self, typ, data):
        msg = self.protocol.make_msg(typ, data)
        print(msg)

    def set_motor_velctrl_pid(self, channel, kp, ki, kd):
        """
  const unsigned char POSITIONPID = 7;
  const unsigned char MOTORVELOCITYCTRL = 26;
  const unsigned char MOTORVELOCITYCTRLALL = 27;

  const unsigned char SERVOCTRL = 28;
  const unsigned char SERVOCTRLALL = 29;
  const unsigned char MOTORENABLE = 0x1e;

  const unsigned char MOTORFRICCOMP = 31;
  const unsigned char CUSTOMIO = 22;
  const unsigned char POWERCTRL = 22;

        """
        MOTORPARAMETERSETTING = 7
        VELOCITYPID = 8
        KP_ID = 1
        KD_ID = 2
        KI_ID = 3
        msg = struct.pack('<BB BH BH BH', VELOCITYPID, channel, KP_ID, kp, KD_ID, ki, KI_ID, ki)
        assert len(msg) == 11, str(msg)
        self.send_msg(MOTORPARAMETERSETTING, msg)
        
    def do_go(self):
        self.logger.debug('Connect!')
        self.set_motor_velctrl_pid(0, 1, 0, 1)

    def do_stop(self):
        self.logger.debug('Stop!')


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s|%(levelname)8s|%(name)0.8s|%(message)s')
    app = QApplication(sys.argv)
    w = ControlPanel()
    w.show()
    app.exec()
