from PyQt5 import QtCore, QtWidgets, QtSerialPort
import time
import pyfirmata


class SerialManagerArduino(QtCore.QObject):
    valueChanged = QtCore.pyqtSignal(list)
    valueChanged = QtCore.pyqtSignal(list)
    update = QtCore.pyqtSignal(QtCore.QObject)

    def __init__(self, check_fn, parent=None):
        super().__init__(parent)
        # due = {
        #         'digital': tuple(x for x in range(22, 54)),
        #         'analog': tuple(x for x in range(12)),
        #         'pwm': tuple(x for x in range(2, 14)),
        #         'use_ports': True,
        #         'disabled': ()
        #     }
        # board = pyfirmata.Board('COM11', layout=due)
        # while True:
        #     print('looping')
        #     print(len(board.digital))
        #     board.digital[0].write(1)
        #     time.sleep(0)
        #     board.digital[0].write(0)
        #     time.sleep(0)

        self.serial_port = QtSerialPort.QSerialPort("COM11")
        self.serial_port.setBaudRate(QtSerialPort.QSerialPort.Baud9600)
        self.serial_port.errorOccurred.connect(self.handle_error)
        self.serial_port.readyRead.connect(self.handle_ready_read)
        self.serial_port.open(QtCore.QIODevice.ReadWrite)

        # self.serial_port_agilent = QtSerialPort.QSerialPort("COM8")
        # self.serial_port_agilent.setBaudRate(QtSerialPort.QSerialPort.Baud9600)
        # self.serial_port_agilent.errorOccurred.connect(self.handle_error)
        # self.serial_port_agilent.readyRead.connect(self.handle_ready_read)
        # self.serial_port_agilent.open(QtCore.QIODevice.ReadWrite)

        self.check = check_fn
        self.current_value = 0

    def handle_ready_read(self):
        while self.serial_port.canReadLine():
            # self.serial_port.write(b'1')
            codec = QtCore.QTextCodec.codecForName("UTF-8")
            line = self.serial_port.readLine()
            print(line)

            line = codec.toUnicode(line).strip().strip("\x00")

            line = line.split("\t")

            # if line[0][0] == '^':
            #     return

            try:
                value = [float(t) for t in line]
                # self.serial_port_agilent.write('#0002UHFIG1\r'.encode())
                # print('writes to agilent')
                # try:
                #     p = [float(codec.toUnicode(self.serial_port_agilent.readAll())[1:])]
                # except ValueError as e:
                #     p = [False]
                # value += p
            except ValueError as e:
                print("error", e)
            else:
                if self.check():
                    self.valueChanged.emit(value)
                    self.update.emit(self)

                    if self.serial_port.isWritable():
                        # self.serial_port.write(('DAC0, ' + str(self.current_value) + '\r').encode())
                        self.serial_port.write((str(self.current_value) + '\r\n').encode())
                        # print('happy :)')
                    # else:
                        # print('sad :(')
                    # print()

    def handle_error(self, error):
        if error == QtSerialPort.QSerialPort.NoError:
            return
        print(error, self.serial_port.errorString())


class SerialManagerCombine(QtCore.QObject):
    valueChanged = QtCore.pyqtSignal(list)
    update = QtCore.pyqtSignal()

    def __init__(self, check_fn, parent=None):
        super().__init__(parent)
        self.serial_port_arduino = QtSerialPort.QSerialPort("COM6")
        self.serial_port_arduino.setBaudRate(QtSerialPort.QSerialPort.Baud9600)
        self.serial_port_arduino.errorOccurred.connect(self.handle_error_arduino)
        self.serial_port_arduino.readyRead.connect(self.handle_ready_read)
        self.serial_port_arduino.open(QtCore.QIODevice.ReadWrite)

        self.serial_port_agilent = QtSerialPort.QSerialPort("COM4")
        self.serial_port_agilent.setBaudRate(QtSerialPort.QSerialPort.Baud9600)
        self.serial_port_agilent.errorOccurred.connect(self.handle_error_agilent)
        self.serial_port_agilent.open(QtCore.QIODevice.ReadWrite)
        self.serial_port_agilent.write('#0002UHFIG1\r'.encode())
        self.serial_port_agilent.readLine()

        self.check = check_fn
        self.pressure = 0

    def handle_ready_read(self):
        while self.serial_port_arduino.canReadLine():
            # self.serial_port.write(b'1')
            codec = QtCore.QTextCodec.codecForName("UTF-8")
            line = codec.toUnicode(self.serial_port_arduino.readLine()).strip().strip("\x00")
            line = line.split("\t")
            self.serial_port_agilent.write('#0002UHFIG1\r'.encode())
            time.sleep(0.02)
            line2 = codec.toUnicode(self.serial_port_agilent.readLine())
            print(line2)
            try:
                value = [float(t) for t in line]
                value += [0]#[float(line2[1:])]
            except ValueError as e:
                print("error", e)
            else:
                if self.check():
                    self.valueChanged_arduino.emit(value)
                    self.update.emit()

    def handle_error_arduino(self, error):
        if error == QtSerialPort.QSerialPort.NoError:
            return
        print(error, self.serial_port_arduino.errorString())

    def handle_error_agilent(self, error):
        if error == QtSerialPort.QSerialPort.NoError:
            return
        print(error, self.serial_port_agilent.errorString())


class SerialManagerAgilent(QtCore.QObject):
    valueChanged = QtCore.pyqtSignal(float)
    update = QtCore.pyqtSignal()

    def __init__(self, check_fn, parent=None):
        super().__init__(parent)
        self.serial_port_agilent = QtSerialPort.QSerialPort("COM8")
        self.serial_port_agilent.setBaudRate(QtSerialPort.QSerialPort.Baud9600)
        self.serial_port_agilent.errorOccurred.connect(self.handle_error)
        self.serial_port_agilent.readyRead.connect(self.handle_ready_read)
        self.serial_port_agilent.open(QtCore.QIODevice.ReadWrite)

        self.serial_port_agilent.write('#0002UHFIG1\r'.encode())
        self.check = check_fn

    def handle_ready_read(self):
        while self.serial_port_agilent.canReadLine():
            codec = QtCore.QTextCodec.codecForName("UTF-8")
            line = codec.toUnicode(self.serial_port_agilent.readLine())
            try:
                value = float(line[1:])
            except ValueError as e:
                print("error", e)
            else:
                if self.check():
                    self.valueChanged.emit(value)
                    self.update.emit()
            time.sleep(1)
            self.serial_port_agilent.write('#0002UHFIG1\r'.encode())

    def handle_error(self, error):
        if error == QtSerialPort.QSerialPort.NoError:
            return
        print(error, self.serial_port_agilent.errorString())
