from PyQt5 import QtCore, QtWidgets, QtSerialPort
import time


class SerialManagerArduino(QtCore.QObject):
    valueChanged = QtCore.pyqtSignal(list)
    valueChanged = QtCore.pyqtSignal(list)
    update = QtCore.pyqtSignal(QtCore.QObject)

    def __init__(self, check_fn, parent=None):
        super().__init__(parent)

        # Change the COM port for each computer this program is run on (since the arduino may have a different port)
        self.serial_port = QtSerialPort.QSerialPort("COM9")
        self.serial_port.setBaudRate(QtSerialPort.QSerialPort.Baud9600)
        self.serial_port.errorOccurred.connect(self.handle_error)
        self.serial_port.readyRead.connect(self.handle_ready_read)
        self.serial_port.open(QtCore.QIODevice.ReadWrite)

        self.check = check_fn   # Check if the if the process has been enabled
        self.current_value = 0  # The current output value of the current

    def handle_ready_read(self):
        while self.serial_port.canReadLine():

            # Read the input from the serial port
            codec = QtCore.QTextCodec.codecForName("UTF-8")
            line = self.serial_port.readLine()
            print(line)

            line = codec.toUnicode(line).strip().strip("\x00")

            line = line.split("\t")

            try:
                # Separate the input into a list of floats
                value = [float(t) for t in line]
            except ValueError as e:
                print("error", e)
            else:
                if self.check():
                    # Emit to get_arduino() and update_current() in main.py
                    self.valueChanged.emit(value)
                    self.update.emit(self)

                    # Write the pid-calculated current to the arduino
                    if self.serial_port.isWritable():
                        self.serial_port.write((str(self.current_value) + '\r\n').encode())

    def handle_error(self, error):
        if error == QtSerialPort.QSerialPort.NoError:
            return
        print(error, self.serial_port.errorString())


# This class is unused
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


# This class is unused
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
