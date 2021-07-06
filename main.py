import math
import os
import sys
import time
import shutil
import qdarkstyle
from qtGUI import Ui_MainWindow
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from SerialManager import SerialManagerArduino, SerialManagerCombine
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QPushButton
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT as NavigationToolbar
import datetime
from scipy import interpolate
import numpy as np
# from PID import PID
from simple_pid import PID


class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100, y_scale="linear"):
        with plt.style.context("dark_background"):
            fig = Figure(figsize=(width, height), dpi=dpi)
            self.axes = fig.add_subplot(111, yscale=y_scale)
        super(MplCanvas, self).__init__(fig)


def k_type_fit():
    volt = []
    with open("k-type-table.txt", "r") as f:
        lines = f.readlines()
    for l in lines:
        l = l.strip().split('\t')
        volt += [(float(v)/1000) for v in l[1:-1:1]]
    temp = np.arange(volt.__len__())
    f = interpolate.interp1d(volt, temp, bounds_error=False, fill_value="extrapolate")
    return f


class BakingLogGui(Ui_MainWindow):

    def __init__(self, mainwindow):
        # Initialize the qt designer gui
        self.mainwindow = mainwindow
        self.setupUi(mainwindow)

        # Parameters
        self.channelNum = 6  # Maximum available channels in this app
        self.maxVoltage = 3.3  # (V) arduino due analog reading maximum
        self.bitdepth = 12  # arduino due analog read bit depth
        self.t0 = 0  # To store the start time
        self.isLogging = False
        self.dataFileName = "data.txt"
        self.dac = 0

        # Initialize data array
        self.channelData = [[[], []] for _ in range(self.channelNum)]
        self.pressureData = [[], []]

        # Read channel switches from file
        self.channelSwitches = self.read_channel_switches()

        # Initialize states of the checkboxes
        self.channelCheckboxes = (self.actionChannel_1, self.actionChannel_2, self.actionChannel_3,
                                  self.actionChannel_4, self.actionChannel_5, self.actionChannel_6)
        for i in range(self.channelNum):
            if self.channelSwitches[i] > 0:
                self.channelCheckboxes[i].setChecked(True)
            else:
                self.channelCheckboxes[i].setChecked(False)

        # Connect toggled signals to checkboxes
        self.actionChannel_1.toggled.connect(self.toggled_signal_generator(0))
        self.actionChannel_2.toggled.connect(self.toggled_signal_generator(1))
        self.actionChannel_3.toggled.connect(self.toggled_signal_generator(2))
        self.actionChannel_4.toggled.connect(self.toggled_signal_generator(3))
        self.actionChannel_5.toggled.connect(self.toggled_signal_generator(4))
        self.actionChannel_6.toggled.connect(self.toggled_signal_generator(5))

        # Track whether this is the first/second update (for boundary conditions in update_plot)
        self.first_update = True
        self.second_update = False

        # Add the temperature canvas and related widgets to the gui
        self.canvasT = MplCanvas(self, width=5, height=4, dpi=100, y_scale="linear")
        self.canvasT.axes.set_xlabel("time (s)")
        self.canvasT.axes.set_title("Temperature ($^\circ$C)")
        toolbarT = NavigationToolbar(self.canvasT, mainwindow)
        layoutT = QtWidgets.QVBoxLayout()

        self.port_button = QPushButton('Switch DAC')
        self.port_button.setCheckable(True)
        self.port_button.toggle()
        layoutT.addWidget(self.port_button)
        self.prev_port = 0

        self.button1 = QPushButton('Toggle Autoscaling')
        self.button1.setCheckable(True)
        self.button1.toggle()
        layoutT.addWidget(self.button1)
        # self.autoscale_T = True

        layoutT.addWidget(toolbarT)
        layoutT.addWidget(self.canvasT)


        # Add the pressure canvas and related widgets to the gui
        self.canvasP = MplCanvas(self, width=5, height=4, dpi=100, y_scale="linear")
        self.canvasP.axes.set_xlabel("time (s)")
        self.canvasP.axes.set_title("Current (A)")
        toolbarP = NavigationToolbar(self.canvasP, mainwindow)
        layoutP = QtWidgets.QVBoxLayout()

        self.current_button = QPushButton('Toggle Current')
        self.current_button.setCheckable(True)
        # self.current_button.toggle()
        layoutP.addWidget(self.current_button)

        self.button2 = QPushButton('Toggle Autoscaling')
        self.button2.setCheckable(True)
        self.button2.toggle()
        layoutP.addWidget(self.button2)
        # self.autoscale_P = True

        layoutP.addWidget(toolbarP)
        layoutP.addWidget(self.canvasP)


        # Add the temperature and pressure layouts together to finish building the gui
        self.horizontalLayoutPlot.addLayout(layoutT)
        self.horizontalLayoutPlot.addLayout(layoutP)

        self.actionStart.triggered.connect(self.start_logging)
        self.actionRefresh.triggered.connect(self.refresh)


        # Create an empty file
        with open(self.dataFileName, "w") as f:
            f.write('time\tT1\tT2\tT3\tT4\tT5\tT6\tP\n')

        self.actionSave_data.triggered.connect(self.save_file)

        # Thermocouple
        self.f = k_type_fit()
        self.gain = [6.46159613e+01, 64.85970839, 64.33, 64.33, 64.33, 64.33]
        self.offset = [-6.32907620e-02, -0.0699907, -0.076, -0.076, -0.076, -0.076]

        # Feedback loop parameters
        self.setPointInput.setText('70')
        self.target_temp = 70
        self.step_size = [0.1, 0.3]

        self.pid = PID(100.0, 0.6, 5.0, setpoint=self.target_temp)
        self.pid.output_limits = (0, 4095)
        # self.pid.setPoint(70.0)
        # self.pid.setPoint(50.0)

    # Read channel switches from file
    def read_channel_switches(self):
        with open("channelSwitches.txt", "rb") as f:
            return [int(f.read(1)) for _ in range(self.channelNum)]

    # Write channel switches to file
    def write_channel_switches(self, channel_switches):
        with open("channelSwitches", "wb") as f:
            for i in range(self.channelNum):
                f.write(str(channel_switches[i]).encode())

    def toggled_signal_generator(self, i):
        def toggled_signal(ischeck):
            if ischeck:
                self.channelSwitches[i] = 1
            else:
                self.channelSwitches[i] = 0
            self.write_channel_switches(self.channelSwitches)

        return toggled_signal

    def start_logging(self):
        self.first_update = True

        self.isLogging = True
        if self.t0 == 0:
            self.t0 = time.time()
        self.actionStart.setText("Stop")
        self.actionStart.triggered.connect(self.stop_logging)

    def check_logging_status(self):
        return self.isLogging

    def stop_logging(self):
        self.isLogging = False
        self.actionStart.setText("Start")
        self.actionStart.triggered.connect(self.start_logging)

    def refresh(self):
        self.channelData = [[[], []] for _ in range(self.channelNum)]
        self.pressureData = [[], []]
        if self.check_logging_status():
            self.t0 = time.time()
        else:
            self.t0 = 0

        # First/second iteration boundary conditions
        self.first_update = True
        self.second_update = True

        self.update_plot()
        with open(self.dataFileName, "w") as f:
            f.write('time\tT1\tT2\tT3\tT4\tT5\tT6\tP\n')

    def get_arduino(self, v):
        # v = [vv / 2 ** self.bitdepth * self.maxVoltage for vv in v[:-1]] + [
        #     v[-1]]  # convert from arduino result to real voltage (V)
        v = [vv / 2 ** self.bitdepth * self.maxVoltage for vv in v]  # convert from arduino result to real voltage (V)
        # print((v[1] - self.offset[1]) / self.gain[1])
        # T = self.f(v)
        t = time.time() - self.t0
        print('Time: ' + str(t))
        s = str(t) + '\t'
        for i in range(self.channelNum):
            if self.channelSwitches[i] > 0:
                self.channelData[i][0] += [t]
                T = self.f((v[i] - self.offset[i]) / self.gain[i])  # convert to the voltage before the amplifier
                self.channelData[i][1] += [T]
                s += str(T)
            else:
                s += "-1"
            s += '\t'
        # if v[-1]:
        #     self.pressureData[0] += [t]
        #     self.pressureData[1] += [v[-1]]
        #     s += str(v[-1])
        # else:
        #     s += "-1"
        s += '\n'
        with open(self.dataFileName, "a") as f:
            f.write(s)

    def update_current(self, m):
        if len(self.setPointInput.text()) > 1:
            self.target_temp = int(self.setPointInput.text())
            self.pid.setpoint = self.target_temp

        if self.port_button.isChecked():
            self.port_button.setText('DAC0 Active')
            self.dac = 0
        else:
            self.port_button.setText('DAC1 Active')
            self.dac = 1
        if self.prev_port != self.dac:
            self.pressureData = [[], []]
            self.prev_port = self.dac
        print('Temperature: ' + str(self.channelData[self.dac][1][-1]))
        print('Target Temp: ' + str(self.target_temp))
        self.pressureData[0].append(self.channelData[self.dac][0][-1])
        if len(self.pressureData[1]) > 0:
            # direction = (self.target_temp - self.channelData[self.dac][1][-1])
            # direction = direction / abs(direction)
            # change = self.step_size[1] * direction * (math.e ** (self.step_size[0] * abs(self.target_temp - self.channelData[self.dac][1][-1])))
            if self.current_button.isChecked():
                self.current_button.setText('Current Enabled')

                past_time = -1 * min(len(self.channelData[self.dac][0]), 30)
                coeffs = np.polyfit(self.channelData[self.dac][0][past_time:], self.channelData[self.dac][1][past_time:], 1)

                future_t = self.channelData[self.dac][0][-1] + 15
                pval = coeffs[0] * future_t + coeffs[1]

                change = self.pid(pval)
            else:
                self.current_button.setText('Current Disabled')
                change = 0
            # if abs(change) < 1:
            #     change = change / abs(change)

            # self.pressureData[1].append((change + self.pressureData[1][-1]))

            self.pressureData[1].append(change)
            # self.pressureData[1].append(int(math.sin(time.time() / 5) * 1000) + 2000)
        else:
            self.pressureData[1].append(0)
        if self.pressureData[1][-1] > 4095 or self.pressureData[1][-1] < 0:
            print('current bounds exceeded')
        # print('Current: ' + str(self.pressureData[1][-1]))
        self.pressureData[1][-1] = max(min((self.pressureData[1][-1]), 4095), 0)
        print('Current: ' + str(self.pressureData[1][-1]))

        m.current_value = int(self.pressureData[1][-1] + self.dac * 4096)
        # print()
        self.update_plot()

    # Todo: radio buttons and spin boxes
    def update_plot(self):
        with plt.style.context("dark_background"):

            if self.first_update or self.second_update:
                if not self.first_update and self.second_update:
                    self.second_update = False
                self.first_update = False

                # Clear the canvas.
                self.canvasT.axes.cla()

                # Plot the current data
                for i in range(self.channelNum):
                    if self.channelSwitches[i] > 0 and len(self.channelData[i][1]) > 0:
                        self.canvasT.axes.plot(self.channelData[i][0], self.channelData[i][1],
                                               label="ch" + str(i + 1) + ": " + "%.1f" % (self.channelData[i][1][-1]) + "C")
                print(self.channelData)

                # set the lower x bound and finish building the graph
                self.canvasT.axes.set_xbound(lower=0)
                self.canvasT.axes.legend()
                self.canvasT.axes.set_xlabel("time (s)")
                self.canvasT.axes.set_title("Temperature ($^\circ$C)")
                self.canvasT.draw()

                # Clear the canvas
                self.canvasP.axes.cla()

                # Plot the current data
                # if len(self.pressureData[1]) > 0:
                # self.canvasP.axes.plot(self.pressureData[0], self.pressureData[1])
                self.canvasP.axes.plot(self.pressureData[0], [x / 409.5 for x in self.pressureData[1]])

                # set the lower x bound and finish building the graph
                self.canvasP.axes.set_xbound(lower=0)
                # self.canvasP.axes.legend()
                self.canvasP.axes.set_xlabel("time (s)")
                self.canvasP.axes.set_title("Current (A)")
                self.canvasP.draw()

                return

            labels = []
            lines = self.canvasT.axes.get_lines()
            active_channels = []

            # Reset the data and update the labels
            for i in range(self.channelNum):
                if self.channelSwitches[i] > 0 and len(self.channelData[i][1]) > 0:
                    active_channels.append(i)
                    line = lines.pop(0)
                    line.set_data(self.channelData[i][0], self.channelData[i][1])
                    labels.append("ch" + str(i + 1) + ": " + "%.1f" % (self.channelData[i][1][-1]) + "C")

            # Update scaling if the temperature autoscale is toggled
            if self.button1.isChecked():
                self.button1.setText('Autoscaling Enabled')
                self.canvasT.axes.set_xbound(lower=0, upper=max(self.channelData[0][0]) * 1.05)
                y_max = max(max(y) for y in (self.channelData[i][1] for i in active_channels))
                y_min = min(min(y) for y in (self.channelData[i][1] for i in active_channels))
                if y_max != y_min:
                    self.canvasT.axes.set_ybound(lower=y_min - (y_max - y_min) * 0.05, upper=y_max + (y_max - y_min) * 0.05)
            else:
                self.button1.setText('Autoscaling Disabled')

            # Trigger the canvas to update and redraw.
            self.canvasT.axes.legend(labels)
            self.canvasT.draw()
            self.canvasT.flush_events()


            # Reset the data and update the labels
            if len(self.pressureData[1]) > 0:
                line = self.canvasP.axes.get_lines()[0]
                # line.set_data(self.pressureData[0], self.pressureData[1])
                line.set_data(self.pressureData[0], [x / 409.5 for x in self.pressureData[1]])
                # self.canvasP.axes.legend(["I:" + "%.0f" % self.pressureData[1][-1]])
                self.canvasP.axes.legend(["I: " + "%.3f" % (self.pressureData[1][-1] / 409.5) + "A"])

                # Update scaling if the pressure autoscale is toggled
                if self.button2.isChecked():
                    self.button2.setText('Autoscaling Enabled')
                    self.canvasP.axes.set_xbound(lower=0, upper=max(self.pressureData[0]) * 1.05)
                    # y_max = max(self.pressureData[1])
                    # y_min = min(self.pressureData[1])
                    y_max = max([x / 409.5 for x in self.pressureData[1]])
                    y_min = min([x / 409.5 for x in self.pressureData[1]])
                    if y_max != y_min:
                        self.canvasP.axes.set_ybound(lower=y_min - (y_max - y_min) * 0.05, upper=y_max + (y_max - y_min) * 0.05)
                else:
                    self.button2.setText('Autoscaling Disabled')

            # Trigger the canvas to update and redraw.
            self.canvasP.axes.tick_params(axis='y', which='both', labelsize='small', direction='in', pad=0)
            self.canvasP.draw()
            self.canvasP.flush_events()

    def save_file(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        name, _ = QtWidgets.QFileDialog.getSaveFileName(self.mainwindow, "Save data", "", "Text Files (*.txt)",
                                                        options=options)
        if name[-4:].lower() != '.txt':
            name += ".txt"
        shutil.copyfile(os.path.join(os.getcwd(), self.dataFileName), name)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    MainWindow = QtWidgets.QMainWindow()
    ui = BakingLogGui(MainWindow)
    MainWindow.show()
    manager = SerialManagerArduino(ui.check_logging_status)
    manager.valueChanged.connect(ui.get_arduino)
    manager.update.connect(ui.update_current)
    app.exec_()
    sys.exit()
