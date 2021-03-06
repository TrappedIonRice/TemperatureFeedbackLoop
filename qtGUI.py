# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'BakingLog.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 700)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.horizontalLayoutPlot = QtWidgets.QHBoxLayout()
        self.horizontalLayoutPlot.setObjectName("horizontalLayoutPlot")
        self.gridLayout.addLayout(self.horizontalLayoutPlot, 0, 0, 1, 1)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.groupBoxTemp = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBoxTemp.setObjectName("groupBoxTemp")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.groupBoxTemp)
        self.gridLayout_4.setContentsMargins(2, -1, 2, -1)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.horizontalLayout_4.setStretch(1, 1)
        self.horizontalLayout_4.setStretch(3, 1)
        self.gridLayout_4.addLayout(self.horizontalLayout_4, 0, 0, 1, 1)
        self.horizontalLayout_2.addWidget(self.groupBoxTemp)
        self.horizontalLayout_2.setStretch(0, 2)
        self.horizontalLayout_2.setStretch(1, 3)
        self.horizontalLayout_2.setStretch(2, 3)
        self.gridLayout.addLayout(self.horizontalLayout_2, 1, 0, 1, 1)
        self.gridLayout.setRowStretch(0, 1)
        self.gridLayout_2.addLayout(self.gridLayout, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1045, 26))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuChannels = QtWidgets.QMenu(self.menubar)
        self.menuChannels.setObjectName("menuChannels")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionSave_data = QtWidgets.QAction(MainWindow)
        self.actionSave_data.setShortcutContext(QtCore.Qt.WindowShortcut)
        self.actionSave_data.setShortcutVisibleInContextMenu(True)
        self.actionSave_data.setObjectName("actionSave_data")
        self.actionChannel_1 = QtWidgets.QAction(MainWindow)
        self.actionChannel_1.setCheckable(True)
        self.actionChannel_1.setChecked(True)
        self.actionChannel_1.setObjectName("actionChannel_1")
        self.actionChannel_2 = QtWidgets.QAction(MainWindow)
        self.actionChannel_2.setCheckable(True)
        self.actionChannel_2.setChecked(True)
        self.actionChannel_2.setObjectName("actionChannel_2")
        self.actionChannel_3 = QtWidgets.QAction(MainWindow)
        self.actionChannel_3.setCheckable(True)
        self.actionChannel_3.setChecked(True)
        self.actionChannel_3.setObjectName("actionChannel_3")
        self.actionChannel_4 = QtWidgets.QAction(MainWindow)
        self.actionChannel_4.setCheckable(True)
        self.actionChannel_4.setChecked(True)
        self.actionChannel_4.setObjectName("actionChannel_4")
        self.actionChannel_5 = QtWidgets.QAction(MainWindow)
        self.actionChannel_5.setCheckable(True)
        self.actionChannel_5.setChecked(True)
        self.actionChannel_5.setObjectName("actionChannel_5")
        self.actionChannel_6 = QtWidgets.QAction(MainWindow)
        self.actionChannel_6.setCheckable(True)
        self.actionChannel_6.setChecked(True)
        self.actionChannel_6.setObjectName("actionChannel_6")
        self.actionStart = QtWidgets.QAction(MainWindow)
        self.actionStart.setShortcutVisibleInContextMenu(True)
        self.actionStart.setObjectName("actionStart")
        self.actionRefresh = QtWidgets.QAction(MainWindow)
        self.actionRefresh.setShortcutVisibleInContextMenu(True)
        self.actionRefresh.setObjectName("actionRefresh")
        self.menuFile.addAction(self.actionStart)
        self.menuFile.addAction(self.actionRefresh)
        self.menuFile.addAction(self.actionSave_data)
        self.menuChannels.addAction(self.actionChannel_1)
        self.menuChannels.addAction(self.actionChannel_2)
        self.menuChannels.addAction(self.actionChannel_3)
        self.menuChannels.addAction(self.actionChannel_4)
        self.menuChannels.addAction(self.actionChannel_5)
        self.menuChannels.addAction(self.actionChannel_6)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuChannels.menuAction())

        self.setPointInput = QtWidgets.QLineEdit()
        self.setPointInput.setValidator(QtGui.QIntValidator())
        self.setPointInput.setMaxLength(3)
        self.setPointInput.setAlignment(QtCore.Qt.AlignLeft)

        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.addRow('Target Temperature', self.setPointInput)

        self.horizontalLayout_4.addWidget(self.setPointInput)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBoxTemp.setTitle(_translate("MainWindow", "Target Temperature"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.menuChannels.setTitle(_translate("MainWindow", "Channels"))
        self.actionSave_data.setText(_translate("MainWindow", "Save data"))
        self.actionSave_data.setShortcut(_translate("MainWindow", "Ctrl+S"))
        self.actionChannel_1.setText(_translate("MainWindow", "Channel 1"))
        self.actionChannel_1.setShortcut(_translate("MainWindow", "Ctrl+F1"))
        self.actionChannel_2.setText(_translate("MainWindow", "Channel 2"))
        self.actionChannel_2.setShortcut(_translate("MainWindow", "Ctrl+F2"))
        self.actionChannel_3.setText(_translate("MainWindow", "Channel 3"))
        self.actionChannel_3.setShortcut(_translate("MainWindow", "Ctrl+F3"))
        self.actionChannel_4.setText(_translate("MainWindow", "Channel 4"))
        self.actionChannel_4.setShortcut(_translate("MainWindow", "Ctrl+F4"))
        self.actionChannel_5.setText(_translate("MainWindow", "Channel 5"))
        self.actionChannel_5.setShortcut(_translate("MainWindow", "Ctrl+F5"))
        self.actionChannel_6.setText(_translate("MainWindow", "Channel 6"))
        self.actionChannel_6.setShortcut(_translate("MainWindow", "Ctrl+F6"))
        self.actionStart.setText(_translate("MainWindow", "Start"))
        self.actionStart.setShortcut(_translate("MainWindow", "Ctrl+Space"))
        self.actionRefresh.setText(_translate("MainWindow", "Refresh"))
        self.actionRefresh.setShortcut(_translate("MainWindow", "Ctrl+R"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
