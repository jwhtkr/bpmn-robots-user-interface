# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/ryanjacobson/src/camunda_ws/src/user_interface/graphical_ui/src/main_window.ui'
#
# Created by: PyQt5 UI code generator 5.13.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_GUI(object):
    def setupUi(self, GUI):
        GUI.setObjectName("GUI")
        GUI.resize(1002, 467)
        self.centralwidget = QtWidgets.QWidget(GUI)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.mainStackedWidget = QtWidgets.QStackedWidget(self.centralwidget)
        self.mainStackedWidget.setObjectName("mainStackedWidget")
        self.fileSelectPage = QtWidgets.QWidget()
        self.fileSelectPage.setObjectName("fileSelectPage")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.fileSelectPage)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        spacerItem = QtWidgets.QSpacerItem(963, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_2.addItem(spacerItem)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem1 = QtWidgets.QSpacerItem(100, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.horizontalLayout.addItem(spacerItem1)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.filePathLineEdit = QtWidgets.QLineEdit(self.fileSelectPage)
        self.filePathLineEdit.setObjectName("filePathLineEdit")
        self.gridLayout.addWidget(self.filePathLineEdit, 2, 1, 1, 1)
        self.fileSelectPrompt = QtWidgets.QLabel(self.fileSelectPage)
        self.fileSelectPrompt.setObjectName("fileSelectPrompt")
        self.gridLayout.addWidget(self.fileSelectPrompt, 1, 1, 1, 1)
        self.fileSelectButton = QtWidgets.QPushButton(self.fileSelectPage)
        self.fileSelectButton.setObjectName("fileSelectButton")
        self.gridLayout.addWidget(self.fileSelectButton, 2, 0, 1, 1)
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout.addItem(spacerItem2, 8, 1, 1, 1)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem3)
        self.submitButton = QtWidgets.QPushButton(self.fileSelectPage)
        self.submitButton.setObjectName("submitButton")
        self.horizontalLayout_2.addWidget(self.submitButton)
        self.gridLayout.addLayout(self.horizontalLayout_2, 4, 1, 1, 1)
        spacerItem4 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout.addItem(spacerItem4, 0, 1, 1, 1)
        self.horizontalLayout.addLayout(self.gridLayout)
        spacerItem5 = QtWidgets.QSpacerItem(100, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.horizontalLayout.addItem(spacerItem5)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        spacerItem6 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_2.addItem(spacerItem6)
        self.mainStackedWidget.addWidget(self.fileSelectPage)
        self.userInputPage = QtWidgets.QWidget()
        self.userInputPage.setObjectName("userInputPage")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.userInputPage)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label = QtWidgets.QLabel(self.userInputPage)
        self.label.setObjectName("label")
        self.horizontalLayout_3.addWidget(self.label)
        self.missionName = QtWidgets.QLabel(self.userInputPage)
        self.missionName.setText("")
        self.missionName.setObjectName("missionName")
        self.horizontalLayout_3.addWidget(self.missionName)
        spacerItem7 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem7)
        self.verticalLayout_8.addLayout(self.horizontalLayout_3)
        spacerItem8 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_8.addItem(spacerItem8)
        self.gridLayout_2.addLayout(self.verticalLayout_8, 0, 0, 1, 1)
        self.verticalLayout_4.addLayout(self.gridLayout_2)
        self.mainStackedWidget.addWidget(self.userInputPage)
        self.verticalLayout.addWidget(self.mainStackedWidget)
        GUI.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(GUI)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1002, 22))
        self.menubar.setObjectName("menubar")
        GUI.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(GUI)
        self.statusbar.setObjectName("statusbar")
        GUI.setStatusBar(self.statusbar)

        self.retranslateUi(GUI)
        self.mainStackedWidget.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(GUI)

    def retranslateUi(self, GUI):
        _translate = QtCore.QCoreApplication.translate
        GUI.setWindowTitle(_translate("GUI", "MainWindow"))
        self.fileSelectPrompt.setText(_translate("GUI", "Please select a mission file to run. (.bpmn)"))
        self.fileSelectButton.setText(_translate("GUI", "Choose File"))
        self.submitButton.setText(_translate("GUI", "Submit"))
        self.label.setText(_translate("GUI", "The current Mission is :"))