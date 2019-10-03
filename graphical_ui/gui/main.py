import sys
from PyQt5 import QtWidgets, uic, QtCore

from MainWindow import Ui_MainWindow


class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)
        self.setUpWidgets()
        self.connectWidgets()

    def selectService(self, item):
        if   item.data(0,0) == "Complete":
          self.serviceStack.setCurrentIndex(0)
        elif item.data(0,0) == "Send A Signal":
          self.serviceStack.setCurrentIndex(1)
        elif item.data(0,0) == "Set Variable":
          self.serviceStack.setCurrentIndex(2)
          self.variableStack.setCurrentIndex(0)

    
    def refreshClicked(self):
        self.serviceStack.setCurrentIndex(3)
        self.variableStack.setCurrentIndex(2)
        self.treeWidget.clearSelection()
        self.treeWidget.collapseAll()
        self.comboBox.setCurrentIndex(0)

    def varSelection(self, index):
        if index == 0:
          self.variableStack.setCurrentIndex(0)
        elif index == 1:
          self.variableStack.setCurrentIndex(1)
    
    def setUpWidgets(self):
        self.serviceStack.setCurrentIndex(3)
        self.variableStack.setCurrentIndex(2)

    def connectWidgets(self):
        self.treeWidget.itemDoubleClicked.connect(self.selectService)
        self.button_refresh.clicked.connect(self.refreshClicked)
        self.comboBox.currentIndexChanged.connect(self.varSelection)

    def setTableUneditable(self):
        for i in self.tableWidget.rowCount():
          self.item(i,0).setFlags(self.ItemIsEditable)

app = QtWidgets.QApplication(sys.argv)

window = MainWindow()
window.show()
app.exec()
