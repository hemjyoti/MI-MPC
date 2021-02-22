from PyQt5 import QtGui  # (the example applies equally well to PySide2)
import pyqtgraph as pg

import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets
import numpy as np
import sys

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    w = gl.GLViewWidget()

    xx = 0
    yx = 0
    zx = 0

    xy = 1
    yy = 0
    zy = 0

    Xdot = (xx, yx, zx)
    Ydot = (xy, yy, zy)

    pts = np.array([Xdot, Ydot])
    sh1 = gl.GLLinePlotItem(pos=pts, width=1, antialias=False)
    w.addItem(sh1)
    w.show()
    #w.hide()
    app.exec()