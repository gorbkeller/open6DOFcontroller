#!/usr/bin/env python3

from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy import linalg
from math import atan2


class Slider(QWidget):
    def __init__(self,label):
        QWidget.__init__(self)
        self.layout = QHBoxLayout()
        self.setLayout(self.layout)
        self.label = QLabel(label)
        self.layout.addWidget(self.label)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setValue(49)
        self.slider.setTracking(True)
        self.layout.addWidget(self.slider)

class PlotDisplay(QWidget):
    x_shift=0.0
    y_shift=0.0
    z_shift=0.0
    tilt=0.0
    roll=0.0
    pan=0.0
    def __init__(self):
        QWidget.__init__(self)

        # a figure instance to plot on
        self.figure = plt.figure() 

   
        # this is the Canvas Widget that 
        # displays the 'figure'it takes the
        # 'figure' instance as a parameter to __init__
        self.canvas = FigureCanvas(self.figure)

        self.drawPlot()
   
        # this is the Navigation widget
        # it takes the Canvas widget and a parent
        self.toolbar = NavigationToolbar(self.canvas, self)
   
        # creating a Vertical Box layout
        layout = QVBoxLayout()
           
        # adding tool bar to the layout
        layout.addWidget(self.toolbar)
           
        # adding canvas to the layout
        layout.addWidget(self.canvas)
           
        # setting layout to the main window
        self.setLayout(layout)

    def compute_inverse(self,xs,ys,zs):
        # calculating the point of intersection based on the normal vecs, i.e. projections on the hall bases
        def calc_int_pt(M): # M should be 3x4 for A,B,C,D
            # get the det for the A,B,C cols
            det = linalg.det(M[:, :3])

            # get x,y,z
            x_mat = np.array([M[:,3],M[:,1],M[:,2]])
            x = linalg.det(x_mat)/det

            y_mat = np.array([M[:,0],M[:,3],M[:,2]])
            y = linalg.det(y_mat)/det

            z_mat = np.array([M[:,0],M[:,1],M[:,3]])
            z = linalg.det(z_mat)/det

            return(np.array([x,y,z]))

        M_x = np.array([[*xs[0],np.dot(xs[0],xs[0])], [*xs[1],np.dot(xs[1],xs[1])], [*xs[2],np.dot(xs[2],xs[2])]])
        M_y = np.array([[*ys[0],np.dot(ys[0],ys[0])], [*ys[1],np.dot(ys[1],ys[1])], [*ys[2],np.dot(ys[2],ys[2])]])
        M_z = np.array([[*zs[0],np.dot(zs[0],zs[0])], [*zs[1],np.dot(zs[1],zs[1])], [*zs[2],np.dot(zs[2],zs[2])]])

        #print(f'Calculated mouse_x: \t{calc_int_pt(M_x)}')
        #print(f'Calculated mouse_y: \t{calc_int_pt(M_y)}')
        #print(f'Calculated mouse_y: \t{calc_int_pt(M_z)}')

        M = np.array([[1,0,0],
                          [0,1,0],
                          [0,0,1]])
        M_adj = np.array([calc_int_pt(M_x),calc_int_pt(M_y),calc_int_pt(M_z)]).T

        x = M[:,0]
        y = M[:,1]
        z = M[:,2]

        n = np.cross((z-y),(x-y))
        print(n)
        n_hat = n/linalg.norm(n)
        d = np.dot(M[:,0],n_hat)

        # get plane equation for the adjusted plane

        x_adj = M_adj[:,0]
        y_adj = M_adj[:,1]
        z_adj = M_adj[:,2]

        n_adj = np.cross((z_adj-y_adj),(x_adj-y_adj))
        n_adj_hat = n_adj/linalg.norm(n_adj)

        # find the point on the plane equidistant from all of the points and make Matrix
        e = (x_adj + y_adj + z_adj)/3.0
        t = -n_adj_hat*d + e
        T = np.outer(t,np.array([1,1,1]))

        R_new = np.matmul((M_adj - T),linalg.inv(M))

        def rad2deg(rad):
            return rad*180/3.1415

        #pitch_new = np.arcsin(-R_new[2,0])
        #print(rad2deg(pitch_new))
        #yaw_new = np.arcsin(R_new[2,1]/np.cos(pitch_new))
        #print(rad2deg(yaw_new))
        #roll_new = np.arccos(R_new[0,0]/np.cos(pitch_new))
        #print(rad2deg(roll_new))
        sy = np.sqrt(R_new[0,0]*R_new[0,0] + R_new[1,0]*R_new[1,0])
        singular = sy <1e-6
        x,y,z = 0.0,0.0,0.0
        if not singular:
            x = np.arctan2(R_new[2,1],R_new[2,2])
            y = np.arctan2(-R_new[2,0],sy)
            z = np.arctan2(R_new[1,0],R_new[0,0])
        else:
            x = np.arctan2(R_new[1,2],R_new[1,1])
            y = np.arctan2(-R_new[2,0],sy)
            z = 0

        print(rad2deg(x),rad2deg(y),rad2deg(z))
        return [0,0,0,0,0,0]

    def drawPlot(self):
        def deg2rad(angle):
            return angle*3.1415/180.0

        def quat(theta,x_hat,y_hat,z_hat):
            q0 = np.cos(theta/2.0)
            q1 = x_hat*np.sin(theta/2.0)
            q2 = y_hat*np.sin(theta/2.0)
            q3 = z_hat*np.sin(theta/2.0)
            return np.array([q0,q1,q2,q3])

        def rot_mat_quat(q0,q1,q2,q3):
            r00 = q0**2 + q1**2 - q2**2 - q3**2
            r01 = 2*q1*q2 - 2*q0*q3
            r02 = 2*q1*q3 + 2*q0*q2
            r10 = 2*q1*q2 + 2*q0*q3
            r11 = q0**2 - q1**2 + q2**2 - q3**2
            r12 = 2*q2*q3 - 2*q0*q1
            r20 = 2*q1*q3 - 2*q0*q2
            r21 = 2*q2*q3 + 2*q0*q1
            r22 = q0**2 - q1**2 - q2**2 + q3**2

            return np.array([[r00,r01,r02],
                             [r10,r11,r12],
                             [r20,r21,r22]])

        def rot_mat_rpy(y,r,p):

            r00 = np.cos(y)*np.cos(p)
            r01 = np.cos(y)*np.sin(p)*np.sin(r) - np.sin(y)*np.cos(r)
            r02 = np.cos(y)*np.sin(p)*np.cos(r) + np.sin(y)*np.sin(r)
            r10 = np.sin(y)*np.cos(p)
            r11 = np.sin(y)*np.sin(p)*np.sin(r) + np.cos(y)*np.cos(r)
            r12 = np.sin(y)*np.sin(p)*np.cos(r) - np.cos(y)*np.sin(r)
            r20 = -np.sin(p)
            r21 = np.cos(p)*np.sin(r)
            r22 = np.cos(p)*np.cos(r)

            return np.array([[r00,r01,r02],
                             [r10,r11,r12],
                             [r20,r21,r22]])

        # the angle between each axis from the centerline of the 3hall_basis
        hall_basis_angle = 30*3.1415/180.0 # rad
        hall_rot_angle = 120*3.1415/180.0 # rad

        # create a 3hall_basis around x axis with quaternions
        rot_quat_x = quat(hall_rot_angle,1.0,0.0,0.0)
        x_1 = np.array([np.cos(hall_basis_angle), 0.0, np.sin(hall_basis_angle)])
        x_2 = np.dot(rot_mat_quat(*rot_quat_x),x_1)
        x_3 = np.dot(rot_mat_quat(*rot_quat_x),x_2)

        # create a 3hall_basis around x axis with quaternions
        rot_quat_y = quat(hall_rot_angle,0.0,1.0,0.0)
        y_1 = np.array([0.0, np.cos(hall_basis_angle), np.sin(hall_basis_angle)])
        y_2 = np.dot(rot_mat_quat(*rot_quat_y),y_1)
        y_3 = np.dot(rot_mat_quat(*rot_quat_y),y_2)

        # create a 3hall_basis around x axis with quaternions
        rot_quat_z = quat(hall_rot_angle,0.0,0.0,1.0)
        z_1 = np.array([0.0, np.sin(hall_basis_angle), np.cos(hall_basis_angle)])
        z_2 = np.dot(rot_mat_quat(*rot_quat_z),z_1)
        z_3 = np.dot(rot_mat_quat(*rot_quat_z),z_2)

        x_hall_basis = np.array([[0, 0, 0, *x_1],
                        [0, 0, 0, *x_2],
                        [0, 0, 0, *x_3]])

        y_hall_basis = np.array([[0, 0, 0, *y_1],
                        [0, 0, 0, *y_2],
                        [0, 0, 0, *y_3]])

        z_hall_basis = np.array([[0, 0, 0, *z_1],
                        [0, 0, 0, *z_2],
                        [0, 0, 0, *z_3]])

        X1, Y1, Z1, U1, V1, W1 = zip(*x_hall_basis)
        X2, Y2, Z2, U2, V2, W2 = zip(*y_hall_basis)
        X3, Y3, Z3, U3, V3, W3 = zip(*z_hall_basis)

        # plot our adjusted mouse
        t = np.array([self.x_shift,self.y_shift,self.z_shift])
        R = rot_mat_rpy(deg2rad(self.pan),deg2rad(self.tilt),deg2rad(self.roll))
        pts = np.array([[1,0,0],[0,1,0],[0,0,1]])
        pts_adj = np.matmul(R,pts.T)+np.array([t,t,t]).T
        x_pt_adj = pts_adj[:,0]
        y_pt_adj = pts_adj[:,1]
        z_pt_adj = pts_adj[:,2]
        mouse_pose = np.array([[*t, *(x_pt_adj-t)],
                        [*t, *(y_pt_adj-t)],
                        [*t, *(z_pt_adj-t)]])
        X4, Y4, Z4, U4, V4, W4 = zip(*mouse_pose)

        self.figure.clear()
        ax1 = self.figure.add_subplot(111, projection='3d')
        ax1.quiver(X1, Y1, Z1, U1, V1, W1, linestyle='dashed', alpha=0.7, color='pink')
        ax1.quiver(X2, Y2, Z2, U2, V2, W2, linestyle='dashed', alpha=0.4, color='olive')
        ax1.quiver(X3, Y3, Z3, U3, V3, W3, linestyle='dashed', alpha=0.4, color='teal')
        ax1.set_xlim([-1, 1])
        ax1.set_ylim([-1, 1])
        ax1.set_zlim([-1, 1])
        ax1.quiver(X4, Y4, Z4, U4, V4, W4, color='purple')

        x_1_proj = np.dot(x_pt_adj,x_1.T)*x_1
        x_2_proj = np.dot(x_pt_adj,x_2.T)*x_2
        x_3_proj = np.dot(x_pt_adj,x_3.T)*x_3

        y_1_proj = np.dot(y_pt_adj,y_1.T)*y_1
        y_2_proj = np.dot(y_pt_adj,y_2.T)*y_2
        y_3_proj = np.dot(y_pt_adj,y_3.T)*y_3

        z_1_proj = np.dot(z_pt_adj,z_1.T)*z_1
        z_2_proj = np.dot(z_pt_adj,z_2.T)*z_2
        z_3_proj = np.dot(z_pt_adj,z_3.T)*z_3

        ax1.scatter3D(*[[x] for x in x_1_proj],color='r',marker='*')
        ax1.scatter3D(*[[x] for x in x_2_proj],color='r',marker='*')
        ax1.scatter3D(*[[x] for x in x_3_proj],color='r',marker='*')

        ax1.scatter3D(*[[x] for x in y_1_proj],color='g',marker='*')
        ax1.scatter3D(*[[x] for x in y_2_proj],color='g',marker='*')
        ax1.scatter3D(*[[x] for x in y_3_proj],color='g',marker='*')

        ax1.scatter3D(*[[x] for x in z_1_proj],color='b',marker='*')
        ax1.scatter3D(*[[x] for x in z_2_proj],color='b',marker='*')
        ax1.scatter3D(*[[x] for x in z_3_proj],color='b',marker='*') 
        self.canvas.draw()

        x,y,z,r,t,p = self.compute_inverse(np.array([x_1_proj, x_2_proj,x_3_proj]),
                np.array([y_1_proj,y_2_proj,y_3_proj]),
                np.array([z_1_proj,z_2_proj,z_3_proj]))

    def updateXShift(self,value):
        self.x_shift = (value-50.0)/60.0
        self.drawPlot()

    def updateYShift(self,value):
        self.y_shift = (value-50.0)/60.0
        self.drawPlot()
        
    def updateZShift(self,value):
        self.z_shift = (value-50.0)/60.0
        self.drawPlot()

    def updatePan(self,value):
        self.pan = (value-50.0)
        self.drawPlot()
        
    def updateTilt(self,value):
        self.tilt = (value-50.0)
        self.drawPlot()

    def updateRoll(self,value):
        self.roll = (value-50.0)
        self.drawPlot()

def test():
    print('test')


class Window(QWidget):
    def __init__(self):

        QWidget.__init__(self)
        self.setWindowTitle("6DOF Mouse Testing")

        layout = QGridLayout() 
        self.setLayout(layout)

        title_label = QLabel("6DOF Mouse Testing")
        layout.addWidget(title_label,0,0)

        vlayout = QVBoxLayout()
        slider_widgets = QWidget()
        slider_widgets.setLayout(vlayout)

        self.x_shift_slider = Slider('X Shift')
        vlayout.addWidget(self.x_shift_slider)

        self.y_shift_slider = Slider('Y Shift')
        vlayout.addWidget(self.y_shift_slider)

        self.z_shift_slider = Slider('Z Shift')
        vlayout.addWidget(self.z_shift_slider)

        self.roll_slider = Slider('Roll')
        vlayout.addWidget(self.roll_slider)

        self.tilt_slider = Slider('Tilt')
        vlayout.addWidget(self.tilt_slider)

        self.pan_slider = Slider('Pan')
        vlayout.addWidget(self.pan_slider)

        self.reset_button = QPushButton('Reset')
        vlayout.addWidget(self.reset_button)

        layout.addWidget(slider_widgets,1,0,5,6)

        plotter = PlotDisplay()
        layout.addWidget(plotter,0,6,6,6)
        self.x_shift_slider.slider.valueChanged.connect(plotter.updateXShift)
        self.y_shift_slider.slider.valueChanged.connect(plotter.updateYShift)
        self.z_shift_slider.slider.valueChanged.connect(plotter.updateZShift)
        self.roll_slider.slider.valueChanged.connect(plotter.updateRoll)
        self.tilt_slider.slider.valueChanged.connect(plotter.updateTilt)
        self.pan_slider.slider.valueChanged.connect(plotter.updatePan)
        self.reset_button.clicked.connect(self.resetSliders)

    def resetSliders(self):
        self.x_shift_slider.slider.setValue(49)
        self.y_shift_slider.slider.setValue(49)
        self.z_shift_slider.slider.setValue(49)
        self.roll_slider.slider.setValue(49)
        self.tilt_slider.slider.setValue(49)
        self.pan_slider.slider.setValue(49)


app = QApplication(sys.argv)

screen = Window()
screen.show()

sys.exit(app.exec_())
