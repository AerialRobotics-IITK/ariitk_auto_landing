#!/usr/bin/env python

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ROS_GUI.ui'
#
# Created by: PyQt4 UI code generator 4.12.1
#
# WARNING! All changes made in this file will be lost!


import roslib
import rospy
from PyQt4 import QtCore, QtGui
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import numpy as np
from nav_msgs.msg import Odometry


# roslib.load_manifest("ros_gui")


try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

husky_odom = Odometry()
msg = Twist()
husky_x = 2
husky_y = 2
STOP = True

def ros_interface():

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if STOP == False :
            cmd_vel_pub.publish(msg)
        
        # rate.sleep()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if STOP == False :
            cmd_vel_pub.publish(msg)
        spinOnce()
        
        rate.sleep()

def forward():
    # msg = Twist()
    msg.linear.x = 1
    msg.angular.z = 0
    cmd_vel_pub.publish(msg)
    print("moving forward")

def right():
    # msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = -1 * 0.4
    cmd_vel_pub.publish(msg)
    print("turned right")
def left():
    # msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 1* 0.4
    cmd_vel_pub.publish(msg)
    print("turned left")
    
def eight():
    None
def circle():
    STOP = False
    # msg = Twist()
    msg.linear.x = 8
    msg.angular.z = 0.2
    # rate = rospy.Rate(10)
    # while STOP == False:
        # cmd_vel_pub.publish(msg)
        # rate.sleep()
    print("HUSKY STARTS MOVING IN CIRCULAR TRAJECTORY. PRESS STOP BUTTON TO STOP IT ..!!")
    ros_interface()
def stop():
    # msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    STOP = True
    # cmd_vel_pub.publish(msg)


def linear():
    # msg = Twist()
    msg.linear.x = 4
    msg.angular.z = 0
    STOP = False
    # rate = rospy.Rate(10)
    # while STOP == False:
        # cmd_vel_pub.publish(msg)
        # rate.sleep()
    print("HUSKY STARTS MOVING IN LINEAR TRAJECTORY. PRESS STOP BUTTON TO STOP IT ..!!")
def keybr():
    None

def husky_clbk(msg):
    index = 0

    while msg.name[index]!= "/":
        index += 1

    husky_odom.pose.pose.position.x = msg.pose[index].position.x
    husky_odom.pose.pose.position.y = msg.pose[index].position.y


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(827, 515)
        self.Forward = QtGui.QPushButton(Form)
        self.Forward.setGeometry(QtCore.QRect(140, 100, 221, 61))
        self.Forward.setObjectName(_fromUtf8("pushButton"))
        self.Forward.setAutoRepeat(False)
        self.Right = QtGui.QPushButton(Form)
        self.Right.setGeometry(QtCore.QRect(320, 200, 151, 51))
        self.Right.setObjectName(_fromUtf8("pushButton_3"))
        self.Right.setAutoRepeat(False)
        self.Left = QtGui.QPushButton(Form)
        self.Left.setGeometry(QtCore.QRect(10, 200, 151, 51))
        self.Left.setObjectName(_fromUtf8("pushButton_4"))
        self.Left.setAutoRepeat(False)
        self.Eight = QtGui.QPushButton(Form)
        self.Eight.setGeometry(QtCore.QRect(530, 130, 181, 51))
        self.Eight.setObjectName(_fromUtf8("pushButton_5"))
        self.Eight.setAutoRepeat(False)
        self.Stop = QtGui.QPushButton(Form)
        self.Stop.setGeometry(QtCore.QRect(140, 290, 221, 61))
        self.Stop.setObjectName(_fromUtf8("pushButton_6"))
        self.Stop.setAutoRepeat(False)
        self.Circle = QtGui.QPushButton(Form)
        self.Circle.setGeometry(QtCore.QRect(530, 190, 181, 51))
        self.Circle.setObjectName(_fromUtf8("pushButton_7"))
        self.Circle.setAutoRepeat(False)
        self.Linear = QtGui.QPushButton(Form)
        self.Linear.setGeometry(QtCore.QRect(530, 250, 181, 51))
        self.Linear.setObjectName(_fromUtf8("pushButton_8"))
        self.Linear.setAutoRepeat(False)
        self.USE_KEYBOARD = QtGui.QPushButton(Form)
        self.USE_KEYBOARD.setGeometry(QtCore.QRect(60, 400, 361, 71))
        self.USE_KEYBOARD.setObjectName(_fromUtf8("pushButton_2"))
        self.USE_KEYBOARD.setAutoRepeat(False)

        self.retranslateUi(Form)
        # QtCore.QObject.connect(self.pushButton,QtCore.SIGNAL(_fromUtf8("clicked()")), self.forward)
        
        self.Forward.clicked.connect(forward)
        self.Right.clicked.connect(right)
        self.Left.clicked.connect(left)
        self.Eight.clicked.connect(eight)
        self.Stop.clicked.connect(stop)
        self.Circle.clicked.connect(circle)
        self.Linear.clicked.connect(linear)
        self.USE_KEYBOARD.clicked.connect(keybr)


        QtCore.QMetaObject.connectSlotsByName(Form)


    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.Forward.setText(_translate("Form", "Forward", None))
        self.Right.setText(_translate("Form", "Right", None))
        self.Left.setText(_translate("Form", "Left", None))
        self.Eight.setText(_translate("Form", "Eight", None))
        self.Stop.setText(_translate("Form", "Stop", None))
        self.Circle.setText(_translate("Form", "Circle", None))
        self.Linear.setText(_translate("Form", "Linear", None))
        self.USE_KEYBOARD.setText(_translate("Form", "USE_KEYBOARD", None))

          

if __name__ == "__main__":
    import sys

    rospy.init_node('ros_gui_node',anonymous=True)
    cmd_vel_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/gazebo/model_states",ModelStates,husky_clbk)
    

    # husky_odom = ModelStates()
    # husky_x = 2
    # husky_y = 2
    

    
    try:
                app
    except:
        #app = QtGui.QApplication(sys.argv)
    
        app = QtGui.QApplication(sys.argv)
        # Form = Widget()

        Form = QtGui.QWidget()
        ui = Ui_Form()
        ui.setupUi(Form)

        Form.setWindowTitle("ROS_GUI_AUTO_LANDING")
        Form.show()

       
        
        sys.exit(app.exec_())

