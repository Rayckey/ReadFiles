#-------------------------------------------------
#
# Project created by QtCreator 2018-08-30T10:57:43
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

TARGET = ReadFiles
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui



INCLUDEPATH += $$PWD/../Eigen3
DEPENDPATH += $$PWD/../Eigen3





win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../XR1ControllerPM-build/release/ -lXR1ControllerPM
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../XR1ControllerPM-build/debug/ -lXR1ControllerPM
else:unix: LIBS += -L$$PWD/../XR1ControllerPM-build/ -lXR1ControllerPM

INCLUDEPATH += $$PWD/../XR1ControllerPM
DEPENDPATH += $$PWD/../XR1ControllerPM


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../XR1ControllerALP-build/release/ -lXR1ControllerALP
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../XR1ControllerALP-build/debug/ -lXR1ControllerALP
else:unix: LIBS += -L$$PWD/../XR1ControllerALP-build/ -lXR1ControllerALP

INCLUDEPATH += $$PWD/../XR1ControllerALP
DEPENDPATH += $$PWD/../XR1ControllerALP

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../XR1Controller-build/release/ -lXR1Controller
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../XR1Controller-build/debug/ -lXR1Controller
else:unix: LIBS += -L$$PWD/../XR1Controller-build/ -lXR1Controller

INCLUDEPATH += $$PWD/../XR1Controller
DEPENDPATH += $$PWD/../XR1Controller
