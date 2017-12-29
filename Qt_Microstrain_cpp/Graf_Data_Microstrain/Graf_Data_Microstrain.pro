#-------------------------------------------------
#
# Project created by QtCreator 2017-10-24T19:38:09
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = Graf_Data_Microstrain
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    funcionesgraf.cpp \
    qcustomplot.cpp

HEADERS  += mainwindow.h \
    funcionesgraf.h \
    qcustomplot.h

FORMS    += mainwindow.ui
