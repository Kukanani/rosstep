#-------------------------------------------------
#
# Project created by QtCreator 2015-07-23T15:59:39
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = rosstep-creator
TEMPLATE = app
DESTDIR = ../tool/rosstep-creator

OBJECTS_DIR = ../build/rosstep-creator
UI_DIR = ../build/rosstep-creator
MOC_DIR = ../build/rosstep-creator


SOURCES += main.cpp\
        mainwindow.cpp \
    step.cpp

HEADERS  += mainwindow.h \
    step.h

FORMS    += mainwindow.ui \
    step.ui

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += yaml-cpp
