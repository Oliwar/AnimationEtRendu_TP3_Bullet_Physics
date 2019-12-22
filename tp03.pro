#-------------------------------------------------
#
# Project created by QtCreator 2019-01-12T12:27:11
#
#-------------------------------------------------

QT       += core gui

CONFIG += c++14
CONFIG += link_pkgconfig
PKGCONFIG += bullet

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = tp03
TEMPLATE = app


SOURCES += main.cpp\
        boite.cpp \
        princ.cpp \
        glarea.cpp \
        sphere.cpp

HEADERS  += princ.h \
        boite.h \
        glarea.h \
        sphere.h

FORMS    += princ.ui

RESOURCES += \
    tp03.qrc

DISTFILES +=
