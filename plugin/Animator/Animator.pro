QT       += core gui testlib

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

TEMPLATE = lib
DEFINES += ANIMATOR_LIBRARY
DEFINES += QT_DEPRECATED_WARNINGS
DESTDIR = ../../build


INCLUDEPATH += ../../build/config
INCLUDEPATH += ../../include
INCLUDEPATH += ../../

SOURCES += \
    AnimatorActivator.cpp \
    base_widget.cpp \
    glpainter.cpp \
    glwidget.cpp



HEADERS += \
    AnimatorActivator.h \
    base_widget.h \
    glpainter.h \
    glwidget.h

FORMS += \
    base_widget.ui


