QT       += core gui testlib

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

TEMPLATE = lib
DEFINES += SWARM_RL_EXPERIMENT_LIBRARY
DEFINES += QT_DEPRECATED_WARNINGS
DESTDIR = ../../build
QMAKE_CXXFLAGS += /arch:AVX
QMAKE_CXXFLAGS_DEBUG += /arch:AVX

INCLUDEPATH += ../../build/config
INCLUDEPATH += ../../include
INCLUDEPATH += ../../
INCLUDEPATH += ../../3rdlib


SOURCES += \
    swarm_rl_experimentActivator.cpp \
    main_stream.cpp \
    mypainter.cpp \
    simulator.cpp \



HEADERS += \
    swarm_rl_experimentActivator.h \
    main_stream.h \
    mypainter.h \
    simulator.h \


