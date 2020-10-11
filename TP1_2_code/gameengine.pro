QT       += core gui widgets

CONFIG += c++11
QMAKE_CXXFLAGS += -std=c++0x

TARGET = gameengine
TEMPLATE = app

SOURCES += main.cpp

SOURCES += \
    mainwidget.cpp \
    geometryengine.cpp

HEADERS += \
    mainwidget.h \
    geometryengine.h

RESOURCES += \
    shaders.qrc \
    textures.qrc

# install
target.path = $$[YOUR_PATH]
INSTALLS += target
