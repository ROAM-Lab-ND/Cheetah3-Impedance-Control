#-------------------------------------------------
#
# Project created by QtCreator 2020-12-09T17:29:46
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = LegControl
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
        main.cpp \
        controlgui.cpp

HEADERS += \
        controlgui.h \
    lcm-types/ecat_data_t.hpp \
    lcm-types/ecat_command_t.hpp \
    lcm-types/ecat_hard_enable_t.hpp \
    lcm-types/ecat_adl_status_t.hpp

FORMS += \
        controlgui.ui


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../usr/local/lib/release/ -llcm
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../usr/local/lib/debug/ -llcm
else:unix: LIBS += -L$$PWD/../../../../../../../usr/local/lib/ -llcm

INCLUDEPATH += $$PWD/../../../../../../../usr/local/include/lcm
DEPENDPATH += $$PWD/../../../../../../../usr/local/include/lcm

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../../usr/local/lib/release/liblcm.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../../usr/local/lib/debug/liblcm.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../../usr/local/lib/release/lcm.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../../usr/local/lib/debug/lcm.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../../../../../../usr/local/lib/liblcm.a
