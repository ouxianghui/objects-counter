#-------------------------------------------------
#
# Project created by QtCreator 2016-08-12T09:11:05
#
#-------------------------------------------------

QT       -= core

QT       -= gui

QT       -= qt

INCLUDEPATH += \
    #$$PWD/camport_linux \
    #$$PWD/camport_linux/include \
    $$PWD/include \
    $$PWD/library \
    /usr/include \
    /usr/include/opencv \
    /usr/include/opencv2

#TARGET = X2
CONFIG   += console
CONFIG   -= app_bundle
CONFIG += c++11


TEMPLATE = app


SOURCES += main.cpp \
    cvBlob/cvtrack.cpp \
    cvBlob/cvlabel.cpp \
    cvBlob/cvcontour.cpp \
    cvBlob/cvcolor.cpp \
    cvBlob/cvblob.cpp \
    cvBlob/cvaux.cpp \
    library/ComponentLabeling.cpp \
    library/BlobResult.cpp \
    library/BlobOperators.cpp \
    library/BlobContour.cpp \
    library/blob.cpp \
    Fitting.cpp \
    mser2.cpp \
    mser3.cpp \
    BlobCouting.cpp \
    BlobTracking.cpp \
    BlobCounter.cpp \
    BlobTracker.cpp \
    DepthBlobsExtracter.cpp \
    AdaptableBlobsExtracter.cpp

#LIBS += -L$$PWD/camport_linux2/lib_x64/ -lcamm
LIBS += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_features2d -lopencv_contrib -lopencv_photo -lopencv_legacy

HEADERS += \
    cvBlob/cvblob.h \
    library/ComponentLabeling.h \
    library/BlobResult.h \
    library/BlobOperators.h \
    library/BlobLibraryConfiguration.h \
    library/BlobContour.h \
    library/blob.h \
    Fitting.h \
    spline.h \
    mser2.hpp \
    mser3.hpp \
    BlobCouting.h \
    BlobTracking.h \
    BlobCounter.h \
    BlobTracker.h \
    DepthBlobsExtracter.h \
    AdaptableBlobsExtracter.h \
    IExtracter.h \
    persistence1d.hpp

CONFIG(debug, debug|release) {
DESTDIR = $$PWD/debug
TARGET = DepthCounterd
} else {
DESTDIR = $$PWD/release
TARGET = DepthCounterd
}

OTHER_FILES += \
    library/CMakeLists.txt

