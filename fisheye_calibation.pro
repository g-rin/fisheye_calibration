TEMPLATE = app
SOURCES += main.cpp
LIBS *= \
    -lopencv_core \
    -lopencv_calib3d \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_imgcodecs \
    -lpopt \

OTHER_FILES += CMakeLists.txt




