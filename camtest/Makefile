# The name of the compiler that we want to use
CC=g++

# List of directories where the compiler should look for header files
INCDIRS=-I/usr/local/include/opencv

# Compiler flags
CFLAGS=$(INCDIRS) -std=c++11

# List of directories where the linker should look for libraries
LIBDIRS=-L/usr/local/lib -L/usr/local/cuda/lib \

# List of libraries that should be linked into the final executable
# comment out "-lopencv_imgcodecs \" to run on Tim's TK1
LIBS=-lopencv_calib3d \
-lopencv_contrib \
-lopencv_core \
-lopencv_features2d \
-lopencv_flann \
-lopencv_gpu \
-lopencv_highgui \
-lopencv_imgproc \
-lopencv_imgcodecs \
-lopencv_legacy \
-lopencv_ml \
-lopencv_objdetect \
-lopencv_photo \
-lopencv_stitching \
-lopencv_superres \
-lopencv_ts \
-lopencv_video \
-lopencv_videoio \
-lopencv_videostab \
-lopencv_esm_panorama \
-lopencv_facedetect  \
-lopencv_imuvstab \
-lopencv_tegra \
-lopencv_vstab \
-lcufft -lnpps -lnppi -lnppc -lcudart -lrt -lpthread -lm -ldl

# Linker flags
LDFLAGS=$(LIBDIRS) $(LIBS)

# List of source files that should be compiled into the final executable program
SOURCES=camtest.cpp

# List of header files that should be included while compiling
HEADERS=

all: camtest

camtest: $(SOURCES) $(HEADERS)
	$(CC) $(SOURCES) $(CFLAGS) $(LDFLAGS) -o camtest

clean:
	rm -f *.o camtest

